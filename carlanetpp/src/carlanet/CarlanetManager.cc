#include "CarlanetManager.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "inet/common/Units.h"
#include "inet/common/TimeTag_m.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/scenario/ScenarioManager.h"

using namespace omnetpp;
using namespace inet;
using nlohmann::json;

Define_Module(CarlanetManager);

namespace {
static std::unordered_map<std::string, double> g_lastSeenSimTime;

static int parseVehIndexOrDefault(const std::string& id, int fallback)
{
    if (id.rfind("veh", 0) == 0) {
        try {
            return std::stoi(id.substr(3));
        }
        catch (...) {
        }
    }
    return fallback;
}

static bool hasParSafe(cModule* module, const char* name)
{
    return module && module->findPar(name) >= 0;
}

static json parseObjectJsonOrEmpty(const std::string& text, const char* context)
{
    if (text.empty())
        return json::object();

    try {
        json j = json::parse(text);
        if (!j.is_object())
            throw cRuntimeError("[%s] expected JSON object", context);
        return j;
    }
    catch (const std::exception& e) {
        throw cRuntimeError("[%s] invalid JSON: %s", context, e.what());
    }
}

static json getLateSpawnActorConfigJson(cModule* manager)
{
    if (!hasParSafe(manager, "lateSpawnActorConfig"))
        return json::object();

    return parseObjectJsonOrEmpty(
        manager->par("lateSpawnActorConfig").stdstringValue(),
        "CarlanetManager lateSpawnActorConfig");
}

static json extractCarlaSpawnConfig(const json& cfg)
{
    // Preferred format:
    // {
    //   "carla": {...},
    //   "appl":  {...}
    // }
    if (cfg.is_object() && cfg.contains("carla") && cfg["carla"].is_object())
        return cfg["carla"];

    // Backward-compatible old format: raw CARLA actor config only.
    if (cfg.is_object() && !cfg.contains("appl"))
        return cfg;

    return json::object();
}

static json extractLateSpawnApplConfig(const json& cfg)
{
    if (cfg.is_object() && cfg.contains("appl") && cfg["appl"].is_object())
        return cfg["appl"];

    return json::object();
}

static bool looksLikePlaceholderActorPosition(const carla_api_base::actor_position& actor)
{
    const double posNorm =
        std::fabs(actor.position[0]) +
        std::fabs(actor.position[1]) +
        std::fabs(actor.position[2]);

    const double velNorm =
        std::fabs(actor.velocity[0]) +
        std::fabs(actor.velocity[1]) +
        std::fabs(actor.velocity[2]);

    const double rotNorm =
        std::fabs(actor.rotation[0]) +
        std::fabs(actor.rotation[1]) +
        std::fabs(actor.rotation[2]);

    return posNorm < 1e-9 && velNorm < 1e-9 && rotNorm < 1e-9;
}

static bool isIntegerParameterName(const std::string& name)
{
    return name == "node_id" ||
           name == "platoon_id" ||
           name == "leader_id" ||
           name == "join_position";
}

static bool isParentIdentityParameterName(const std::string& name)
{
    return name == "actor_id" ||
           name == "platoon_role" ||
           name == "node_id" ||
           name == "platoon_id" ||
           name == "leader_id" ||
           name == "initial_formation" ||
           name == "join_position";
}

static bool shouldSkipDynamicApplParameter(const std::string& name)
{
    // These are already pushed through the parent module before buildInside().
    if (isParentIdentityParameterName(name))
        return true;

    // beaconInterval is inherited from DemoBaseApplLayer and may not be mutable.
    // Let the normal INI/default value handle it.
    if (name == "beaconInterval")
        return true;

    return false;
}

static void setModuleParameterFromJson(cModule* module, const std::string& name, const json& value)
{
    if (!module)
        return;

    if (module->findPar(name.c_str()) < 0) {
        EV_WARN << "[CarlanetManager][setModuleParameterFromJson]"
                << " module=" << module->getFullPath()
                << " warning=unknown_parameter"
                << " name=" << name
                << "\n";
        return;
    }

    cPar& p = module->par(name.c_str());

    try {
        if (value.is_boolean()) {
            p.setBoolValue(value.get<bool>());
        }
        else if (value.is_number_integer() && isIntegerParameterName(name)) {
            p.setIntValue(value.get<int>());
        }
        else if (value.is_number()) {
            // Base SI units: seconds, meters, m/s, etc.
            p.setDoubleValue(value.get<double>());
        }
        else if (value.is_string()) {
            p.setStringValue(value.get<std::string>().c_str());
        }
        else {
            EV_WARN << "[CarlanetManager][setModuleParameterFromJson]"
                    << " module=" << module->getFullPath()
                    << " warning=unsupported_json_parameter_type"
                    << " name=" << name
                    << "\n";
        }
    }
    catch (const std::exception& e) {
        EV_WARN << "[CarlanetManager][setModuleParameterFromJson]"
                << " module=" << module->getFullPath()
                << " warning=set_parameter_failed"
                << " name=" << name
                << " error=\"" << e.what() << "\""
                << "\n";
    }
}

static void applyParentIdentityConfig(cModule* node, const json& applCfg)
{
    if (!node || !applCfg.is_object())
        return;

    for (auto it = applCfg.begin(); it != applCfg.end(); ++it) {
        if (!isParentIdentityParameterName(it.key()))
            continue;

        if (it.value().is_object() || it.value().is_array())
            continue;

        setModuleParameterFromJson(node, it.key(), it.value());
    }
}

static void applyDynamicApplConfig(cModule* appl, const json& applCfg)
{
    if (!appl || !applCfg.is_object())
        return;

    for (auto it = applCfg.begin(); it != applCfg.end(); ++it) {
        if (shouldSkipDynamicApplParameter(it.key()))
            continue;

        if (it.value().is_object() || it.value().is_array()) {
            EV_WARN << "[CarlanetManager][applyDynamicApplConfig]"
                    << " module=" << appl->getFullPath()
                    << " warning=skipped_nested_value"
                    << " name=" << it.key()
                    << "\n";
            continue;
        }

        setModuleParameterFromJson(appl, it.key(), it.value());
    }
}

} // namespace

CarlanetManager::CarlanetManager()
    : context(1)
    , socket(context, zmq::socket_type::req)
{
}

CarlanetManager::~CarlanetManager()
{
    if (simulationTimeStepEvent) {
        cancelAndDelete(simulationTimeStepEvent);
        simulationTimeStepEvent = nullptr;
    }

    try {
        if (zmqReady)
            socket.setsockopt(ZMQ_LINGER, 0);
    }
    catch (...) {
    }

    try {
        if (zmqReady)
            socket.close();
    }
    catch (...) {
    }

    try {
        if (zmqReady)
            context.close();
    }
    catch (...) {
    }
}

void CarlanetManager::finish()
{
    try {
        if (connected) {
            json request;
            request["message_type"] = "SIMULATION_FINISHED";
            request["timestamp"] = SIMTIME_DBL(simTime());

            sendToCarla(request);

            try {
                (void)receiveFromCarla(1.0);
            }
            catch (...) {
            }
        }
    }
    catch (...) {
    }

    try {
        if (zmqReady)
            socket.setsockopt(ZMQ_LINGER, 0);
    }
    catch (...) {
    }

    try {
        if (zmqReady)
            socket.close();
    }
    catch (...) {
    }

    try {
        if (zmqReady)
            context.close();
    }
    catch (...) {
    }

    cSimpleModule::finish();
}

void CarlanetManager::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        protocol = par("protocol").stringValue();
        host = par("host").stringValue();
        port = par("port").intValue();
        timeout_ms = par("communicationTimeoutms");
        simulationTimeStep = par("simulationTimeStep");

        networkActiveModuleType = par("networkActiveModuleType").stringValue();
        networkPassiveModuleType = par("networkPassiveModuleType").stringValue();

        EV_INFO << "CarlanetManager params: "
                << protocol << "://" << host << ":" << port
                << " step=" << simulationTimeStep
                << "s timeout=" << timeout_ms << "ms\n";
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        try {
            connect();
            initializeCarla();
            connected = true;

            simulationTimeStepEvent = new cMessage("simulationTimeStep");
            simulationTimeStepEvent->setSchedulingPriority(-1);

            scheduleAt(simTime() + simulationTimeStep, simulationTimeStepEvent);
        }
        catch (const std::exception& e) {
            throw cRuntimeError("CarlanetManager init failed: %s", e.what());
        }
    }

    if (stage == 21) {
        if (hasParSafe(this, "enableLateSpawn") && par("enableLateSpawn").boolValue()) {
            auto* m = new cMessage("lateSpawnTimer");
            m->setKind(1001);

            double lateSpawnTime = hasParSafe(this, "lateSpawnTime")
                ? par("lateSpawnTime").doubleValue()
                : 0.0;

            scheduleAt(simTime() + SimTime(lateSpawnTime), m);
        }
    }
}

void CarlanetManager::registerMobilityModule(CarlaInetMobility* mobilityModule)
{
    cModule* node = mobilityModule ? mobilityModule->getParentModule() : nullptr;
    if (!node)
        throw cRuntimeError("[CarlanetManager][registerMobilityModule] mobility has no parent module");

    std::string actorId;
    if (node->hasPar("actor_id"))
        actorId = node->par("actor_id").stringValue();

    if (actorId.empty()) {
        throw cRuntimeError(
            "[CarlanetManager][registerMobilityModule] required parameter actor_id missing/empty on %s",
            node->getFullPath().c_str());
    }

    auto it = modulesToTrack.find(actorId);
    if (it != modulesToTrack.end() && it->second != mobilityModule) {
        throw cRuntimeError(
            "[CarlanetManager][registerMobilityModule] duplicate actor_id='%s' on %s",
            actorId.c_str(),
            node->getFullPath().c_str());
    }

    modulesToTrack[actorId] = mobilityModule;

    int idx = getOrAssignGateIndex(actorId);
    connectNodeToControlIn(node, idx);

    EV_INFO << "[CarlanetManager][registerMobilityModule]"
            << " hop=MOBILITY_TO_MANAGER"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actorId
            << " gate_index=" << idx
            << " node=" << node->getFullPath()
            << "\n";
}

const std::map<std::string, cValue>& CarlanetManager::getExtraInitParams()
{
    return check_and_cast<cValueMap*>(par("extraInitParams").objectValue())->getFields();
}

void CarlanetManager::initializeCarla()
{
    std::list<carla_api_base::init_actor> movingActorList;
    std::vector<std::string> actorIds;
    actorIds.reserve(modulesToTrack.size());

    for (auto& kv : modulesToTrack) {
        carla_api_base::init_actor actor;
        actor.actor_id = kv.first;
        actor.actor_type = kv.second->getCarlaActorType();
        actor.actor_configuration = kv.second->getCarlaActorConfiguration()->getFields();

        movingActorList.push_back(actor);
        actorIds.push_back(kv.first);
    }

    std::sort(actorIds.begin(), actorIds.end());

    auto simTimeLimitStr = getEnvir()->getConfigEx()->getConfigValue("sim-time-limit");

    carla_api::init initMsg;
    initMsg.run_id = getEnvir()->getConfigEx()->getVariable(CFGVAR_RUNID);

    int carlaSeed = par("carlaSeed").intValue();
    if (carlaSeed < 0)
        carlaSeed = std::stoi(getEnvir()->getConfigEx()->getVariable(CFGVAR_SEEDSET));

    initMsg.carla_configuration.seed = carlaSeed;
    initMsg.carla_configuration.carla_timestep = simulationTimeStep;
    initMsg.carla_configuration.sim_time_limit = simTimeLimitStr ? std::stod(simTimeLimitStr) : -1.0;
    initMsg.moving_actors = movingActorList;
    initMsg.user_defined = getExtraInitParams();
    initMsg.timestamp = simTime().dbl();

    std::string mapName = "?";
    try {
        const auto& extra = getExtraInitParams();
        auto it = extra.find("map");
        if (it != extra.end())
            mapName = it->second.stdstringValue();
    }
    catch (...) {
    }

    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss << std::setprecision(6);

        oss << "[CarlanetManager][initializeCarla]"
            << " hop=MANAGER_TO_PY"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " message_type=INIT"
            << " run_id=" << initMsg.run_id
            << " map=" << mapName
            << " carla_seed=" << carlaSeed
            << " carla_timestep=" << simulationTimeStep
            << " sim_time_limit=" << initMsg.carla_configuration.sim_time_limit
            << " actors=";

        for (size_t i = 0; i < actorIds.size(); ++i) {
            if (i)
                oss << ",";
            oss << actorIds[i];
        }

        oss << "\n";
        EV_INFO << oss.str();
    }

    sendToCarla(json(initMsg));

    carla_api::init_completed response = receiveFromCarla<carla_api::init_completed>(100.0);
    initialTimestamp = simTime();

    EV_INFO << "[CarlanetManager][initializeCarla]"
            << " hop=PY_TO_MANAGER"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " message_type=INIT_COMPLETED"
            << " actors_count=" << response.actor_positions.size()
            << "\n";

    updateNodesPosition(response.actor_positions);
}

void CarlanetManager::doSimulationTimeStep()
{
    for (auto& userMsgJson : pendingUserMsgs) {
        json ud;

        try {
            ud = json::parse(userMsgJson);
        }
        catch (...) {
            ud = json::object();
        }

        try {
            std::string actor;
            double desiredAcceleration = 0.0;
            double desiredSpeed = 0.0;
            double throttle = 0.0;
            double brake = 0.0;
            bool hasControl = false;

            if (ud.is_object() && ud.value("msg_type", "") == "CONTROL") {
                actor = ud.value("actor_id", "");
                if (ud.contains("ctrl") && ud["ctrl"].is_object()) {
                    const auto& ctrl = ud["ctrl"];
                    desiredAcceleration = ctrl.value("desired_acceleration", 0.0);
                    desiredSpeed = ctrl.value("desired_speed", 0.0);
                    throttle = ctrl.value("throttle", 0.0);
                    brake = ctrl.value("brake", 0.0);
                    hasControl = ctrl.value("has_control", false);
                }
            }

            EV_INFO << "[CarlanetManager][doSimulationTimeStep]"
                    << " hop=MANAGER_TO_PY"
                    << " simulation_time=" << SIMTIME_DBL(simTime())
                    << " actor=" << (actor.empty() ? "?" : actor)
                    << " desired_speed=" << desiredSpeed
                    << " desired_acceleration=" << desiredAcceleration
                    << " throttle=" << throttle
                    << " brake=" << brake
                    << " has_control=" << (hasControl ? 1 : 0)
                    << "\n";
        }
        catch (...) {
        }

        carla_api::generic_message gen;
        gen.timestamp = SIMTIME_DBL(simTime());
        gen.user_defined = ud;

        sendToCarla(json(gen));
        (void)receiveFromCarla<carla_api::generic_response>();
    }

    pendingUserMsgs.clear();

    carla_api::simulation_step stepMsg;
    stepMsg.carla_timestep = simulationTimeStep;
    stepMsg.timestamp = simTime().dbl();

    sendToCarla(json(stepMsg));

    carla_api::updated_postion response = receiveFromCarla<carla_api::updated_postion>();
    updateNodesPosition(response.actor_positions);
}

void CarlanetManager::updateNodesPosition(std::list<carla_api_base::actor_position> actorList)
{
    std::set<std::string> knownActorIds;
    for (const auto& entry : modulesToTrack)
        knownActorIds.insert(entry.first);

    const double now = SIMTIME_DBL(simTime());

    for (const auto& actor : actorList) {
        const double px = actor.position[0];
        const double py = actor.position[1];
        const double pz = actor.position[2];

        const double vx = actor.velocity[0];
        const double vy = actor.velocity[1];
        const double vz = actor.velocity[2];

        const double ax = actor.acceleration[0];
        const double ay = actor.acceleration[1];
        const double az = actor.acceleration[2];

        const double wx = actor.angular_velocity[0];
        const double wy = actor.angular_velocity[1];
        const double wz = actor.angular_velocity[2];

        const double alphax = actor.angular_acceleration[0];
        const double alphay = actor.angular_acceleration[1];
        const double alphaz = actor.angular_acceleration[2];

        const double pitch = actor.rotation[0];
        const double yaw = actor.rotation[1];
        const double roll = actor.rotation[2];

        const double speed = std::sqrt(vx * vx + vy * vy + vz * vz);

        EV_INFO
            << "[CarlanetManager][updateNodesPosition]"
            << " event=REAL_POSITION"
            << " source=CARLA"
            << " simulation_time=" << now
            << " actor=" << actor.actor_id
            << " x=" << px
            << " y=" << py
            << " z=" << pz
            << " vx=" << vx
            << " vy=" << vy
            << " vz=" << vz
            << " ax=" << ax
            << " ay=" << ay
            << " az=" << az
            << " angular_vx=" << wx
            << " angular_vy=" << wy
            << " angular_vz=" << wz
            << " angular_ax=" << alphax
            << " angular_ay=" << alphay
            << " angular_az=" << alphaz
            << " speed=" << speed
            << " pitch=" << pitch
            << " yaw=" << yaw
            << " roll=" << roll
            << " is_net_active=" << (actor.is_net_active ? 1 : 0)
            << "\n";

        auto it = modulesToTrack.find(actor.actor_id);

        if (it == modulesToTrack.end()) {
            createAndInitializeActor(actor);
            g_lastSeenSimTime[actor.actor_id] = now;
            knownActorIds.erase(actor.actor_id);
            continue;
        }

        if (!it->second || !it->second->getParentModule()) {
            EV_WARN << "[CarlanetManager][updateNodesPosition]"
                    << " actor=" << actor.actor_id
                    << " warning=null_or_detached_mobility"
                    << "\n";
            continue;
        }

        Coord position(px, py, pz);
        Coord velocity(vx, vy, vz);
        Coord acceleration(ax, ay, az);

        Quaternion rotation = Quaternion(EulerAngles(
            inet::units::values::rad(yaw * M_PI / 180.0),
            inet::units::values::rad(pitch * M_PI / 180.0),
            inet::units::values::rad(roll * M_PI / 180.0)
        ));

        Quaternion angularVelocity = Quaternion(EulerAngles(
            inet::units::values::rad(wy * M_PI / 180.0),
            inet::units::values::rad(wx * M_PI / 180.0),
            inet::units::values::rad(wz * M_PI / 180.0)
        ));

        Quaternion angularAcceleration = Quaternion(EulerAngles(
            inet::units::values::rad(alphay * M_PI / 180.0),
            inet::units::values::rad(alphax * M_PI / 180.0),
            inet::units::values::rad(alphaz * M_PI / 180.0)
        ));

        const bool firstTime = (g_lastSeenSimTime.find(actor.actor_id) == g_lastSeenSimTime.end());

        if (firstTime) {
            it->second->preInitialize(
                position,
                velocity,
                acceleration,
                rotation,
                angularVelocity,
                angularAcceleration);
        }
        else {
            it->second->nextPosition(
                position,
                velocity,
                acceleration,
                rotation,
                angularVelocity,
                angularAcceleration);
        }

        g_lastSeenSimTime[actor.actor_id] = now;
        knownActorIds.erase(actor.actor_id);
    }

    // Reaping intentionally disabled while stabilizing late-spawn and join behavior.
    (void)knownActorIds;
}

void CarlanetManager::connect()
{
    try {
        socket.setsockopt(ZMQ_RCVTIMEO, timeout_ms);
        socket.setsockopt(ZMQ_SNDTIMEO, timeout_ms);

        std::string addr = protocol + "://" + host + ":" + std::to_string(port);

        EV_INFO << "Connecting to pyCARLANeT at " << addr << "\n";

        socket.connect(addr);
        zmqReady = true;
    }
    catch (const std::exception& e) {
        throw cRuntimeError("CarlanetManager ZMQ connect failed: %s", e.what());
    }
}

void CarlanetManager::handleMessage(cMessage* msg)
{
    if (msg->isSelfMessage()) {
        if (msg->getKind() == 1001) {
            delete msg;
            handleLateSpawn();
            return;
        }

        if (msg == simulationTimeStepEvent) {
            doSimulationTimeStep();
            scheduleAt(simTime() + simulationTimeStep, msg);
            return;
        }

        delete msg;
        return;
    }

    if (msg->arrivedOn("controlIn")) {
        std::string js = msg->hasPar("json") ? msg->par("json").stringValue() : "{}";

        try {
            json ud = json::parse(js);

            std::string actor;
            double desiredAcceleration = 0.0;
            double desiredSpeed = 0.0;
            double throttle = 0.0;
            double brake = 0.0;
            bool hasControl = false;

            if (ud.is_object() && ud.value("msg_type", "") == "CONTROL") {
                actor = ud.value("actor_id", "");
                if (ud.contains("ctrl") && ud["ctrl"].is_object()) {
                    const auto& ctrl = ud["ctrl"];
                    desiredAcceleration = ctrl.value("desired_acceleration", 0.0);
                    desiredSpeed = ctrl.value("desired_speed", 0.0);
                    throttle = ctrl.value("throttle", 0.0);
                    brake = ctrl.value("brake", 0.0);
                    hasControl = ctrl.value("has_control", false);
                }
            }

            EV_INFO << "[CarlanetManager][handleMessage]"
                    << " hop=BRIDGE_TO_MANAGER"
                    << " simulation_time=" << SIMTIME_DBL(simTime())
                    << " actor=" << (actor.empty() ? "?" : actor)
                    << " desired_speed=" << desiredSpeed
                    << " desired_acceleration=" << desiredAcceleration
                    << " throttle=" << throttle
                    << " brake=" << brake
                    << " has_control=" << (hasControl ? 1 : 0)
                    << "\n";
        }
        catch (...) {
            EV_WARN << "[CarlanetManager][handleMessage]"
                    << " hop=BRIDGE_TO_MANAGER"
                    << " simulation_time=" << SIMTIME_DBL(simTime())
                    << " warning=parse_error=1"
                    << "\n";
        }

        pendingUserMsgs.emplace_back(std::move(js));
        delete msg;
        return;
    }

    delete msg;
}

void CarlanetManager::handleLateSpawn()
{
    carla_api::spawn_actor req;
    req.timestamp = simTime().dbl();

    req.actor.actor_id = hasParSafe(this, "lateSpawnActorId")
        ? par("lateSpawnActorId").stdstringValue()
        : "veh4";

    req.actor.actor_type = hasParSafe(this, "lateSpawnActorType")
        ? par("lateSpawnActorType").stdstringValue()
        : "vehicle.tesla.model3";

    const json fullCfg = getLateSpawnActorConfigJson(this);
    req.actor.actor_configuration = extractCarlaSpawnConfig(fullCfg);

    EV_INFO << "[CarlanetManager][handleLateSpawn]"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << req.actor.actor_id
            << " carla_cfg=" << req.actor.actor_configuration.dump()
            << "\n";

    sendToCarla(json(req));

    carla_api::spawn_completed resp = receiveFromCarla<carla_api::spawn_completed>(100.0);

    if (resp.status != 0) {
        EV_ERROR << "[CarlanetManager][handleLateSpawn]"
                 << " late_spawn_failed=1"
                 << " actor=" << req.actor.actor_id
                 << " error=\"" << resp.error << "\""
                 << "\n";
        return;
    }

    std::list<carla_api_base::actor_position> usablePositions;

    for (const auto& pos : resp.actor_positions) {
        if (looksLikePlaceholderActorPosition(pos)) {
            EV_WARN << "[CarlanetManager][handleLateSpawn]"
                    << " actor=" << pos.actor_id
                    << " warning=deferred_placeholder_spawn_position"
                    << " reason=zero_pose_from_spawn_completed"
                    << "\n";
            continue;
        }

        usablePositions.push_back(pos);
    }

    if (!usablePositions.empty()) {
        updateNodesPosition(usablePositions);
    }
    else {
        EV_INFO << "[CarlanetManager][handleLateSpawn]"
                << " actor=" << req.actor.actor_id
                << " action=defer_omnet_actor_creation_until_next_UPDATED_POSITIONS"
                << "\n";
    }

    EV_INFO << "[CarlanetManager][handleLateSpawn]"
            << " late_spawn_completed=1"
            << " actor=" << req.actor.actor_id
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << "\n";
}

void CarlanetManager::createAndInitializeActor(const carla_api_base::actor_position& newActor)
{
    const std::string& moduleTypeName =
        newActor.is_net_active ? networkActiveModuleType : networkPassiveModuleType;

    cModule* root = getSimulation()->getSystemModule();
    cModuleType* actorType = cModuleType::get(moduleTypeName.c_str());

    if (!actorType) {
        throw cRuntimeError(
            "[CarlanetManager][createAndInitializeActor] module type not found: %s",
            moduleTypeName.c_str());
    }

    const int idx = getOrAssignGateIndex(newActor.actor_id);
    const int parsedVehIndex = parseVehIndexOrDefault(newActor.actor_id, idx);

    std::string lateId = hasParSafe(this, "lateSpawnActorId")
        ? par("lateSpawnActorId").stdstringValue()
        : "";

    const bool isLateJoiner = (!lateId.empty() && newActor.actor_id == lateId);

    const json fullCfg = isLateJoiner
        ? getLateSpawnActorConfigJson(this)
        : json::object();

    const json applCfg = isLateJoiner
        ? extractLateSpawnApplConfig(fullCfg)
        : json::object();

    root->setSubmoduleVectorSize("actors", idx + 1);

    cModule* node = actorType->create("actors", root, idx);

    if (!node) {
        throw cRuntimeError(
            "[CarlanetManager][createAndInitializeActor] failed to create module type: %s",
            moduleTypeName.c_str());
    }

    if (!node->hasPar("actor_id")) {
        throw cRuntimeError(
            "[CarlanetManager][createAndInitializeActor] actor module type must define actor_id parameter on %s",
            node->getFullPath().c_str());
    }

    // ---------------------------------------------------------------------
    // Parent-level identity/topology params must be set before finalize/build.
    // The appl submodule defaults read these from parent.*.
    // ---------------------------------------------------------------------
    node->par("actor_id").setStringValue(newActor.actor_id.c_str());

    if (node->hasPar("platoon_role"))
        node->par("platoon_role").setStringValue(isLateJoiner ? "joiner" : "follower");

    if (node->hasPar("node_id"))
        node->par("node_id").setIntValue(parsedVehIndex);

    if (node->hasPar("platoon_id"))
        node->par("platoon_id").setIntValue(0);

    if (node->hasPar("leader_id"))
        node->par("leader_id").setIntValue(0);

    if (node->hasPar("initial_formation"))
        node->par("initial_formation").setStringValue("");

    if (node->hasPar("join_position"))
        node->par("join_position").setIntValue(isLateJoiner ? parsedVehIndex : -1);

    if (isLateJoiner)
        applyParentIdentityConfig(node, applCfg);

    node->finalizeParameters();
    node->buildInside();

    // ---------------------------------------------------------------------
    // App-only params must be applied after buildInside() because the appl
    // submodule does not exist before then, but before callInitialize().
    // ---------------------------------------------------------------------
    cModule* appl = node->getSubmodule("appl");

    if (appl && isLateJoiner) {
        applyDynamicApplConfig(appl, applCfg);
    }
    else if (!appl) {
        EV_WARN << "[CarlanetManager][createAndInitializeActor]"
                << " actor=" << newActor.actor_id
                << " warning=missing_appl_submodule"
                << "\n";
    }

    connectNodeToControlIn(node, idx);

    Coord position(
        newActor.position[0],
        newActor.position[1],
        newActor.position[2]);

    Coord velocity(
        newActor.velocity[0],
        newActor.velocity[1],
        newActor.velocity[2]);

    Coord acceleration(
        newActor.acceleration[0],
        newActor.acceleration[1],
        newActor.acceleration[2]);

    Quaternion rotation = Quaternion(EulerAngles(
        inet::units::values::rad(newActor.rotation[1] * M_PI / 180.0),
        inet::units::values::rad(newActor.rotation[0] * M_PI / 180.0),
        inet::units::values::rad(newActor.rotation[2] * M_PI / 180.0)
    ));

    Quaternion angularVelocity = Quaternion(EulerAngles(
        inet::units::values::rad(newActor.angular_velocity[1] * M_PI / 180.0),
        inet::units::values::rad(newActor.angular_velocity[0] * M_PI / 180.0),
        inet::units::values::rad(newActor.angular_velocity[2] * M_PI / 180.0)
    ));

    Quaternion angularAcceleration = Quaternion(EulerAngles(
        inet::units::values::rad(newActor.angular_acceleration[1] * M_PI / 180.0),
        inet::units::values::rad(newActor.angular_acceleration[0] * M_PI / 180.0),
        inet::units::values::rad(newActor.angular_acceleration[2] * M_PI / 180.0)
    ));

    auto* mobility = check_and_cast<CarlaInetMobility*>(node->getSubmodule("mobility"));

    modulesToTrack[newActor.actor_id] = mobility;

    mobility->preInitialize(
        position,
        velocity,
        acceleration,
        rotation,
        angularVelocity,
        angularAcceleration);

    auto* notification = new inet::cPreModuleInitNotification();
    notification->module = node;
    root->emit(POST_MODEL_CHANGE, notification, nullptr);

    node->scheduleStart(simTime());
    node->callInitialize();

    EV_INFO << "[CarlanetManager][createAndInitializeActor]"
            << " hop=MANAGER_TO_OMNET"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << newActor.actor_id
            << " idx=" << idx
            << " type=" << moduleTypeName
            << " role=" << (isLateJoiner ? "joiner" : "follower")
            << " node_id=" << parsedVehIndex
            << " platoon_id=" << (node->hasPar("platoon_id") ? node->par("platoon_id").intValue() : 0)
            << " leader_id=" << (node->hasPar("leader_id") ? node->par("leader_id").intValue() : 0)
            << " join_position=" << (node->hasPar("join_position") ? node->par("join_position").intValue() : -1)
            << " applied_late_appl_config=" << (isLateJoiner ? 1 : 0)
            << " node=" << node->getFullPath()
            << "\n";
}

void CarlanetManager::destroyActor(std::string actorId)
{
    auto it = modulesToTrack.find(actorId);
    if (it == modulesToTrack.end())
        return;

    cModule* node = it->second ? it->second->getParentModule() : nullptr;

    auto gi = gateIndexByActor.find(actorId);
    if (gi != gateIndexByActor.end()) {
        disconnectNodeFromControlIn(node, gi->second);
        gateIndexByActor.erase(gi);
    }

    if (node) {
        node->callFinish();
        node->deleteModule();
    }

    modulesToTrack.erase(it);
}

int CarlanetManager::getOrAssignGateIndex(const std::string& actorId)
{
    auto it = gateIndexByActor.find(actorId);
    if (it != gateIndexByActor.end())
        return it->second;

    int idx = nextGateIndex++;
    ensureControlInSize(nextGateIndex);
    gateIndexByActor.emplace(actorId, idx);

    return idx;
}

void CarlanetManager::ensureControlInSize(int minSize)
{
    int cur = gateSize("controlIn");
    if (cur < minSize)
        setGateSize("controlIn", minSize);
}

void CarlanetManager::connectNodeToControlIn(cModule* node, int idx)
{
    if (!node)
        return;

    ensureControlInSize(idx + 1);

    int gateId = node->findGate("toManager");
    if (gateId < 0)
        return;

    cGate* src = node->gate(gateId);
    cGate* dst = gate("controlIn", idx);

    if (src && dst && !src->isConnected())
        src->connectTo(dst);
}

void CarlanetManager::disconnectNodeFromControlIn(cModule* node, int idx)
{
    if (!node)
        return;

    int gateId = node->findGate("toManager");
    if (gateId >= 0) {
        cGate* src = node->gate(gateId);
        if (src && src->isConnected())
            src->disconnect();
    }

    if (idx >= 0 && idx < gateSize("controlIn")) {
        cGate* dst = gate("controlIn", idx);
        if (dst && dst->isConnected())
            dst->disconnect();
    }
}

void CarlanetManager::sendToCarla(const json& request)
{
    if (!zmqReady)
        throw cRuntimeError("ZMQ socket not ready");

    const std::string payload = request.dump();

    EV_INFO << "[ZMQ][RAW_SENT] " << payload << "\n";

    zmq::message_t msg(payload.size());
    std::memcpy(msg.data(), payload.data(), payload.size());

    try {
        socket.send(msg, zmq::send_flags::none);
    }
    catch (const std::exception& e) {
        throw cRuntimeError("[CarlanetManager][sendToCarla] ZMQ send failed: %s", e.what());
    }
}

json CarlanetManager::receiveFromCarla(double timeoutFactor)
{
    if (!zmqReady)
        throw cRuntimeError("ZMQ socket not ready");

    int recvTimeoutMs = std::max(4000, int(timeout_ms * timeoutFactor));
    socket.setsockopt(ZMQ_RCVTIMEO, recvTimeoutMs);

    zmq::message_t reply{};

    try {
        if (!socket.recv(reply, zmq::recv_flags::none))
            throw std::runtime_error("CARLA timeout");
    }
    catch (const std::exception& e) {
        throw cRuntimeError("[CarlanetManager][receiveFromCarla] ZMQ receive failed: %s", e.what());
    }

    std::string replyString(static_cast<char*>(reply.data()), reply.size());

    EV_INFO << "[ZMQ][RAW_RECEIVED] " << replyString << "\n";

    json jsonResp;

    try {
        jsonResp = json::parse(replyString);
    }
    catch (const std::exception& e) {
        throw cRuntimeError("[CarlanetManager][receiveFromCarla] invalid JSON: %s", e.what());
    }

    if (jsonResp.contains("simulation_status")) {
        switch (jsonResp["simulation_status"].get<int>()) {
            case SIM_STATUS_FINISHED_OK:
            case SIM_STATUS_FINISHED_ACCIDENT:
            case SIM_STATUS_FINISHED_TIME_LIMIT:
                endSimulation();
                break;

            case SIM_STATUS_ERROR:
                throw std::runtime_error("Communication error. Wrong message sequence!");

            default:
                break;
        }
    }

    return jsonResp;
}