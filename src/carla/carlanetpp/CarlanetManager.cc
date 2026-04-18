#include "CarlanetManager.h"

#include <stdexcept>
#include <algorithm>
#include <set>
#include <cmath>
#include <string>
#include <unordered_map>
#include <sstream>
#include <iomanip>


#include "inet/common/Units.h"        // <-- for inet::units::values::rad
#include "inet/common/TimeTag_m.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/packet/Packet.h"
#include "inet/common/scenario/ScenarioManager.h"

using namespace omnetpp;
using namespace inet;
using nlohmann::json;

Define_Module(CarlanetManager);

namespace { static std::unordered_map<std::string, double> g_lastSeenSimTime; }
// keep reaping disabled while stabilizing
static constexpr double kActorMissingGraceSeconds = 1e9;

CarlanetManager::CarlanetManager() : context(1), socket(context, zmq::socket_type::req) {}
CarlanetManager::~CarlanetManager() {
    if (simulationTimeStepEvent) { cancelAndDelete(simulationTimeStepEvent); simulationTimeStepEvent = nullptr; }
    try { if (zmqReady) socket.setsockopt(ZMQ_LINGER, 0); } catch (...) {}
    try { if (zmqReady) socket.close(); } catch (...) {}
    try { if (zmqReady) context.close(); } catch (...) {}
}

void CarlanetManager::finish() {
    try {
        if (connected) {
            json request; request["message_type"]="SIMULATION_FINISHED"; request["timestamp"]=SIMTIME_DBL(simTime());
            sendToCarla(request);
            try { (void)receiveFromCarla(1.0); } catch (...) {}
        }
    } catch (...) {}
    try { if (zmqReady) socket.setsockopt(ZMQ_LINGER, 0); } catch (...) {}
    try { if (zmqReady) socket.close(); } catch (...) {}
    try { if (zmqReady) context.close(); } catch (...) {}
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

        networkActiveModuleType  = par("networkActiveModuleType").stringValue();
        networkPassiveModuleType = par("networkPassiveModuleType").stringValue();

        EV_INFO << "CarlanetManager params: " << protocol << "://" << host << ":" << port
                << " step=" << simulationTimeStep << "s timeout=" << timeout_ms << "ms\n";
    }
    else if (stage == INITSTAGE_APPLICATION_LAYER) {
        try {
            connect();
            initializeCarla();
            connected = true;

            simulationTimeStepEvent = new cMessage("simulationTimeStep");
            simulationTimeStepEvent->setSchedulingPriority(-1); // run after normal messages at same simtime

            scheduleAt(simTime() + simulationTimeStep, simulationTimeStepEvent);
        } catch (const std::exception& e) {
            throw cRuntimeError("CarlanetManager init failed: %s", e.what());
        }
    }
    if (stage == 21) {
        if (par("enableLateSpawn").boolValue()) {
            auto* m = new cMessage("lateSpawnTimer");
            m->setKind(1001);
            scheduleAt(simTime() + omnetpp::SimTime(par("lateSpawnTime").doubleValue()), m);
        }
    }
}

static int parseVehIndexOrDefault(const std::string& id, int fallback)
{
    if (id.rfind("veh", 0) == 0) {
        try { return std::stoi(id.substr(3)); } catch (...) {}
    }
    return fallback;
}

void CarlanetManager::registerMobilityModule(CarlaInetMobility *mobilityModule){
    cModule* node = mobilityModule->getParentModule();

    std::string actor_id;
    if (node->hasPar("actor_id")) actor_id = node->par("actor_id").stringValue();
    if (actor_id.empty())
        throw cRuntimeError("[CarlanetManager][registerMobilityModule] required parameter actor_id missing/empty on %s", node->getFullPath().c_str());

    auto it = modulesToTrack.find(actor_id);
    if (it != modulesToTrack.end() && it->second != mobilityModule) {
        throw cRuntimeError("[CarlanetManager][registerMobilityModule] duplicate actor_id='%s' on %s (already used by %s)",
            actor_id.c_str(),
            node->getFullPath().c_str(),
            it->second ? it->second->getParentModule()->getFullPath().c_str() : "<null>");
    }

    modulesToTrack[actor_id] = mobilityModule;

    int idx = getOrAssignGateIndex(actor_id);
    connectNodeToControlIn(node, idx);

    {
        std::ostringstream oss;
        oss << "[CarlanetManager][registerMobilityModule]"
            << " hop=MOBILITY_TO_MANAGER"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " gate_index=" << idx
            << " node=" << node->getFullPath()
            << "\n";
        EV_INFO << oss.str();
    }
}

void CarlanetManager::initializeCarla(){
    std::list<carla_api_base::init_actor> movingActorList;
    std::vector<std::string> actorIds;
    actorIds.reserve(modulesToTrack.size());

    for (auto& kv : modulesToTrack){
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
    if (carlaSeed < 0) {
        carlaSeed = std::stoi(getEnvir()->getConfigEx()->getVariable(CFGVAR_SEEDSET));
    }

    initMsg.carla_configuration.seed = carlaSeed;
    initMsg.carla_configuration.carla_timestep = simulationTimeStep; // NOTE: simulationTimeStep is a double in your build
    initMsg.carla_configuration.sim_time_limit = simTimeLimitStr ? std::stod(simTimeLimitStr) : -1.0;
    initMsg.moving_actors = movingActorList;
    initMsg.user_defined = getExtraInitParams();
    initMsg.timestamp = simTime().dbl();

    // best-effort map extraction for logs (from extraInitParams)
    std::string mapName = "";
    try {
        const auto& extra = getExtraInitParams();
        auto it = extra.find("map");
        if (it != extra.end()) mapName = it->second.stdstringValue();
    } catch (...) {}

    // platform-style log (Manager -> Python)
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss << std::setprecision(6);

        oss << "[CarlanetManager][initializeCarla]"
            << " hop=MANAGER_TO_PY"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " message_type=INIT"
            << " run_id=" << initMsg.run_id
            << " map=" << (mapName.empty() ? "?" : mapName)
            << " carla_seed=" << carlaSeed
            << " carla_timestep=" << simulationTimeStep
            << " sim_time_limit=" << initMsg.carla_configuration.sim_time_limit
            << " actors=";

        for (size_t i = 0; i < actorIds.size(); ++i) {
            if (i) oss << ",";
            oss << actorIds[i];
        }
        oss << "\n";

        EV_INFO << oss.str();
    }

    sendToCarla(json(initMsg));

    carla_api::init_completed response = receiveFromCarla<carla_api::init_completed>(100.0);
    initialTimestamp = simTime();

    // platform-style log (Python -> Manager)
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss << std::setprecision(6);

        oss << "[CarlanetManager][initializeCarla]"
            << " hop=PY_TO_MANAGER"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " message_type=INIT_COMPLETED"
            << " actors_count=" << response.actor_positions.size()
            << "\n";

        EV_INFO << oss.str();
    }

    updateNodesPosition(response.actor_positions);
}

const std::map<std::string,cValue>& CarlanetManager::getExtraInitParams(){
    return check_and_cast<cValueMap*>(par("extraInitParams").objectValue())->getFields();
}

void CarlanetManager::doSimulationTimeStep()
{
    // 1) Flush CONTROL first (Manager -> Python) so CARLA applies it on upcoming step.
    for (auto &userMsgJson : pendingUserMsgs) {
        json ud;
        try { ud = json::parse(userMsgJson); }
        catch (...) { ud = json::object(); }

        // platform-style control log (Manager -> Python)
        try {
            std::string actor;
            double a = 0.0, v = 0.0;
            bool has_control = false;

            if (ud.is_object() && ud.value("msg_type", "") == "CONTROL") {
                actor = ud.value("actor_id", "");
                if (ud.contains("ctrl") && ud["ctrl"].is_object()) {
                    a = ud["ctrl"].value("desired_acceleration", 0.0);
                    v = ud["ctrl"].value("desired_speed", 0.0);
                    has_control = ud["ctrl"].value("has_control", false);
                }
            }

            std::ostringstream oss;
            oss.setf(std::ios::fixed);
            oss << std::setprecision(6);

            oss << "[CarlanetManager][doSimulationTimeStep]"
                << " hop=MANAGER_TO_PY"
                << " simulation_time=" << SIMTIME_DBL(simTime())
                << " actor=" << (actor.empty() ? "?" : actor)
                << " desired_speed=" << v
                << " desired_acceleration=" << a
                << " has_control=" << (has_control ? 1 : 0)
                << "\n";

            EV_INFO << oss.str();
        } catch (...) {
            // keep silent; we already tolerate malformed
        }

        carla_api::generic_message gen;
        gen.timestamp = SIMTIME_DBL(simTime());
        gen.user_defined = ud;

        sendToCarla(json(gen));
        (void)receiveFromCarla<carla_api::generic_response>();
    }
    pendingUserMsgs.clear();

    // 2) Now step CARLA.
    carla_api::simulation_step stepMsg;
    stepMsg.carla_timestep = simulationTimeStep;
    stepMsg.timestamp = simTime().dbl();

    sendToCarla(json(stepMsg));
    carla_api::updated_postion response = receiveFromCarla<carla_api::updated_postion>();
    updateNodesPosition(response.actor_positions);
}



void CarlanetManager::updateNodesPosition(std::list<carla_api_base::actor_position> actorList){
    std::set<std::string> knownactor_ids;
    for (const auto& entry : modulesToTrack) knownactor_ids.insert(entry.first);

    const double now = SIMTIME_DBL(simTime());

    for (const auto& actor : actorList){
        auto it = modulesToTrack.find(actor.actor_id);
        if (it == modulesToTrack.end()) {
            createAndInitializeActor(actor);
            g_lastSeenSimTime[actor.actor_id] = now;
            continue;
        }

        if (!it->second || !it->second->getParentModule()) {
            EV_WARN << "[CarlanetManager][updateNodesPosition] mobility for " << actor.actor_id << " is null/detached; skipping\n";
            continue;
        }

        Coord position(actor.position[0], actor.position[1], actor.position[2]);
        Coord velocity(actor.velocity[0], actor.velocity[1], actor.velocity[2]);

        // Python gives [pitch, yaw, roll] in degrees → INET EulerAngles need unit-typed radians
        Quaternion rotation = Quaternion(EulerAngles(
            inet::units::values::rad(actor.rotation[1] * M_PI / 180.0),   // yaw
            inet::units::values::rad(actor.rotation[0] * M_PI / 180.0),   // pitch
            inet::units::values::rad(actor.rotation[2] * M_PI / 180.0)    // roll
        ));

        const bool firstTime = (g_lastSeenSimTime.find(actor.actor_id) == g_lastSeenSimTime.end());
        if (firstTime) it->second->preInitialize(position, velocity, rotation);
        else           it->second->nextPosition(position, velocity, rotation);

        g_lastSeenSimTime[actor.actor_id] = now;
        knownactor_ids.erase(actor.actor_id);
    }

    // Reaping disabled while stabilizing
    (void)knownactor_ids;
}

void CarlanetManager::connect(){
    try {
        socket.setsockopt(ZMQ_RCVTIMEO, timeout_ms);
        socket.setsockopt(ZMQ_SNDTIMEO, timeout_ms);
        std::string addr = protocol + "://" + host + ":" + std::to_string(port);
        EV_INFO << "Connecting to pyCARLANeT at " << addr << "\n";
        socket.connect(addr);
        zmqReady = true;
    } catch (const std::exception& e) {
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
            double a = 0.0, v = 0.0;
            bool has_control = false;

            if (ud.is_object() && ud.value("msg_type", "") == "CONTROL") {
                actor = ud.value("actor_id", "");
                if (ud.contains("ctrl") && ud["ctrl"].is_object()) {
                    a = ud["ctrl"].value("desired_acceleration", 0.0);
                    v = ud["ctrl"].value("desired_speed", 0.0);
                    has_control = ud["ctrl"].value("has_control", false);
                }
            }

            std::ostringstream oss;
            oss.setf(std::ios::fixed);
            oss << std::setprecision(6);

            oss << "[CarlanetManager][handleMessage]"
                << " hop=BRIDGE_TO_MANAGER"
                << " simulation_time=" << SIMTIME_DBL(simTime())
                << " actor=" << (actor.empty() ? "?" : actor)
                << " desired_speed=" << v
                << " desired_acceleration=" << a
                << " has_control=" << (has_control ? 1 : 0)
                << "\n";

            EV_INFO << oss.str();
        } catch (...) {
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
    req.actor.actor_id = par("lateSpawnActorId").stringValue();
    req.actor.actor_type = par("lateSpawnActorType").stringValue();
    req.actor.actor_configuration = json::parse(par("lateSpawnActorConfig").stringValue());

    EV_INFO << "[CarlanetManager][handleLateSpawn] handleLateSpawn called at t=" << simTime() << "\n";

    sendToCarla(json(req));
    carla_api::spawn_completed resp = receiveFromCarla<carla_api::spawn_completed>(100.0);

    if (resp.status != 0 || resp.actor_positions.empty()) {
        EV_ERROR << "[CarlanetManager][handleLateSpawn] late spawn failed for " << req.actor.actor_id
                 << " error=" << resp.error << "\n";
        return;
    }

    // IMPORTANT: Let updateNodesPosition create the OMNeT++ module if missing.
    // Do NOT also create a separate module with a different naming scheme.
    updateNodesPosition(resp.actor_positions);

    EV_INFO << "[CarlanetManager][handleLateSpawn] late spawn completed for actor=" << req.actor.actor_id
            << " at t=" << simTime() << "\n";
}

void CarlanetManager::createAndInitializeActor(const carla_api_base::actor_position& newActor)
{
    const std::string& moduleTypeName =
        newActor.is_net_active ? networkActiveModuleType : networkPassiveModuleType;

    cModule* root = getSimulation()->getSystemModule();
    cModuleType* actorType = cModuleType::get(moduleTypeName.c_str());
    if (!actorType)
        throw cRuntimeError("[CarlanetManager][createAndInitializeActor] module type not found: %s",
                            moduleTypeName.c_str());

    int idx = getOrAssignGateIndex(newActor.actor_id);
    const int parsedVehIndex = parseVehIndexOrDefault(newActor.actor_id, idx);

    root->setSubmoduleVectorSize("actors", idx + 1);

    cModule* node = actorType->create("actors", root, idx);

    if (!node->hasPar("actor_id"))
        throw cRuntimeError("[CarlanetManager][createAndInitializeActor] actor module type must define parameter actor_id (missing on %s)",
                            node->getFullPath().c_str());

    node->par("actor_id").setStringValue(newActor.actor_id.c_str());

    const std::string lateId = par("lateSpawnActorId").stringValue();
    const bool isLateJoiner = (newActor.actor_id == lateId);

    if (node->hasPar("platoon_role"))
        node->par("platoon_role").setStringValue(isLateJoiner ? "joiner" : "follower");

    if (node->hasPar("node_id"))
        node->par("node_id").setIntValue(parsedVehIndex);

    // Keep the late joiner on the same platoon/leader as the current demo.
    if (node->hasPar("platoon_id"))
        node->par("platoon_id").setIntValue(0);

    if (node->hasPar("leader_id"))
        node->par("leader_id").setIntValue(0);

    if (node->hasPar("join_position")) {
        if (isLateJoiner)
            node->par("join_position").setIntValue(parsedVehIndex); // veh4 -> joins at back position 4
        else
            node->par("join_position").setIntValue(-1);
    }

    node->finalizeParameters();
    node->buildInside();

    connectNodeToControlIn(node, idx);

    Coord position(newActor.position[0], newActor.position[1], newActor.position[2]);
    Coord velocity(newActor.velocity[0], newActor.velocity[1], newActor.velocity[2]);
    Quaternion rotation = Quaternion(EulerAngles(
        inet::units::values::rad(newActor.rotation[1] * M_PI / 180.0),
        inet::units::values::rad(newActor.rotation[0] * M_PI / 180.0),
        inet::units::values::rad(newActor.rotation[2] * M_PI / 180.0)
    ));

    auto* mobility = check_and_cast<CarlaInetMobility*>(node->getSubmodule("mobility"));
    modulesToTrack[newActor.actor_id] = mobility;
    mobility->preInitialize(position, velocity, rotation);

    auto* notification = new inet::cPreModuleInitNotification();
    notification->module = node;
    root->emit(POST_MODEL_CHANGE, notification, nullptr);

    node->scheduleStart(simTime());
    node->callInitialize();

    {
        std::ostringstream oss;
        oss << "[CarlanetManager][createAndInitializeActor]"
            << " hop=MANAGER_TO_OMNET"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << newActor.actor_id
            << " idx=" << idx
            << " type=" << moduleTypeName
            << " role=" << (isLateJoiner ? "joiner" : "follower")
            << " node_id=" << parsedVehIndex
            << " platoon_id=0"
            << " leader_id=0"
            << " join_position=" << (isLateJoiner ? parsedVehIndex : -1)
            << " start_maneuver_at=" << (isLateJoiner ? 0.0 : -1.0)
            << " node=" << node->getFullPath()
            << "\n";
        EV_INFO << oss.str();
    }
}

// Not being used, but can despawn/re-spawn or do cleanup of dynamic actors mid-run
void CarlanetManager::destroyActor(std::string actor_id){
    auto it = modulesToTrack.find(actor_id);
    if (it == modulesToTrack.end()) return;

    cModule* node = it->second->getParentModule();

    auto gi = gateIndexByActor.find(actor_id);
    if (gi != gateIndexByActor.end()) {
        disconnectNodeFromControlIn(node, gi->second);
        gateIndexByActor.erase(gi);
    }

    node->callFinish();
    node->deleteModule();
    modulesToTrack.erase(it);
}

// ---- Gate index helpers ------------------------------------------------------
int CarlanetManager::getOrAssignGateIndex(const std::string& actor_id) {
    auto it = gateIndexByActor.find(actor_id);
    if (it != gateIndexByActor.end()) return it->second;
    int idx = nextGateIndex++;
    ensureControlInSize(nextGateIndex);
    gateIndexByActor.emplace(actor_id, idx);
    return idx;
}
void CarlanetManager::ensureControlInSize(int minSize) {
    int cur = gateSize("controlIn");
    if (cur < minSize) setGateSize("controlIn", minSize);
}
void CarlanetManager::connectNodeToControlIn(cModule* node, int idx) {
    int g = node->findGate("toManager");
    if (g >= 0) {
        cGate* src = node->gate(g);
        cGate* dst = gate("controlIn", idx);
        if (!src->isConnected()) src->connectTo(dst);
    }
}
void CarlanetManager::disconnectNodeFromControlIn(cModule* node, int idx) {
    int g = node->findGate("toManager");
    if (g >= 0) {
        cGate* src = node->gate(g);
        if (src->isConnected()) src->disconnect();
    }
}
// -----------------------------------------------------------------------------

json CarlanetManager::receiveFromCarla(double timeoutFactor){
    int recv_timeout_ms = std::max(4000, int(timeout_ms * timeoutFactor));
    socket.setsockopt(ZMQ_RCVTIMEO, recv_timeout_ms);

    zmq::message_t reply{};
    if (!socket.recv(reply, zmq::recv_flags::none)) throw std::runtime_error("CARLA Timeout");
    json jsonResp = json::parse(reply.to_string());

    switch (jsonResp["simulation_status"].get<int>()){
        case SIM_STATUS_FINISHED_OK:
        case SIM_STATUS_FINISHED_ACCIDENT:
        case SIM_STATUS_FINISHED_TIME_LIMIT:
            endSimulation(); break;
        case SIM_STATUS_ERROR:
            throw std::runtime_error("Communication error. Wrong message sequence!");
        default: break;
    }
    return jsonResp;
}
