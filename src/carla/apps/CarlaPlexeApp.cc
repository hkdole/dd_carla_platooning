#include "carla/apps/CarlaPlexeApp.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>


using namespace omnetpp;
using namespace inet;

namespace carla {

Define_Module(CarlaPlexeApp);

// Lazy-registered (initialize stage 0)
simsignal_t CarlaPlexeApp::desiredAccelerationSignal    = SIMSIGNAL_NULL;
simsignal_t CarlaPlexeApp::desiredSpeedSignal           = SIMSIGNAL_NULL;
simsignal_t CarlaPlexeApp::joinStateSignal              = SIMSIGNAL_NULL;

CarlaPlexeApp::~CarlaPlexeApp() {
    if (controlTimer) { cancelAndDelete(controlTimer); controlTimer = nullptr; }
    if (startTimer)   { cancelAndDelete(startTimer);   startTimer   = nullptr; }
    if (retryTimer)   { cancelAndDelete(retryTimer);   retryTimer   = nullptr; }
}

bool CarlaPlexeApp::hasParam(const char* n) const {
    return findPar(n) >= 0;
}

int CarlaPlexeApp::parIntOr(const char* n, int def) {
    return hasParam(n) ? par(n).intValue() : def;
}

double CarlaPlexeApp::parDoubleOr(const char* n, double def) {
    return hasParam(n) ? par(n).doubleValue() : def;
}

double CarlaPlexeApp::parSpeedMpsOr(const char* n, double defMps) {
    if (!hasParam(n)) return defMps;
    // If ini/NED uses units, this will work. Otherwise, it returns in base unit anyway.
    try { return par(n).doubleValueInUnit("mps"); }
    catch (...) { return par(n).doubleValue(); }
}

bool CarlaPlexeApp::parBoolOr(const char* n, bool def) {
    return hasParam(n) ? par(n).boolValue() : def;
}

std::string CarlaPlexeApp::parStringOr(const char* n, const std::string& def) {
    return hasParam(n) ? par(n).stdstringValue() : def;
}

simtime_t CarlaPlexeApp::parTimeOr(const char* n, simtime_t def) {
    return hasParam(n) ? (simtime_t)par(n) : def;
}

int CarlaPlexeApp::deriveMyId() const {
    cModule* host = getParentModule();
    if (!host) return getId();
    int idx = host->getIndex();
    if (idx >= 0) return idx;
    return host->getId();
}

void CarlaPlexeApp::initialize(int stage) {
    veins::DemoBaseApplLayer::initialize(stage);

    // IMPORTANT: do full init at stage 0 
    if (stage != 0)
        return;

    // Safe signal registration (once per class)
    if (desiredAccelerationSignal == SIMSIGNAL_NULL)
        desiredAccelerationSignal = registerSignal("desired_acceleration");
    if (desiredSpeedSignal == SIMSIGNAL_NULL)
        desiredSpeedSignal = registerSignal("desired_speed");
    if (joinStateSignal == SIMSIGNAL_NULL)
        joinStateSignal = registerSignal("join_state");

    // Parameters (with safe fallbacks)
    actor_id = parStringOr("actor_id", getParentModule() ? getParentModule()->getFullName() : "actor");
    role    = parStringOr("platoon_role", "follower");

    node_id = parIntOr("node_id", -1);
    if (node_id < 0) node_id = deriveMyId();

    platoon_id = parIntOr("platoon_id", 0);
    leader_id  = parIntOr("leader_id", 0);

    initialFormationStr = parStringOr("initial_formation", "");

    beaconInterval  = parTimeOr("beaconInterval", 0.1);
    control_interval = parTimeOr("control_interval", 0.05);

    headway = parDoubleOr("headway", headway);
    gap_min  = parDoubleOr("gap_min",  gap_min);
    k_gap    = parDoubleOr("k_gap",    k_gap);
    k_dv     = parDoubleOr("k_dv",     k_dv);

    platoon_speed = parSpeedMpsOr("platoon_speed", platoon_speed);
    k_speed_p      = parDoubleOr("k_speed_p", k_speed_p);
    a_min         = parDoubleOr("a_min", a_min);
    a_max         = parDoubleOr("a_max", a_max);

    // --- leader brake profile (optional) ---
    leader_brake_at         = parTimeOr("leader_brake_at", -1);
    leader_brake_hold_for   = parTimeOr("leader_brake_hold_for", 0);
    leader_brake_speed      = parSpeedMpsOr("leader_brake_speed", 0.0);
    leader_post_brake_speed = parSpeedMpsOr("leader_post_brake_speed", -1.0);

    if (isLeader()) {
        EV_INFO << "[JAB][leader_brake_profile]"
                << " actor=" << actor_id
                << " leader_brake_at=" << leader_brake_at
                << " leader_brake_speed=" << leader_brake_speed
                << " leader_brake_hold_for=" << leader_brake_hold_for
                << " leader_post_brake_speed=" << leader_post_brake_speed
                << "\n";
    }

    start_maneuver_at = parTimeOr("start_maneuver_at", 10.0);
    join_req_retry    = parTimeOr("join_req_retry", 0.5);
    // For “strict mirroring”, default to 1 requejoinStateunless you override.
    join_req_max_tries = parIntOr("join_req_max_tries", join_req_max_tries);

    approach_delta_v  = parSpeedMpsOr("approach_delta_v", approach_delta_v); // default 30km/h
    approach_spacing = parDoubleOr("approach_spacing", approach_spacing); // default 15m
    in_pos_slack      = parDoubleOr("in_pos_slack", in_pos_slack);           // default +11m
    vehicle_length   = parDoubleOr("vehicle_length", vehicle_length);

    max_age = parTimeOr("max_age", 0.5);
    debug  = parBoolOr("debug", false);

    joinAllowed = parBoolOr("joinAllowed", true);

    mobility = dynamic_cast<CarlaInetMobility*>(getParentModule()->getSubmodule("mobility"));
    if (!mobility)
        throw cRuntimeError("[JAB] mobility submodule not found / wrong type on %s", getFullPath().c_str());

    // Leader initializes formation
    platoonFormation.clear();
    if (isLeader()) {
        if (!initialFormationStr.empty()) platoonFormation = decForm(initialFormationStr);
        if (platoonFormation.empty()) platoonFormation.push_back(node_id);
    }

    // Timers (ours)
    controlTimer = new cMessage("jabControlTimer");
    startTimer   = new cMessage("jabStartTimer");
    retryTimer   = new cMessage("jabRetryTimer");

    // Control loop MUjoinStaterun or BridgeApp will stay at 0/0
    scheduleAt(simTime() + control_interval, controlTimer);

    // PLEXE scenario triggers maneuver at a scheduled time
    if (isJoiner()) {
        simtime_t tStart = start_maneuver_at;
        if (tStart <= simTime())
            tStart = simTime() + control_interval;
        scheduleAt(tStart, startTimer);
    }

    // Seed outputs
    desired_acceleration    = 0.0;
    desired_speed = platoon_speed;
    lastForward = inet::Coord(1, 0, 0);

    setState(State::IDLE);

    if (debug) {
        EV_INFO << "[JAB] init actor_id=" << actor_id
                << " role=" << role
                << " node_id=" << node_id
                << " platoon_id=" << platoon_id
                << " leader_id=" << leader_id
                << " platoonFormation=" << encForm(platoonFormation)
                << " beaconInterval=" << beaconInterval
                << " control_interval=" << control_interval
                << " approach_spacing=" << approach_spacing
                << " approach_delta_v=" << approach_delta_v
                << "\n";
    }
}

void CarlaPlexeApp::handleSelfMsg(cMessage* msg) {
    // Intercept DemoBaseApplLayer beacon selfmsg and use it to send OUR beacons.
    if (msg->isSelfMessage()) {
        const char* n = msg->getName();
        if (n && std::string(n).find("beacon") != std::string::npos) {
            sendPlatoonBeacon();
            scheduleAt(simTime() + beaconInterval, msg);
            return;
        }
    }

    if (msg == controlTimer) {
        onControlTick();
        scheduleAt(simTime() + control_interval, controlTimer);
        return;
    }
    if (msg == startTimer) {
        startJoinManeuver();
        return;
    }
    if (msg == retryTimer) {
        // Optional reliability layer. Default join_req_max_tries=1 for PLEXE-like behavior.
        if (joinState== State::J_WAIT_REPLY) {
            if (joinReqTries++ < join_req_max_tries) {
                sendManeuverMsg("JOIN_REQ", leader_id);
                scheduleAt(simTime() + join_req_retry, retryTimer);
            }
            else {
                if (debug) EV_WARN << "[JAB] joiner " << node_id << " abort: no JOIN_RSP\n";
                setState(State::IDLE);
            }
        }
        return;
    }

    veins::DemoBaseApplLayer::handleSelfMsg(msg);
}

// -------------------- DemoSafetyMessage param helpers ------------------------

void CarlaPlexeApp::setParI(veins::DemoSafetyMessage* m, const char* n, int v) {
    int i = m->findPar(n);
    if (i < 0) { m->addPar(n); i = m->findPar(n); }
    m->par(i).setLongValue((long)v);
}
void CarlaPlexeApp::setParD(veins::DemoSafetyMessage* m, const char* n, double v) {
    int i = m->findPar(n);
    if (i < 0) { m->addPar(n); i = m->findPar(n); }
    m->par(i).setDoubleValue(v);
}
void CarlaPlexeApp::setParB(veins::DemoSafetyMessage* m, const char* n, bool v) {
    int i = m->findPar(n);
    if (i < 0) { m->addPar(n); i = m->findPar(n); }
    m->par(i).setBoolValue(v);
}
void CarlaPlexeApp::setParS(veins::DemoSafetyMessage* m, const char* n, const std::string& v) {
    int i = m->findPar(n);
    if (i < 0) { m->addPar(n); i = m->findPar(n); }
    m->par(i).setStringValue(v.c_str());
}

bool CarlaPlexeApp::getParI(veins::DemoSafetyMessage* m, const char* n, int& v) const {
    int i = m->findPar(n);
    if (i < 0) return false;
    v = (int)m->par(i).longValue();
    return true;
}
bool CarlaPlexeApp::getParD(veins::DemoSafetyMessage* m, const char* n, double& v) const {
    int i = m->findPar(n);
    if (i < 0) return false;
    v = m->par(i).doubleValue();
    return true;
}
bool CarlaPlexeApp::getParB(veins::DemoSafetyMessage* m, const char* n, bool& v) const {
    int i = m->findPar(n);
    if (i < 0) return false;
    v = m->par(i).boolValue();
    return true;
}
bool CarlaPlexeApp::getParS(veins::DemoSafetyMessage* m, const char* n, std::string& v) const {
    int i = m->findPar(n);
    if (i < 0) return false;
    const char* s = m->par(i).stringValue();
    v = s ? std::string(s) : std::string();
    return true;
}

std::string CarlaPlexeApp::encForm(const std::vector<int>& f) {
    std::ostringstream oss;
    for (size_t i = 0; i < f.size(); ++i) {
        if (i) oss << ",";
        oss << f[i];
    }
    return oss.str();
}

std::vector<int> CarlaPlexeApp::decForm(const std::string& s) {
    std::vector<int> out;
    std::istringstream iss(s);
    std::string tok;
    while (std::getline(iss, tok, ',')) {
        if (!tok.empty()) out.push_back(std::stoi(tok));
    }
    return out;
}


const char* CarlaPlexeApp::stateStr(State s) const {
    switch (s) {
        case State::IDLE:                     return "IDLE";
        case State::L_WAIT_JOINER_IN_POSITION: return "L_WAIT_JOINER_IN_POSITION";
        case State::L_WAIT_JOINER_TO_JOIN:    return "L_WAIT_JOINER_TO_JOIN";
        case State::J_WAIT_REPLY:             return "J_WAIT_REPLY";
        case State::J_WAIT_INFORMATION:       return "J_WAIT_INFORMATION";
        case State::J_MOVE_IN_POSITION:       return "J_MOVE_IN_POSITION";
        case State::J_WAIT_JOIN:              return "J_WAIT_JOIN";
        case State::J_FOLLOWER:                 return "J_FOLLOWER";
        default:                              return "UNKNOWN";
    }
}

// ---------------------------- state ------------------------------------------

void CarlaPlexeApp::setState(State s) {
    if (joinState == s) return;

    State prev = joinState;
    joinState = s;

    emit(joinStateSignal, static_cast<long>(static_cast<int>(joinState)));

    // platform-style log
    {
        std::ostringstream oss;
        oss << "[CarlaPlexeApp][setState]"
            << " hop=JAB_STATE_LOCAL"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " node_id=" << node_id
            << " role=" << role
            << " join_state_prev=" << stateStr(prev)
            << " join_state_next=" << stateStr(joinState)
            << "\n";
        EV_INFO << oss.str();
    }
}


// ---------------------------- V2V send --------------------------------------

void CarlaPlexeApp::sendPlatoonBeacon() {
    const Coord pos = mobility->getCurrentPosition();
    const Coord vel = mobility->getCurrentVelocity();

    auto* bsm = new veins::DemoSafetyMessage("PlatoonBeacon");
    populateWSM(bsm);

    bsm->setSenderPos(veins::Coord{pos.x, pos.y, pos.z});
    bsm->setSenderSpeed(veins::Coord{vel.x, vel.y, vel.z});

    setParS(bsm, "msgType", "PlatooningBeacon");
    setParI(bsm, "senderId", node_id);
    setParS(bsm, "role", role);
    setParI(bsm, "platoon_id", platoon_id);
    setParI(bsm, "leader_id", leader_id);

    setParD(bsm, "desired_acceleration", desired_acceleration);
    setParD(bsm, "desired_speed",        desired_speed);
    setParD(bsm, "platoon_speed", platoon_speed);

    if (isLeader() && !platoonFormation.empty())
        setParS(bsm, "platoonFormation", encForm(platoonFormation));

    sendDown(bsm);
}

void CarlaPlexeApp::sendManeuverMsg(const std::string& msgType, int destination_id, bool permitted) {
    const Coord pos = mobility->getCurrentPosition();
    const Coord vel = mobility->getCurrentVelocity();

    auto* m = new veins::DemoSafetyMessage(msgType.c_str());
    populateWSM(m);

    m->setSenderPos(veins::Coord{pos.x, pos.y, pos.z});
    m->setSenderSpeed(veins::Coord{vel.x, vel.y, vel.z});

    setParS(m, "msgType", msgType);
    setParI(m, "senderId", node_id);
    setParI(m, "destination_id", destination_id);

    setParS(m, "role", role);
    setParI(m, "platoon_id", platoon_id);
    setParI(m, "leader_id", leader_id);

    const uint32_t sequence = nextSequence(msgType);
    setParI(m, "sequence", (int)sequence);
    setParI(m, "externalId", (int)externalId);

    if (msgType == "JOIN_RSP") {
        setParB(m, "permitted", permitted);
    }

    const bool includeFormation =
        (msgType == "MOVE_TO_POS" || msgType == "MOVE_TO_POS_ACK" ||
         msgType == "JOIN_FORMATION" || msgType == "JOIN_FORMATION_ACK" ||
         msgType == "UPDATE_FORMATION");

    if (includeFormation) {
        setParD(m, "platoon_speed", platoon_speed);
        setParI(m, "joinIndex", joinIndex);
        setParS(m, "pendingFormation", encForm(pendingFormation));
    }

    // platform-style log
    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss << std::setprecision(6);

        oss << "[CarlaPlexeApp][sendManeuverMsg]"
            << " hop=JAB_TO_V2V"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " msgType=" << msgType
            << " src=" << node_id
            << " dst=" << destination_id
            << " platoon_id=" << platoon_id
            << " leader_id=" << leader_id
            << " sequence=" << sequence
            << " externalId=" << externalId;

        if (msgType == "JOIN_RSP")
            oss << " permitted=" << (permitted ? 1 : 0);

        if (includeFormation) {
            oss << " joinIndex=" << joinIndex
                << " platoon_speed=" << platoon_speed
                << " pendingFormation=" << encForm(pendingFormation);
        }

        oss << "\n";
        EV_INFO << oss.str();
    }

    sendDown(m);
}



// ---------------------------- RX path ---------------------------------------

void CarlaPlexeApp::onBSM(veins::DemoSafetyMessage* bsm) {
    int senderVehicleId = -1;
    int destinationId    = -1;

    std::string msgType = "PlatooningBeacon";
    (void)getParS(bsm, "msgType", msgType);

    if (!getParI(bsm, "senderId", senderVehicleId))
        return;

    (void)getParI(bsm, "destination_id", destinationId);

    if (senderVehicleId == node_id)
        return;

    // --- refresh neighbor state (always) ---
    Neighbor& neighborState = neighborsByVehicleId[senderVehicleId];
    const auto& sp = bsm->getSenderPos();
    const auto& sv = bsm->getSenderSpeed();
    neighborState.pos  = Coord(sp.x, sp.y, sp.z);
    neighborState.vel  = Coord(sv.x, sv.y, sv.z);
    neighborState.last= simTime();

    (void)getParS(bsm, "role", neighborState.role);
    (void)getParD(bsm, "desired_acceleration", neighborState.desired_acceleration);

    // --- parse routing/correlation fields ---
    int msgPlatoonId = -1;
    int msgLeaderId  = -1;
    int rxSequence        = -1;
    int rxExternalId      = -1;

    (void)getParI(bsm, "platoon_id", msgPlatoonId);
    (void)getParI(bsm, "leader_id",  msgLeaderId);
    (void)getParI(bsm, "sequence",        rxSequence);
    (void)getParI(bsm, "externalId",      rxExternalId);

    auto logDrop = [&](const char* reason) {
        std::ostringstream oss;
        oss << "[CarlaPlexeApp][onBSM]"
            << " hop=V2V_TO_JAB"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " msgType=" << msgType
            << " src=" << senderVehicleId
            << " dst=" << destinationId
            << " platoon_id=" << msgPlatoonId
            << " leader_id=" << msgLeaderId
            << " sequence=" << rxSequence
            << " externalId=" << rxExternalId
            << " reason=" << (reason ? reason : "")
            << "\n";
        EV_INFO << oss.str();
    };

    // --- Directed addressing enforcement for maneuver messages ---
    if (msgType != "PlatooningBeacon") {
        if (destinationId < 0) { logDrop("missing_destination_id"); return; }
        if (destinationId != node_id) { logDrop("not_dst"); return; }

        // accepted maneuver addressed to us
        {
            std::ostringstream oss;
            oss << "[CarlaPlexeApp][onBSM]"
                << " hop=V2V_TO_JAB"
                << " simulation_time=" << SIMTIME_DBL(simTime())
                << " actor=" << actor_id
                << " msgType=" << msgType
                << " src=" << senderVehicleId
                << " dst=" << destinationId
                << " platoon_id=" << msgPlatoonId
                << " leader_id=" << msgLeaderId
                << " sequence=" << rxSequence
                << " externalId=" << rxExternalId
                << "\n";
            EV_INFO << oss.str();
        }
    }

    // --- PlatooningBeacon handling ---
    if (msgType == "PlatooningBeacon") {
        if (neighborState.role == "leader") {
            std::string formationStr;
            if (getParS(bsm, "platoonFormation", formationStr))
                platoonFormation = decForm(formationStr);
        }

        if (isJoiner() && joinState== State::J_MOVE_IN_POSITION) {
            if (joinIndex > 0 && joinIndex <= (int)pendingFormation.size() - 1) {
                const int frontId = pendingFormation[joinIndex - 1];
                if (senderVehicleId == frontId) {
                    joinerCheckInPositionOnFrontPlatooningBeacon(frontId);
                }
            }
        }
        return;
    }

    // --- externalId correlation ---
    if (msgType == "JOIN_REQ" && isLeader() && rxExternalId >= 0) {
        externalId = (uint32_t)rxExternalId;
    }
    if (isJoiner() && joinerInManeuver() && rxExternalId >= 0 && externalId != 0 && (uint32_t)rxExternalId != externalId) {
        logDrop("externalId_mismatch");
        return;
    }

    // --- Join-time learning / relaxed filtering (leader → joiner) ---
    const bool leaderToJoinerManeuver =
        joinerInManeuver() && senderVehicleId == leader_id &&
        (msgType == "JOIN_RSP" || msgType == "MOVE_TO_POS" || msgType == "JOIN_FORMATION" || msgType == "UPDATE_FORMATION");

    if (leaderToJoinerManeuver) {
        if (msgPlatoonId >= 0) platoon_id = msgPlatoonId;
        if (msgLeaderId  >= 0) leader_id  = msgLeaderId;
    }
    else {
        if (msgLeaderId >= 0 && msgLeaderId != leader_id) { logDrop("leader_mismatch"); return; }

        if (msgPlatoonId >= 0 && msgPlatoonId != platoon_id) {
            const bool allowUnknownToLeader =
                isLeader() &&
                (msgType == "JOIN_REQ" || msgType == "MOVE_TO_POS_ACK" || msgType == "JOIN_FORMATION_ACK") &&
                (msgPlatoonId <= 0);

            if (!allowUnknownToLeader) { logDrop("platoon_mismatch"); return; }
        }
    }

    // --- Dispatch maneuver handlers ---
    if (msgType == "JOIN_REQ" && isLeader()) { handleJoinReq(senderVehicleId, msgPlatoonId, msgLeaderId); return; }

    if (msgType == "JOIN_RSP" && isJoiner()) {
        bool permitted = false;
        (void)getParB(bsm, "permitted", permitted);
        handleJoinRsp(permitted, senderVehicleId);
        return;
    }

    if (msgType == "MOVE_TO_POS" && isJoiner()) {
        double ps = platoon_speed;
        int idx = -1;
        std::string formationStr;
        (void)getParD(bsm, "platoon_speed", ps);
        (void)getParI(bsm, "joinIndex", idx);
        (void)getParS(bsm, "pendingFormation", formationStr);
        handleMoveToPos(ps, idx, decForm(formationStr));
        return;
    }

    if (msgType == "MOVE_TO_POS_ACK" && isLeader()) { handleMoveToPosAck(senderVehicleId); return; }

    if (msgType == "JOIN_FORMATION" && isJoiner()) {
        double ps = platoon_speed;
        int idx = -1;
        std::string formationStr;
        (void)getParD(bsm, "platoon_speed", ps);
        (void)getParI(bsm, "joinIndex", idx);
        (void)getParS(bsm, "pendingFormation", formationStr);
        handleJoinFormation(ps, idx, decForm(formationStr));
        return;
    }

    if (msgType == "JOIN_FORMATION_ACK" && isLeader()) { handleJoinFormationAck(senderVehicleId); return; }

    if (msgType == "UPDATE_FORMATION") {
        std::string formationStr;
        if (getParS(bsm, "pendingFormation", formationStr))
            handleUpdateFormation(decForm(formationStr));
        return;
    }

    logDrop("unknown_kind");
}


// ------------------------- maneuver handlers ---------------------------------

void CarlaPlexeApp::startJoinManeuver() {
    if (!isJoiner())
        return;

    txSeqByMessageType.clear();
    externalId = (uint32_t)intrand(1000000000);

    joinReqTries = 0;
    joinerId     = node_id;

    setState(State::J_WAIT_REPLY);

    // platform-style log
    {
        std::ostringstream oss;
        oss << "[CarlaPlexeApp][startJoinManeuver]"
            << " hop=JAB_TO_V2V"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " joiner_id=" << node_id
            << " leader_id=" << leader_id
            << " externalId=" << externalId
            << "\n";
        EV_INFO << oss.str();
    }

    sendManeuverMsg("JOIN_REQ", leader_id);
    scheduleAt(simTime() + join_req_retry, retryTimer);
}


void CarlaPlexeApp::handleJoinReq(int fromId, int reqPlatoonId, int reqLeaderId) {
    if (!isLeader())
        return;

    if (!joinAllowed) {
        // denied
        {
            std::ostringstream oss;
            oss << "[CarlaPlexeApp][handleJoinReq]"
                << " hop=V2V_TO_JAB"
                << " simulation_time=" << SIMTIME_DBL(simTime())
                << " actor=" << actor_id
                << " joiner_id=" << fromId
                << " leader_id=" << leader_id
                << " permitted=0"
                << " reason=join_disabled"
                << "\n";
            EV_INFO << oss.str();
        }
        sendManeuverMsg("JOIN_RSP", fromId, /*permitted=*/false);
        return;
    }

    if ((joinState== State::L_WAIT_JOINER_IN_POSITION || joinState== State::L_WAIT_JOINER_TO_JOIN) &&
        joinerId >= 0 && joinerId != fromId)
    {
        {
            std::ostringstream oss;
            oss << "[CarlaPlexeApp][handleJoinReq]"
                << " hop=V2V_TO_JAB"
                << " simulation_time=" << SIMTIME_DBL(simTime())
                << " actor=" << actor_id
                << " joiner_id=" << fromId
                << " leader_id=" << leader_id
                << " permitted=0"
                << " reason=leader_busy"
                << " active_joiner_id=" << joinerId
                << "\n";
            EV_INFO << oss.str();
        }

        sendManeuverMsg("JOIN_RSP", fromId, /*permitted=*/false);
        return;
    }

    if (reqLeaderId != leader_id) {
        sendManeuverMsg("JOIN_RSP", fromId, /*permitted=*/false);
        return;
    }
    if (reqPlatoonId > 0 && reqPlatoonId != platoon_id) {
        sendManeuverMsg("JOIN_RSP", fromId, /*permitted=*/false);
        return;
    }

    joinerId = fromId;

    if (platoonFormation.empty())
        platoonFormation.push_back(node_id);

    pendingFormation = platoonFormation;
    if (std::find(pendingFormation.begin(), pendingFormation.end(), joinerId) == pendingFormation.end())
        pendingFormation.push_back(joinerId);

    joinIndex = (int)pendingFormation.size() - 1;

    // platform-style accept log
    {
        std::ostringstream oss;
        oss << "[CarlaPlexeApp][handleJoinReq]"
            << " hop=V2V_TO_JAB"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " joiner_id=" << joinerId
            << " platoon_id=" << platoon_id
            << " leader_id=" << leader_id
            << " permitted=1"
            << " joinIndex=" << joinIndex
            << " externalId=" << externalId
            << " pendingFormation=" << encForm(pendingFormation)
            << "\n";
        EV_INFO << oss.str();
    }

    sendManeuverMsg("JOIN_RSP",    joinerId, /*permitted=*/true);
    sendManeuverMsg("MOVE_TO_POS", joinerId);

    setState(State::L_WAIT_JOINER_IN_POSITION);
}



void CarlaPlexeApp::handleJoinRsp(bool permitted, int fromLeader) {
    if (!isJoiner()) return;
    if (joinState!= State::J_WAIT_REPLY) return;
    if (fromLeader != leader_id) return;

    if (retryTimer && retryTimer->isScheduled())
        cancelEvent(retryTimer);

    if (!permitted) {
        setState(State::IDLE);
        if (debug)
            EV_WARN << "[JAB] joiner " << node_id << " denied by leader " << fromLeader << "\n";
        return;
    }

    setState(State::J_WAIT_INFORMATION);
    if (debug)
        EV_INFO << "[JAB] joiner " << node_id << " permitted; waiting MOVE_TO_POS\n";
}

void CarlaPlexeApp::handleMoveToPos(double pSpeed, int idx, const std::vector<int>& f) {
    if (!isJoiner()) return;
    if (joinState!= State::J_WAIT_INFORMATION) return;

    platoon_speed = pSpeed;
    joinIndex    = idx;
    pendingFormation = f;

    setState(State::J_MOVE_IN_POSITION);

    if (debug)
        EV_INFO << "[JAB] joiner MOVE_TO_POS platoon_speed=" << platoon_speed
                << " joinIndex=" << joinIndex
                << " pendingFormation=" << encForm(pendingFormation)
                << " (approach_spacing=" << approach_spacing
                << ", approach_delta_v=" << approach_delta_v << ")\n";
}

void CarlaPlexeApp::handleMoveToPosAck(int fromJoiner) {
    if (!isLeader()) return;
    if (joinState!= State::L_WAIT_JOINER_IN_POSITION) return;
    if (fromJoiner != joinerId) return;

    sendManeuverMsg("JOIN_FORMATION", joinerId);
    setState(State::L_WAIT_JOINER_TO_JOIN);

    if (debug)
        EV_INFO << "[JAB] leader got MOVE_TO_POS_ACK; sent JOIN_FORMATION\n";
}

void CarlaPlexeApp::handleJoinFormation(double pSpeed, int idx, const std::vector<int>& f) {
    if (!isJoiner()) return;
    if (joinState!= State::J_WAIT_JOIN) return;

    platoon_speed = pSpeed;
    joinIndex    = idx;
    platoonFormation    = f;

    role = "follower";
    setState(State::J_FOLLOWER);

    // platform-style join complete log
    {
        std::ostringstream oss;
        oss << "[CarlaPlexeApp][handleJoinFormation]"
            << " hop=V2V_TO_JAB"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " platoon_id=" << platoon_id
            << " leader_id=" << leader_id
            << " joinIndex=" << joinIndex
            << " externalId=" << externalId
            << " platoonFormation=" << encForm(platoonFormation)
            << "\n";
        EV_INFO << oss.str();
    }

    sendManeuverMsg("JOIN_FORMATION_ACK", leader_id);
}



void CarlaPlexeApp::handleJoinFormationAck(int fromJoiner) {
    if (!isLeader()) return;
    if (joinState!= State::L_WAIT_JOINER_TO_JOIN) return;
    if (fromJoiner != joinerId) return;

    platoonFormation = pendingFormation;

    for (int id : platoonFormation) {
        if (id == node_id) continue;
        sendManeuverMsg("UPDATE_FORMATION", id);
    }

    setState(State::IDLE);

    if (debug)
        EV_INFO << "[JAB] leader received JOIN_FORMATION_ACK; platoonFormation=" << encForm(platoonFormation) << "\n";
}

void CarlaPlexeApp::handleUpdateFormation(const std::vector<int>& f) {
    platoonFormation = f;
    if (debug)
        EV_INFO << "[JAB] node " << node_id << " updated platoonFormation=" << encForm(platoonFormation) << "\n";
}

// ------------------------------ control --------------------------------------

bool CarlaPlexeApp::getNeighbor(int id, Neighbor& out, simtime_t& age) const{
    auto it = neighborsByVehicleId.find(id);
    if (it == neighborsByVehicleId.end()) return false;
    age = simTime() - it->second.last;
    if (age > max_age) return false;
    out = it->second;
    return true;
}

inet::Coord CarlaPlexeApp::forwardUnit(const inet::Coord& v) {
    const double spd = v.length();
    if (spd > 1e-3) {
        lastForward = v / spd;
        return lastForward;
    }
    return lastForward;
}

double CarlaPlexeApp::gapToIdLong(int predId,
                                      const inet::Coord& myPos,
                                      const inet::Coord& myVel,
                                      double& vPredLong,
                                      simtime_t& ageOut)
{
    Neighbor pred;
    if (!getNeighbor(predId, pred, ageOut)) return -1.0;

    const inet::Coord fwd = forwardUnit(myVel);
    const inet::Coord rel = pred.pos - myPos;

    double gapLong = rel.x * fwd.x + rel.y * fwd.y + rel.z * fwd.z;
    gapLong -= vehicle_length;

    vPredLong = pred.vel.x * fwd.x + pred.vel.y * fwd.y + pred.vel.z * fwd.z;
    return gapLong;
}

bool CarlaPlexeApp::findMyIndexInFormation(int& idxOut) const{
    idxOut = -1;
    for (int i = 0; i < (int)platoonFormation.size(); ++i) {
        if (platoonFormation[i] == node_id) { idxOut = i; return true; }
    }
    return false;
}

void CarlaPlexeApp::joinerCheckInPositionOnFrontPlatooningBeacon(int frontId) {
    // Mirror PLEXE onPlatoonBeacon() logic:
    // distance < targetDistance(platoon_speed) + 11  => send MoveToPositionAck and wait join.
    if (!isJoiner()) return;
    if (joinState!= State::J_MOVE_IN_POSITION) return;

    Neighbor front;
    simtime_t age = 0;
    if (!getNeighbor(frontId, front, age)) return;

    const Coord myPos = mobility->getCurrentPosition();
    const double distance = (front.pos - myPos).length() - vehicle_length;

    const double thresh = targetDistance(platoon_speed) + in_pos_slack;

    if (distance < thresh) {
        sendManeuverMsg("MOVE_TO_POS_ACK", leader_id);
        setState(State::J_WAIT_JOIN);

        if (debug) {
            EV_INFO << "[JAB] joiner in position (PlatooningBeacon-driven): dist=" << distance
                    << " thresh=" << thresh
                    << " targetDistance=" << targetDistance(platoon_speed)
                    << " slack=" << in_pos_slack
                    << " -> sent MOVE_TO_POS_ACK, now J_WAIT_JOIN\n";
        }
    }
}

void CarlaPlexeApp::onControlTick() {
    const Coord pos = mobility->getCurrentPosition();
    const Coord vel = mobility->getCurrentVelocity();

    const Coord fwd = forwardUnit(vel);
    const double vLong = vel.x * fwd.x + vel.y * fwd.y + vel.z * fwd.z;
    const double speedLong = std::max(0.0, vLong);

    double a = 0.0;
    double vTarget = platoon_speed;

    if (isLeader()) {
        // nominal leader target
        vTarget = platoon_speed;

        // optional brake window (speed step)
        if (leader_brake_at >= SIMTIME_ZERO && simTime() >= leader_brake_at) {
            bool inBrake = true;

            // hold_for == 0s => brake forever
            if (leader_brake_hold_for > SIMTIME_ZERO) {
                inBrake = simTime() < (leader_brake_at + leader_brake_hold_for);
            }

            if (inBrake) {
                vTarget = std::max(0.0, leader_brake_speed);
            } else {
                // after brake window: either explicit pojoinStatespeed, or back to platoon_speed
                if (leader_post_brake_speed >= 0.0) {
                    vTarget = leader_post_brake_speed;
                } else {
                    vTarget = platoon_speed;
                }
            }
        }

        a = k_speed_p * (vTarget - speedLong);
    }
    else if (isJoiner()) {
        if (joinState== State::J_MOVE_IN_POSITION || joinState== State::J_WAIT_JOIN) {
            vTarget = platoon_speed + approach_delta_v;

            double aSpeed = k_speed_p * (vTarget - speedLong);
            a = aSpeed;

            if (joinIndex > 0 && joinIndex <= (int)pendingFormation.size() - 1) {
                const int frontId = pendingFormation[joinIndex - 1];

                double vPredLong = 0.0;
                simtime_t agePred = 0;
                const double gapLong = gapToIdLong(frontId, pos, vel, vPredLong, agePred);

                if (gapLong > 0) {
                    const double sRef = approach_spacing;
                    const double eGap = gapLong - sRef;
                    const double dv   = vPredLong - speedLong;

                    a = k_gap * eGap + k_dv * dv;
                }
            }
        }
        else {
            vTarget = platoon_speed;
            a = k_speed_p * (vTarget - speedLong);
        }
    }
    else {
        int myIdx = -1;
        if (findMyIndexInFormation(myIdx) && myIdx > 0) {
            const int predId = platoonFormation[myIdx - 1];

            double vPredLong = 0.0;
            simtime_t age = 0;
            const double gapLong = gapToIdLong(predId, pos, vel, vPredLong, age);

            if (gapLong > 0) {
                const double sRef = targetDistance(speedLong);
                const double eGap = gapLong - sRef;
                const double dv   = vPredLong - speedLong;
                a = k_gap * eGap + k_dv * dv;
            } else {
                a = 0.4 * k_speed_p * (platoon_speed - speedLong);
            }
        } else {
            a = 0.4 * k_speed_p * (platoon_speed - speedLong);
        }
    }

    a = std::max(a_min, std::min(a, a_max));

    desired_acceleration = a;
    desired_speed        = vTarget;


    {
        std::ostringstream oss;
        oss.setf(std::ios::fixed);
        oss << std::setprecision(6);

        oss << "[CarlaPlexeApp][onControlTick]"
            << " hop=JAB_TO_BRIDGE"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " desired_speed=" << desired_speed
            << " desired_acceleration=" << desired_acceleration
            << " has_control=1"
            << "\n";

        EV_INFO << oss.str();
    }

    emit(desiredAccelerationSignal, a);
    emit(desiredSpeedSignal, vTarget);
}



bool CarlaPlexeApp::joinerInManeuver() const{
    return isJoiner() && (joinState== State::J_WAIT_REPLY ||
                          joinState== State::J_WAIT_INFORMATION ||
                          joinState== State::J_MOVE_IN_POSITION ||
                          joinState== State::J_WAIT_JOIN);
}

uint32_t CarlaPlexeApp::nextSequence(const std::string& msgType) {
    return ++txSeqByMessageType[msgType];
}



} // namespace carla
