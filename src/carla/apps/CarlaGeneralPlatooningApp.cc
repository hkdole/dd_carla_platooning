#include "carla/apps/CarlaGeneralPlatooningApp.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "veins/modules/utility/Consts80211p.h"

using namespace omnetpp;

namespace {

static inline veins::Coord toVeinsCoord(const inet::Coord& c)
{
    return veins::Coord(c.x, c.y, c.z);
}

static inline bool looksLikeBeaconSelfMsg(const omnetpp::cMessage* msg)
{
    if (!msg || !msg->isSelfMessage()) return false;
    const char* n = msg->getName();
    return n && std::string(n).find("beacon") != std::string::npos;
}

template <typename T>
static inline T clampValue(T v, T lo, T hi)
{
    return std::max(lo, std::min(hi, v));
}

template <typename MsgT>
static std::vector<int> extractFormation(const MsgT* msg)
{
    std::vector<int> formation;
    if (!msg) return formation;

    formation.reserve(msg->getNewPlatoonFormationArraySize());
    for (unsigned int i = 0; i < msg->getNewPlatoonFormationArraySize(); ++i) {
        formation.push_back(msg->getNewPlatoonFormation(i));
    }
    return formation;
}

template <typename MsgT>
static std::vector<int> extractPlatoonFormation(const MsgT* msg)
{
    std::vector<int> formation;
    if (!msg) return formation;

    formation.reserve(msg->getPlatoonFormationArraySize());
    for (unsigned int i = 0; i < msg->getPlatoonFormationArraySize(); ++i) {
        formation.push_back(msg->getPlatoonFormation(i));
    }
    return formation;
}

} // namespace

namespace carla {

Define_Module(CarlaGeneralPlatooningApp);

simsignal_t CarlaGeneralPlatooningApp::desiredAccelerationSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::desiredSpeedSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::controlModeSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::activeControllerSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::hasControlSignal_ = SIMSIGNAL_NULL;

simsignal_t CarlaGeneralPlatooningApp::speedSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::accelerationSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::controllerAccelerationExportSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::distanceSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::relativeSpeedSignal_ = SIMSIGNAL_NULL;

CarlaGeneralPlatooningApp::CarlaGeneralPlatooningApp() = default;

CarlaGeneralPlatooningApp::~CarlaGeneralPlatooningApp()
{
    if (controlTimer_) cancelAndDelete(controlTimer_);
    if (startTimer_) cancelAndDelete(startTimer_);
    delete joinManeuver_;
}

void CarlaGeneralPlatooningApp::setInManeuver(bool b, CarlaManeuver* maneuver)
{
    inManeuver_ = b;
    activeManeuver_ = b ? maneuver : nullptr;
}

double CarlaGeneralPlatooningApp::getStandstillDistance(ActiveController controller) const
{
    (void)controller;
    return gapMin_;
}

double CarlaGeneralPlatooningApp::getHeadway(ActiveController controller) const
{
    (void)controller;
    return headway_;
}

double CarlaGeneralPlatooningApp::getTargetDistance(double speed) const
{
    return gapMin_ + headway_ * std::max(0.0, speed);
}

double CarlaGeneralPlatooningApp::getTargetDistance(ActiveController controller, double speed) const
{
    (void)controller;
    return getTargetDistance(speed);
}

ActiveController CarlaGeneralPlatooningApp::getTargetController() const
{
    return ActiveController::CACC;
}

veins::Coord CarlaGeneralPlatooningApp::getCurrentPosition() const
{
    if (!mobility_) return veins::Coord(0, 0, 0);
    const auto p = mobility_->getCurrentPosition();
    return veins::Coord(p.x, p.y, p.z);
}

int CarlaGeneralPlatooningApp::getCurrentLaneIndex() const
{
    if (controllerAdapter_.hasFixedLane()) return controllerAdapter_.getFixedLane();
    if (positionHelper_.getPlatoonLane() >= 0) return positionHelper_.getPlatoonLane();
    return 0;
}

double CarlaGeneralPlatooningApp::scalarSpeedFromVelocity(const veins::Coord& v) const
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

std::vector<int> CarlaGeneralPlatooningApp::parseFormation(const std::string& csv) const
{
    std::vector<int> out;
    std::stringstream ss(csv);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) out.push_back(std::stoi(tok));
    }
    return out;
}

double CarlaGeneralPlatooningApp::computeGapToNeighbor(const PlatoonNeighborState& n) const
{
    const auto myPos = getCurrentPosition();
    const double frontLen = (n.length > 0.0) ? n.length : vehicleLength_;
    return myPos.distance(n.pos) - frontLen;
}

double CarlaGeneralPlatooningApp::computeSpeedBiasFromGapError(double gapError, double gain, double limit) const
{
    return clampValue(gain * gapError, -limit, limit);
}

int CarlaGeneralPlatooningApp::findFrontVehicleIdInFormation(const std::vector<int>& formation) const
{
    const auto it = std::find(formation.begin(), formation.end(), nodeId_);
    if (it == formation.end()) return -1;
    if (it == formation.begin()) return -1;
    return *(it - 1);
}

void CarlaGeneralPlatooningApp::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    if (stage != 0) return;

    if (desiredAccelerationSignal_ == SIMSIGNAL_NULL)
        desiredAccelerationSignal_ = registerSignal("desired_acceleration");
    if (desiredSpeedSignal_ == SIMSIGNAL_NULL)
        desiredSpeedSignal_ = registerSignal("desired_speed");
    if (controlModeSignal_ == SIMSIGNAL_NULL)
        controlModeSignal_ = registerSignal("control_mode");
    if (hasControlSignal_ == SIMSIGNAL_NULL)
        hasControlSignal_ = registerSignal("has_control");
    if (activeControllerSignal_ == SIMSIGNAL_NULL)
        activeControllerSignal_ = registerSignal("active_controller");

    // PLEXE-style exported result vectors
    if (speedSignal_ == SIMSIGNAL_NULL)
        speedSignal_ = registerSignal("speed");
    if (accelerationSignal_ == SIMSIGNAL_NULL)
        accelerationSignal_ = registerSignal("acceleration");
    if (controllerAccelerationExportSignal_ == SIMSIGNAL_NULL)
        controllerAccelerationExportSignal_ = registerSignal("controllerAcceleration");
    if (distanceSignal_ == SIMSIGNAL_NULL)
        distanceSignal_ = registerSignal("distance");
    if (relativeSpeedSignal_ == SIMSIGNAL_NULL)
        relativeSpeedSignal_ = registerSignal("relativeSpeed");

    mobility_ = check_and_cast<CarlaInetMobility*>(getParentModule()->getSubmodule("mobility"));

    actorId_ = par("actor_id").stdstringValue();
    nodeId_ = par("node_id").intValue();
    platoonId_ = par("platoon_id").intValue();
    leaderId_ = par("leader_id").intValue();

    nominalPlatoonSpeed_ = par("nominal_platoon_speed").doubleValue();
    leaderTargetSpeed_ = par("leader_target_speed").doubleValue();
    vehicleLength_ = par("vehicle_length").doubleValue();

    headway_ = par("headway").doubleValue();
    gapMin_ = par("gap_min").doubleValue();
    kGap_ = par("k_gap").doubleValue();
    kDv_ = par("k_dv").doubleValue();
    kSpeedP_ = par("k_speed_p").doubleValue();
    aMin_ = par("a_min").doubleValue();
    aMax_ = par("a_max").doubleValue();

    if (hasPar("k_acc_ff")) kAccFF_ = par("k_acc_ff").doubleValue();
    if (hasPar("k_leader_dv")) kLeaderDv_ = par("k_leader_dv").doubleValue();
    if (hasPar("k_gap_speed")) kGapSpeed_ = par("k_gap_speed").doubleValue();
    if (hasPar("max_closure_speed")) maxClosureSpeed_ = par("max_closure_speed").doubleValue();

    // carried into the parsed comparison schema, matching PLEXE labels
    if (hasPar("caccXi")) caccXi_ = par("caccXi").doubleValue();
    if (hasPar("caccOmegaN")) caccOmegaN_ = par("caccOmegaN").doubleValue();

    beaconInterval_ = par("beaconInterval");
    controlInterval_ = par("control_interval");
    maxAge_ = par("max_age");

    if (hasPar("start_maneuver_at")) startManeuverAt_ = par("start_maneuver_at");
    if (hasPar("join_position")) joinPosition_ = par("join_position").intValue();
    if (hasPar("approach_delta_v")) approachDeltaV_ = par("approach_delta_v").doubleValue();
    if (hasPar("in_pos_slack")) inPosSlack_ = par("in_pos_slack").doubleValue();

    positionHelper_.setId(nodeId_);
    positionHelper_.setExternalId(actorId_);
    positionHelper_.setPlatoonId(platoonId_);
    positionHelper_.setLeaderId(leaderId_);
    positionHelper_.setPlatoonSpeed(nominalPlatoonSpeed_);
    positionHelper_.setController(ActiveController::CACC);
    positionHelper_.setDistance(gapMin_);
    positionHelper_.setHeadway(headway_);

    if (hasPar("initial_formation")) {
        const auto formation = parseFormation(par("initial_formation").stdstringValue());
        if (!formation.empty()) positionHelper_.setPlatoonFormation(formation);
    }

    const std::string roleStr = par("platoon_role").stdstringValue();
    if (roleStr == "leader") {
        role_ = PlatoonRole::LEADER;
        positionHelper_.setController(ActiveController::CC);
        controllerAdapter_.setActiveController(ActiveController::CC);
        controllerAdapter_.setControlMode(ControlMode::LEADER_CRUISE);
    }
    else if (roleStr == "follower") {
        role_ = PlatoonRole::FOLLOWER;
        positionHelper_.setController(ActiveController::CACC);
        controllerAdapter_.setActiveController(ActiveController::CACC);
        controllerAdapter_.setCACCConstantSpacing(getTargetDistance(nominalPlatoonSpeed_));
        controllerAdapter_.setControlMode(ControlMode::FOLLOWER_PLATOON);
    }
    else if (roleStr == "joiner") {
        role_ = PlatoonRole::JOINER;
        positionHelper_.setController(ActiveController::CC);
        controllerAdapter_.setActiveController(ActiveController::CC);
        controllerAdapter_.setControlMode(ControlMode::JOINER_FREE_CRUISE);
    }
    else {
        role_ = PlatoonRole::NONE;
        positionHelper_.setController(ActiveController::UNKNOWN);
        controllerAdapter_.setActiveController(ActiveController::UNKNOWN);
        controllerAdapter_.setControlMode(ControlMode::HOLD);
    }

    joinManeuver_ = new CarlaJoinAtBack(this);

    controlTimer_ = new cMessage("platooningControlTimer");
    startTimer_ = new cMessage("joinStartTimer");

    // initialize export state from current mobility state
    {
        veins::Coord vel(0, 0, 0);
        if (mobility_) vel = toVeinsCoord(mobility_->getCurrentVelocity());
        lastExportSpeed_ = scalarSpeedFromVelocity(vel);
        lastExportTime_ = simTime();
        actualAcceleration_ = 0.0;
    }

    scheduleAt(simTime() + controlInterval_, controlTimer_);

    if (role_ == PlatoonRole::JOINER && startManeuverAt_ >= SIMTIME_ZERO) {
        simtime_t tStart = startManeuverAt_;
        if (tStart <= simTime())
            tStart = simTime() + controlInterval_;
        scheduleAt(tStart, startTimer_);
    }

    EV_INFO << "[CarlaGeneralPlatooningApp][initialize]"
            << " actor=" << actorId_
            << " nodeId=" << nodeId_
            << " platoonId=" << platoonId_
            << " leaderId=" << leaderId_
            << " role=" << static_cast<int>(role_)
            << " nominalPlatoonSpeed=" << nominalPlatoonSpeed_
            << " caccXi=" << caccXi_
            << " caccOmegaN=" << caccOmegaN_
            << "\n";

    EV_INFO << "[CarlaGeneralPlatooningApp][startupParams]"
            << " actor=" << actorId_
            << " nodeId=" << nodeId_
            << " role=" << static_cast<int>(role_)
            << " headway=" << headway_
            << " gap_min=" << gapMin_
            << " approach_delta_v=" << approachDeltaV_
            << " in_pos_slack=" << inPosSlack_
            << "\n";
}

void CarlaGeneralPlatooningApp::handleSelfMsg(cMessage* msg)
{
    if (activeManeuver_ && activeManeuver_->handleSelfMsg(msg)) return;

    if (looksLikeBeaconSelfMsg(msg)) {
        sendPlatooningBeacon();
        scheduleAt(simTime() + beaconInterval_, msg);
        return;
    }

    if (msg == controlTimer_) {
        onControlTick();
        scheduleAt(simTime() + controlInterval_, controlTimer_);
        return;
    }

    if (msg == startTimer_) {
        startJoinManeuverIfConfigured();
        return;
    }

    DemoBaseApplLayer::handleSelfMsg(msg);
}

void CarlaGeneralPlatooningApp::startJoinManeuverIfConfigured()
{
    if (!joinManeuver_) return;
    if (role_ != PlatoonRole::JOINER) return;

    JoinManeuverParameters params;
    params.platoonId = platoonId_;
    params.leaderId = leaderId_;
    params.position = joinPosition_;

    EV_INFO << "[CarlaGeneralPlatooningApp][startJoinManeuverIfConfigured] actor=" << actorId_
            << " platoonId=" << params.platoonId
            << " leaderId=" << params.leaderId
            << " position=" << params.position
            << " at t=" << simTime()
            << "\n";

    joinManeuver_->startManeuver(&params);
}

void CarlaGeneralPlatooningApp::sendUnicast(cPacket* msg, int destination)
{
    take(msg);

    auto* mm = dynamic_cast<ManeuverMessage*>(msg);
    if (mm) {
        mm->setDestinationId(destination);
    }

    auto* frame = new veins::BaseFrame1609_4(msg->getName(), msg->getKind());
    frame->setRecipientAddress(-1);
    frame->setChannelNumber(static_cast<int>(veins::Channel::cch));
    frame->encapsulate(msg);
    sendDown(frame);

    EV_INFO << "[CarlaGeneralPlatooningApp][sendUnicast]"
            << " actor=" << actorId_
            << " logical_dst=" << destination
            << " packet=" << msg->getName()
            << " mode=broadcast_with_payload_filter"
            << "\n";
}

void CarlaGeneralPlatooningApp::fillManeuverMessage(
    ManeuverMessage* msg,
    int vehicleId,
    const std::string& externalId,
    int platoonId,
    int destinationId)
{
    msg->setKind(0);
    msg->setVehicleId(vehicleId);
    msg->setExternalId(externalId.c_str());
    msg->setPlatoonId(platoonId);
    msg->setDestinationId(destinationId);
}

UpdatePlatoonFormation* CarlaGeneralPlatooningApp::createUpdatePlatoonFormation(
    int vehicleId,
    const std::string& externalId,
    int platoonId,
    int destinationId,
    double platoonSpeed,
    int platoonLane,
    const std::vector<int>& platoonFormation)
{
    auto* msg = new UpdatePlatoonFormation("UpdatePlatoonFormation");
    fillManeuverMessage(msg, vehicleId, externalId, platoonId, destinationId);
    msg->setPlatoonSpeed(platoonSpeed);
    msg->setPlatoonLane(platoonLane);
    msg->setPlatoonFormationArraySize(platoonFormation.size());
    for (unsigned int i = 0; i < platoonFormation.size(); ++i) {
        msg->setPlatoonFormation(i, platoonFormation[i]);
    }
    return msg;
}

UpdatePlatoonData* CarlaGeneralPlatooningApp::createUpdatePlatoonData(
    int vehicleId,
    const std::string& externalId,
    int platoonId,
    int destinationId,
    double platoonSpeed,
    int platoonLane,
    const std::vector<int>& platoonFormation,
    int newPlatoonId)
{
    auto* msg = new UpdatePlatoonData("UpdatePlatoonData");
    fillManeuverMessage(msg, vehicleId, externalId, platoonId, destinationId);
    msg->setPlatoonSpeed(platoonSpeed);
    msg->setPlatoonLane(platoonLane);
    msg->setPlatoonFormationArraySize(platoonFormation.size());
    for (unsigned int i = 0; i < platoonFormation.size(); ++i) {
        msg->setPlatoonFormation(i, platoonFormation[i]);
    }
    msg->setNewPlatoonId(newPlatoonId);
    return msg;
}

void CarlaGeneralPlatooningApp::refreshNeighborFromBeacon(const PlatooningBeacon* pb)
{
    if (!pb) return;

    const int srcId = pb->getVehicleId();
    if (srcId < 0) return;
    if (srcId == nodeId_) return;

    PlatoonNeighborState& n = neighborsByVehicleId_[srcId];
    n.pos = veins::Coord(pb->getPositionX(), pb->getPositionY(), 0);
    n.vel = veins::Coord(pb->getSpeedX(), pb->getSpeedY(), 0);
    n.actualAcceleration = pb->getAcceleration();
    n.controllerAcceleration = pb->getControllerAcceleration();
    n.scalarSpeed = pb->getSpeed();
    n.length = pb->getLength();
    n.angle = pb->getAngle();
    n.last = simTime();
    n.valid = true;

    EV_INFO << "[CarlaGeneralPlatooningApp][refreshNeighborFromBeacon]"
            << " actor=" << actorId_
            << " srcId=" << srcId
            << " speed=" << n.scalarSpeed
            << " accel=" << n.actualAcceleration
            << " ctrlAccel=" << n.controllerAcceleration
            << " pos=(" << n.pos.x << "," << n.pos.y << ")"
            << " last=" << n.last
            << "\n";
}

bool CarlaGeneralPlatooningApp::getNeighbor(int id, PlatoonNeighborState& out, simtime_t& age) const
{
    auto it = neighborsByVehicleId_.find(id);
    if (it == neighborsByVehicleId_.end()) {
        age = SIMTIME_ZERO;
        return false;
    }

    age = simTime() - it->second.last;
    out = it->second;
    return it->second.valid;
}

void CarlaGeneralPlatooningApp::handleLowerMsg(cMessage* msg)
{
    if (auto* dsm = dynamic_cast<veins::DemoSafetyMessage*>(msg)) {
        EV_INFO << "[CarlaGeneralPlatooningApp][handleLowerMsg]"
                << " actor=" << actorId_
                << " ignored_base_DemoSafetyMessage"
                << " name=" << (dsm->getName() ? dsm->getName() : "")
                << "\n";
        delete dsm;
        return;
    }

    if (auto* pb = dynamic_cast<PlatooningBeacon*>(msg)) {
        refreshNeighborFromBeacon(pb);
        onPlatoonBeacon(pb);
        delete pb;
        return;
    }

    if (auto* upd = dynamic_cast<UpdatePlatoonData*>(msg)) {
        if (upd->getDestinationId() == nodeId_) {
            handleUpdatePlatoonData(upd);
        }
        delete upd;
        return;
    }

    if (auto* upf = dynamic_cast<UpdatePlatoonFormation*>(msg)) {
        if (upf->getDestinationId() == nodeId_) {
            handleUpdatePlatoonFormation(upf);
        }
        delete upf;
        return;
    }

    if (auto* mm = dynamic_cast<ManeuverMessage*>(msg)) {
        if (mm->getDestinationId() == nodeId_) {
            EV_INFO << "[CarlaGeneralPlatooningApp][handleLowerMsg]"
                    << " actor=" << actorId_
                    << " received_maneuver=" << mm->getName()
                    << " src=" << mm->getVehicleId()
                    << " dst=" << mm->getDestinationId()
                    << " platoonId=" << mm->getPlatoonId()
                    << "\n";
            onManeuverMessage(mm);
        }
        delete mm;
        return;
    }

    if (auto* frame = dynamic_cast<veins::BaseFrame1609_4*>(msg)) {
        cPacket* pkt = frame->decapsulate();
        delete frame;

        if (!pkt) {
            EV_WARN << "[CarlaGeneralPlatooningApp][handleLowerMsg]"
                    << " actor=" << actorId_
                    << " dropped_empty_frame"
                    << "\n";
            return;
        }

        if (auto* dsm = dynamic_cast<veins::DemoSafetyMessage*>(pkt)) {
            EV_INFO << "[CarlaGeneralPlatooningApp][handleLowerMsg]"
                    << " actor=" << actorId_
                    << " ignored_inner_DemoSafetyMessage"
                    << " name=" << (dsm->getName() ? dsm->getName() : "")
                    << "\n";
            delete dsm;
            return;
        }

        if (auto* pb = dynamic_cast<PlatooningBeacon*>(pkt)) {
            refreshNeighborFromBeacon(pb);
            onPlatoonBeacon(pb);
            delete pb;
            return;
        }

        if (auto* upd = dynamic_cast<UpdatePlatoonData*>(pkt)) {
            if (upd->getDestinationId() == nodeId_) {
                handleUpdatePlatoonData(upd);
            }
            delete upd;
            return;
        }

        if (auto* upf = dynamic_cast<UpdatePlatoonFormation*>(pkt)) {
            if (upf->getDestinationId() == nodeId_) {
                handleUpdatePlatoonFormation(upf);
            }
            delete upf;
            return;
        }

        if (auto* mm = dynamic_cast<ManeuverMessage*>(pkt)) {
            if (mm->getDestinationId() == nodeId_) {
                EV_INFO << "[CarlaGeneralPlatooningApp][handleLowerMsg]"
                        << " actor=" << actorId_
                        << " received_maneuver=" << mm->getName()
                        << " src=" << mm->getVehicleId()
                        << " dst=" << mm->getDestinationId()
                        << " platoonId=" << mm->getPlatoonId()
                        << "\n";
                onManeuverMessage(mm);
            }
            delete mm;
            return;
        }

        EV_WARN << "[CarlaGeneralPlatooningApp][handleLowerMsg]"
                << " actor=" << actorId_
                << " dropped_unknown_packet class=" << pkt->getClassName()
                << " name=" << pkt->getName()
                << "\n";
        delete pkt;
        return;
    }

    EV_WARN << "[CarlaGeneralPlatooningApp][handleLowerMsg]"
            << " actor=" << actorId_
            << " dropped_nonframe_packet class=" << msg->getClassName()
            << " name=" << msg->getName()
            << "\n";
    delete msg;
}

void CarlaGeneralPlatooningApp::onPlatoonBeacon(const PlatooningBeacon* pb)
{
    if (joinManeuver_) joinManeuver_->onPlatoonBeacon(pb);
}

void CarlaGeneralPlatooningApp::onManeuverMessage(const ManeuverMessage* mm)
{
    if (!mm) return;

    if (auto* mtp = dynamic_cast<const MoveToPosition*>(mm)) {
        if (role_ == PlatoonRole::JOINER && mtp->getDestinationId() == nodeId_) {
            const auto formation = extractFormation(mtp);
            const int frontId = findFrontVehicleIdInFormation(formation);

            maneuverSetFormation(formation);
            maneuverSetPlatoonSpeed(mtp->getPlatoonSpeed());

            if (frontId >= 0) maneuverSetJoinFrontVehicleId(frontId);
            else maneuverClearJoinFrontVehicleId();

            EV_INFO << "[CarlaGeneralPlatooningApp][onManeuverMessage]"
                    << " actor=" << actorId_
                    << " maneuver=MoveToPosition"
                    << " frontId=" << frontId
                    << " platoonSpeed=" << mtp->getPlatoonSpeed()
                    << " formationSize=" << formation.size()
                    << "\n";
        }
    }
    else if (auto* jf = dynamic_cast<const JoinFormation*>(mm)) {
        if (role_ == PlatoonRole::JOINER && jf->getDestinationId() == nodeId_) {
            const auto formation = extractFormation(jf);
            const int frontId = findFrontVehicleIdInFormation(formation);

            maneuverSetFormation(formation);
            maneuverSetPlatoonSpeed(jf->getPlatoonSpeed());

            if (frontId >= 0) maneuverSetJoinFrontVehicleId(frontId);
            else maneuverClearJoinFrontVehicleId();

            EV_INFO << "[CarlaGeneralPlatooningApp][onManeuverMessage]"
                    << " actor=" << actorId_
                    << " maneuver=JoinFormation"
                    << " frontId=" << frontId
                    << " platoonSpeed=" << jf->getPlatoonSpeed()
                    << " formationSize=" << formation.size()
                    << "\n";
        }
    }

    if (activeManeuver_) activeManeuver_->onManeuverMessage(mm);
    else if (joinManeuver_) joinManeuver_->onManeuverMessage(mm);

    if (role_ == PlatoonRole::FOLLOWER ||
        controllerAdapter_.getControlMode() == ControlMode::FOLLOWER_PLATOON) {
        joinFrontVehicleId_ = -1;
        platoonId_ = positionHelper_.getPlatoonId();
        leaderId_ = positionHelper_.getLeaderId();
    }
}

void CarlaGeneralPlatooningApp::handleUpdatePlatoonFormation(const UpdatePlatoonFormation* msg)
{
    if (role_ != PlatoonRole::FOLLOWER) return;
    if (msg->getPlatoonId() != positionHelper_.getPlatoonId()) return;
    if (msg->getVehicleId() != positionHelper_.getLeaderId()) return;

    const int oldPred = positionHelper_.getPredecessorId();
    const auto oldFormation = positionHelper_.getPlatoonFormation();

    std::vector<int> formation;
    formation.reserve(msg->getPlatoonFormationArraySize());
    for (unsigned int i = 0; i < msg->getPlatoonFormationArraySize(); ++i) {
        formation.push_back(msg->getPlatoonFormation(i));
    }
    positionHelper_.setPlatoonFormation(formation);

    const int newPred = positionHelper_.getPredecessorId();

    EV_INFO << "[CarlaGeneralPlatooningApp][handleUpdatePlatoonFormation]"
            << " actor=" << actorId_
            << " oldFormationSize=" << oldFormation.size()
            << " newFormationSize=" << formation.size()
            << " oldPred=" << oldPred
            << " newPred=" << newPred
            << "\n";
}

void CarlaGeneralPlatooningApp::handleUpdatePlatoonData(const UpdatePlatoonData* msg)
{
    if (role_ != PlatoonRole::FOLLOWER) return;
    if (msg->getPlatoonId() != positionHelper_.getPlatoonId()) return;
    if (msg->getVehicleId() != positionHelper_.getLeaderId()) return;

    const int oldPlatoonId = positionHelper_.getPlatoonId();

    handleUpdatePlatoonFormation(msg);
    positionHelper_.setPlatoonId(msg->getNewPlatoonId());
    platoonId_ = msg->getNewPlatoonId();

    EV_INFO << "[CarlaGeneralPlatooningApp][handleUpdatePlatoonData]"
            << " actor=" << actorId_
            << " oldPlatoonId=" << oldPlatoonId
            << " newPlatoonId=" << msg->getNewPlatoonId()
            << "\n";
}

void CarlaGeneralPlatooningApp::sendPlatooningBeacon()
{
    const auto pos = getCurrentPosition();

    veins::Coord vel(0, 0, 0);
    if (mobility_) vel = toVeinsCoord(mobility_->getCurrentVelocity());

    const double speed = scalarSpeedFromVelocity(vel);

    auto* pb = new PlatooningBeacon("PlatooningBeacon");
    pb->setVehicleId(positionHelper_.getId());

    // Match PLEXE semantics:
    // acceleration            = measured / actual longitudinal acceleration
    // controllerAcceleration  = controller command
    pb->setAcceleration(actualAcceleration_);
    pb->setControllerAcceleration(desiredAcceleration_);

    pb->setSpeed(speed);
    pb->setPositionX(pos.x);
    pb->setPositionY(pos.y);
    pb->setTime(simTime().dbl());
    pb->setSequenceNumber(++beaconSequence_);
    pb->setLength(vehicleLength_);
    pb->setSpeedX(vel.x);
    pb->setSpeedY(vel.y);
    pb->setAngle(0.0);

    auto* frame = new veins::BaseFrame1609_4("PlatooningBeaconFrame", 0);
    frame->setRecipientAddress(-1);
    frame->setChannelNumber(static_cast<int>(veins::Channel::cch));
    frame->encapsulate(pb);

    EV_INFO << "[CarlaGeneralPlatooningApp][sendPlatooningBeacon]"
            << " actor=" << actorId_
            << " nodeId=" << positionHelper_.getId()
            << " seq=" << beaconSequence_
            << " speed=" << speed
            << " actualAcceleration=" << actualAcceleration_
            << " controllerAcceleration=" << desiredAcceleration_
            << " pos=(" << pos.x << "," << pos.y << ")"
            << "\n";

    sendDown(frame);
}

ControllerInputs CarlaGeneralPlatooningApp::buildControllerInputs() const
{
    ControllerInputs in;
    in.controlMode = controllerAdapter_.getControlMode();
    in.activeController = controllerAdapter_.getActiveController();

    if (in.activeController == ActiveController::UNKNOWN) {
        switch (in.controlMode) {
            case ControlMode::LEADER_CRUISE:
            case ControlMode::JOINER_FREE_CRUISE:
                in.activeController = ActiveController::CC;
                break;
            case ControlMode::FOLLOWER_PLATOON:
                in.activeController = ActiveController::CACC;
                break;
            case ControlMode::JOINER_MOVE_IN_POSITION:
            case ControlMode::JOINER_WAIT_JOIN:
                in.activeController = ActiveController::FAKED_CACC;
                break;
            default:
                break;
        }
    }

    veins::Coord vel(0, 0, 0);
    if (mobility_) vel = toVeinsCoord(mobility_->getCurrentVelocity());
    const double mySpeed = scalarSpeedFromVelocity(vel);

    in.egoSpeed = mySpeed;
    in.targetSpeed = leaderTargetSpeed_;
    in.standstillDistance = gapMin_;
    in.headway = headway_;
    in.targetGap = getTargetDistance(mySpeed);
    in.approachDeltaV = approachDeltaV_;
    in.maxClosureSpeed = maxClosureSpeed_;
    in.kGap = kGap_;
    in.kDv = kDv_;
    in.kSpeedP = kSpeedP_;
    in.kAccFF = kAccFF_;
    in.kLeaderDv = kLeaderDv_;
    in.kGapSpeed = kGapSpeed_;
    in.aMin = aMin_;
    in.aMax = aMax_;

    const int myId = positionHelper_.getId();
    const int predId = positionHelper_.getPredecessorId();
    const int leaderId = positionHelper_.getLeaderId();

    PlatoonNeighborState pred;
    simtime_t predAge = SIMTIME_ZERO;
    const bool predFresh =
        (predId >= 0) &&
        (predId != myId) &&
        getNeighbor(predId, pred, predAge) &&
        (predAge <= maxAge_);
    if (predFresh) {
        in.predecessor.valid = true;
        in.predecessor.speed = pred.scalarSpeed;
        in.predecessor.actualAcceleration = pred.actualAcceleration;
        in.predecessor.controllerAcceleration = pred.controllerAcceleration;
        in.predecessor.distance = computeGapToNeighbor(pred);
    }

    PlatoonNeighborState leader;
    simtime_t leaderAge = SIMTIME_ZERO;
    const bool leaderFresh =
        (leaderId >= 0) &&
        (leaderId != myId) &&
        getNeighbor(leaderId, leader, leaderAge) &&
        (leaderAge <= maxAge_);
    if (leaderFresh) {
        in.leader.valid = true;
        in.leader.speed = leader.scalarSpeed;
        in.leader.actualAcceleration = leader.actualAcceleration;
        in.leader.controllerAcceleration = leader.controllerAcceleration;
        in.leader.distance = computeGapToNeighbor(leader);
    }

    const auto& fakeLeader = controllerAdapter_.getLeaderVehicleFakeData();
    if (fakeLeader.valid) {
        in.fakeLeader.valid = true;
        in.fakeLeader.speed = fakeLeader.speed;
        in.fakeLeader.actualAcceleration = fakeLeader.actualAcceleration;
        in.fakeLeader.controllerAcceleration = fakeLeader.controllerAcceleration;
    }

    const auto& fakeFront = controllerAdapter_.getFrontVehicleFakeData();
    if (fakeFront.valid) {
        in.fakeFront.valid = true;
        in.fakeFront.speed = fakeFront.speed;
        in.fakeFront.actualAcceleration = fakeFront.actualAcceleration;
        in.fakeFront.controllerAcceleration = fakeFront.controllerAcceleration;
        in.fakeFront.distance = fakeFront.distance;
    }
    else if (joinFrontVehicleId_ >= 0) {
        PlatoonNeighborState front;
        simtime_t frontAge = SIMTIME_ZERO;
        if (getNeighbor(joinFrontVehicleId_, front, frontAge) && frontAge <= maxAge_) {
            in.fakeFront.valid = true;
            in.fakeFront.speed = front.scalarSpeed;
            in.fakeFront.actualAcceleration = front.actualAcceleration;
            in.fakeFront.controllerAcceleration = front.controllerAcceleration;
            in.fakeFront.distance = computeGapToNeighbor(front);
        }
    }

    switch (in.controlMode) {
        case ControlMode::LEADER_CRUISE:
            in.targetSpeed = leaderTargetSpeed_;
            break;

        case ControlMode::JOINER_FREE_CRUISE:
        case ControlMode::JOINER_WAIT_REPLY:
        case ControlMode::JOINER_WAIT_INFORMATION:
            in.targetSpeed = nominalPlatoonSpeed_;
            break;

        case ControlMode::FOLLOWER_PLATOON:
            if (in.predecessor.valid) {
                in.targetGap = getTargetDistance(in.predecessor.speed);
                in.targetSpeed = in.predecessor.speed;
            }
            else if (in.leader.valid) {
                in.targetSpeed = in.leader.speed;
            }
            else {
                in.targetSpeed = mySpeed;
            }
            break;

        case ControlMode::JOINER_MOVE_IN_POSITION: {
            const double spacing = controllerAdapter_.getCACCConstantSpacing();
            if (in.fakeFront.valid) {
                in.targetGap = (spacing > 0.0) ? spacing : getTargetDistance(in.fakeFront.speed);
            }
            else {
                in.targetGap = (spacing > 0.0) ? spacing : getTargetDistance(mySpeed);
            }
            const double cruiseTarget = controllerAdapter_.getCruiseControlDesiredSpeed();
            in.targetSpeed = (cruiseTarget > 0.0) ? cruiseTarget : (nominalPlatoonSpeed_ + approachDeltaV_);
            break;
        }

        case ControlMode::JOINER_WAIT_JOIN:
            if (in.fakeFront.valid) {
                in.targetGap = getTargetDistance(in.fakeFront.speed);
                in.targetSpeed = in.fakeFront.speed;
            }
            else {
                in.targetGap = getTargetDistance(mySpeed);
                in.targetSpeed = nominalPlatoonSpeed_;
            }
            break;

        case ControlMode::HOLD:
        default:
            in.targetSpeed = 0.0;
            break;
    }

    return in;
}

ControlOutput CarlaGeneralPlatooningApp::computeControllerOutput() const
{
    ControllerInputs in = buildControllerInputs();

    // A late-spawn joiner exists as a separate vehicle before the maneuver is triggered.
    // Do not freeze it just because the control mode is still HOLD.
    if (role_ == PlatoonRole::JOINER && !inManeuver_ && in.controlMode == ControlMode::HOLD) {
        in.controlMode = ControlMode::JOINER_FREE_CRUISE;
        in.activeController = ActiveController::CC;
        in.targetSpeed = nominalPlatoonSpeed_;
    }

    return controllerDispatcher_.compute(in);
}

void CarlaGeneralPlatooningApp::onControlTick()
{
    ControlOutput out = computeControllerOutput();

    const double rawAcceleration = out.desiredAcceleration;

    desiredAcceleration_ = clampValue(rawAcceleration, aMin_, aMax_);
    desiredSpeed_ = out.desiredSpeed;
    hasControl_ = out.hasControl;
    controlMode_ = out.controlMode;

    // -----------------------------------------------------------------
    // PLEXE-style exported metrics
    // -----------------------------------------------------------------
    veins::Coord vel(0, 0, 0);
    if (mobility_) vel = toVeinsCoord(mobility_->getCurrentVelocity());
    const double currentSpeed = scalarSpeedFromVelocity(vel);

    double measuredAcceleration = 0.0;
    if (lastExportTime_ >= SIMTIME_ZERO && simTime() > lastExportTime_) {
        const double dt = (simTime() - lastExportTime_).dbl();
        if (dt > 0.0) {
            measuredAcceleration = (currentSpeed - lastExportSpeed_) / dt;
        }
    }

    actualAcceleration_ = measuredAcceleration;
    lastExportSpeed_ = currentSpeed;
    lastExportTime_ = simTime();

    double exportedDistance = -1.0;
    double exportedRelativeSpeed = 0.0;

    const int myId = positionHelper_.getId();
    const int predId = positionHelper_.getPredecessorId();

    if (predId >= 0 && predId != myId) {
        PlatoonNeighborState pred;
        simtime_t predAge = SIMTIME_ZERO;
        const bool predOk = getNeighbor(predId, pred, predAge) && (predAge <= maxAge_);
        if (predOk) {
            exportedDistance = computeGapToNeighbor(pred);
            // Match the control-law sign convention already used in your app:
            // predecessor speed minus ego speed
            exportedRelativeSpeed = pred.scalarSpeed - currentSpeed;
        }
    }

    EV_INFO << "[CarlaGeneralPlatooningApp][onControlTick]"
            << " actor=" << actorId_
            << " role=" << static_cast<int>(role_)
            << " mode=" << static_cast<int>(controlMode_)
            << " activeController=" << static_cast<int>(controllerAdapter_.getActiveController())
            << " hasControl=" << hasControl_
            << " desiredSpeed=" << desiredSpeed_
            << " rawAcceleration=" << rawAcceleration
            << " clippedAcceleration=" << desiredAcceleration_
            << " measuredSpeed=" << currentSpeed
            << " measuredAcceleration=" << actualAcceleration_
            << " distance=" << exportedDistance
            << " relativeSpeed=" << exportedRelativeSpeed
            << "\n";

    // Existing bridge/control signals
    emit(desiredAccelerationSignal_, desiredAcceleration_);
    emit(desiredSpeedSignal_, desiredSpeed_);
    emit(hasControlSignal_, hasControl_ ? 1.0 : 0.0);
    emit(controlModeSignal_, static_cast<long>(static_cast<int>(controlMode_)));
    emit(activeControllerSignal_, static_cast<long>(static_cast<int>(controllerAdapter_.getActiveController())));

    // PLEXE-style exported vectors for .vec parsing
    emit(speedSignal_, currentSpeed);
    emit(accelerationSignal_, actualAcceleration_);
    emit(controllerAccelerationExportSignal_, desiredAcceleration_);
    emit(distanceSignal_, exportedDistance);
    emit(relativeSpeedSignal_, exportedRelativeSpeed);
}

void CarlaGeneralPlatooningApp::maneuverSetControlMode(ControlMode mode)
{
    controllerAdapter_.setControlMode(mode);
    controlMode_ = mode;
}

void CarlaGeneralPlatooningApp::maneuverSetJoinFrontVehicleId(int frontId)
{
    joinFrontVehicleId_ = frontId;
}

void CarlaGeneralPlatooningApp::maneuverClearJoinFrontVehicleId()
{
    joinFrontVehicleId_ = -1;
}

void CarlaGeneralPlatooningApp::maneuverSetPlatoonSpeed(double platoonSpeed)
{
    nominalPlatoonSpeed_ = platoonSpeed;
    positionHelper_.setPlatoonSpeed(platoonSpeed);
}

void CarlaGeneralPlatooningApp::maneuverSetFormation(const std::vector<int>& formation)
{
    positionHelper_.setPlatoonFormation(formation);
}

void CarlaGeneralPlatooningApp::maneuverCompleteJoinAsFollower(
    const std::vector<int>& formation,
    int platoonId,
    int leaderId)
{
    role_ = PlatoonRole::FOLLOWER;
    positionHelper_.setPlatoonFormation(formation);
    positionHelper_.setPlatoonId(platoonId);
    positionHelper_.setLeaderId(leaderId);
    controllerAdapter_.setControlMode(ControlMode::FOLLOWER_PLATOON);
    controlMode_ = ControlMode::FOLLOWER_PLATOON;
    joinFrontVehicleId_ = -1;
    platoonId_ = platoonId;
    leaderId_ = leaderId;
}

} // namespace carla