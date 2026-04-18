#include "carla/platooning/maneuver/CarlaJoinAtBack.h"

#include <cmath>
#include <sstream>

namespace {

template <typename MsgT>
bool formationMatches(const MsgT* msg, const std::vector<int>& expected)
{
    if (!msg) return false;
    if (msg->getNewPlatoonFormationArraySize() != expected.size()) return false;

    for (size_t i = 0; i < expected.size(); ++i) {
        if (msg->getNewPlatoonFormation(i) != expected[i]) return false;
    }
    return true;
}

omnetpp::cSimpleModule* asSimpleModule(carla::ICarlaPlatooningApp* app)
{
    auto* mod = dynamic_cast<omnetpp::cSimpleModule*>(app);
    if (!mod) {
        throw omnetpp::cRuntimeError("CarlaJoinAtBack: app is not a cSimpleModule");
    }
    return mod;
}

void scheduleOnApp(carla::ICarlaPlatooningApp* app, omnetpp::simtime_t when, omnetpp::cMessage* msg)
{
    asSimpleModule(app)->scheduleAt(when, msg);
}

void cancelOnApp(carla::ICarlaPlatooningApp* app, omnetpp::cMessage* msg)
{
    if (!msg) return;
    if (!msg->isScheduled()) return;
    asSimpleModule(app)->cancelEvent(msg);
}

} // anonymous namespace

namespace carla {

CarlaJoinAtBack::CarlaJoinAtBack(ICarlaPlatooningApp* app)
    : CarlaJoinManeuver(app)
{
    retryTimer_ = new omnetpp::cMessage("joinAtBackRetry");
}

CarlaJoinAtBack::~CarlaJoinAtBack()
{
    if (retryTimer_) {
        if (retryTimer_->isScheduled()) {
            cancelOnApp(app_, retryTimer_);
        }
        delete retryTimer_;
        retryTimer_ = nullptr;
    }
}

void CarlaJoinAtBack::resetJoinTracking()
{
    inPositionSampleCount_ = 0;
}

double CarlaJoinAtBack::computeTargetJoinGapMeters(double frontSpeedMetersPerSecond) const
{
    const double frontSpeed = std::max(0.0, frontSpeedMetersPerSecond);
    const auto targetController = app_->getTargetController();

    const double standstill = app_->getStandstillDistance(targetController);
    const double headway = app_->getHeadway(targetController);

    return standstill + headway * frontSpeed;
}

bool CarlaJoinAtBack::updateInPositionConvergence(double actualGapMeters, double targetGapMeters)
{
    const double gapError = actualGapMeters - targetGapMeters;

    if (std::fabs(gapError) <= kJoinGapToleranceMeters_) {
        ++inPositionSampleCount_;
    }
    else {
        inPositionSampleCount_ = 0;
    }

    return inPositionSampleCount_ >= kInPositionRequiredSamples_;
}

bool CarlaJoinAtBack::initializeJoinManeuver(const void* parameters)
{
    const auto* pars = static_cast<const JoinManeuverParameters*>(parameters);
    if (!pars) return false;

    if (joinManeuverState_ != JoinManeuverState::IDLE) return false;
    if (app_->isInManeuver()) return false;

    app_->setInManeuver(true, this);
    app_->setPlatoonRole(PlatoonRole::JOINER);
    positionHelper_->setController(ActiveController::CC);
    controllerAdapter_->setActiveController(ActiveController::CC);

    targetPlatoonData_ = std::make_unique<TargetPlatoonData>();
    targetPlatoonData_->platoonId = pars->platoonId;
    targetPlatoonData_->platoonLeader = pars->leaderId;

    joinReqRetries_ = 0;
    resetJoinTracking();

    joinManeuverState_ = JoinManeuverState::J_WAIT_REPLY;
    controllerAdapter_->setControlMode(ControlMode::JOINER_WAIT_REPLY);

    EV_INFO << "[CarlaJoinAtBack][initializeJoinManeuver]"
            << " actor=" << positionHelper_->getExternalId()
            << " platoonId=" << pars->platoonId
            << " leaderId=" << pars->leaderId
            << " position=" << pars->position
            << " state=J_WAIT_REPLY"
            << "\n";

    return true;
}

void CarlaJoinAtBack::sendJoinRequest()
{
    if (!targetPlatoonData_) return;

    const veins::Coord pos = app_->getCurrentPosition();
    const int currentLane = app_->getCurrentLaneIndex();

    JoinPlatoonRequest* req = createJoinPlatoonRequest(
        positionHelper_->getId(),
        positionHelper_->getExternalId(),
        targetPlatoonData_->platoonId,
        targetPlatoonData_->platoonLeader,
        currentLane,
        pos.x,
        pos.y);

    EV_INFO << "[CarlaJoinAtBack][sendJoinRequest]"
            << " actor=" << positionHelper_->getExternalId()
            << " dstLeader=" << targetPlatoonData_->platoonLeader
            << " platoonId=" << targetPlatoonData_->platoonId
            << " try=" << (joinReqRetries_ + 1)
            << "\n";

    app_->sendUnicast(req, targetPlatoonData_->platoonLeader);
}

void CarlaJoinAtBack::startManeuver(const void* parameters)
{
    if (!initializeJoinManeuver(parameters)) return;

    sendJoinRequest();

    if (retryTimer_ && !retryTimer_->isScheduled()) {
        scheduleOnApp(app_, omnetpp::simTime() + kJoinReqRetrySeconds_, retryTimer_);
    }
}

bool CarlaJoinAtBack::handleSelfMsg(omnetpp::cMessage* msg)
{
    if (msg != retryTimer_) return false;

    if (joinManeuverState_ != JoinManeuverState::J_WAIT_REPLY) {
        return true;
    }

    if (!targetPlatoonData_) {
        return true;
    }

    if (joinReqRetries_ + 1 >= kJoinReqMaxTries_) {
        EV_WARN << "[CarlaJoinAtBack][handleSelfMsg]"
                << " actor=" << positionHelper_->getExternalId()
                << " warning=join_req_timeout"
                << " tries=" << (joinReqRetries_ + 1)
                << "\n";
        abortManeuver();
        return true;
    }

    ++joinReqRetries_;
    sendJoinRequest();
    scheduleOnApp(app_, omnetpp::simTime() + kJoinReqRetrySeconds_, retryTimer_);
    return true;
}

void CarlaJoinAtBack::abortManeuver()
{
    if (retryTimer_ && retryTimer_->isScheduled()) {
        cancelOnApp(app_, retryTimer_);
    }

    const PlatoonRole currentRole = app_->getPlatoonRole();

    joinManeuverState_ = JoinManeuverState::IDLE;
    targetPlatoonData_.reset();
    joinerData_.reset();
    joinReqRetries_ = 0;
    resetJoinTracking();

    if (currentRole == PlatoonRole::JOINER) {
        app_->setPlatoonRole(PlatoonRole::JOINER);
        positionHelper_->setController(ActiveController::CC);
        controllerAdapter_->setActiveController(ActiveController::CC);
        controllerAdapter_->setControlMode(ControlMode::JOINER_FREE_CRUISE);
    }
    else if (currentRole == PlatoonRole::LEADER) {
        app_->setPlatoonRole(PlatoonRole::LEADER);
        positionHelper_->setController(ActiveController::CC);
        controllerAdapter_->setActiveController(ActiveController::CC);
        controllerAdapter_->setControlMode(ControlMode::LEADER_CRUISE);
    }
    else {
        controllerAdapter_->setActiveController(ActiveController::UNKNOWN);
        controllerAdapter_->setControlMode(ControlMode::HOLD);
    }

    app_->setInManeuver(false, nullptr);

    EV_INFO << "[CarlaJoinAtBack][abortManeuver]"
            << " actor=" << positionHelper_->getExternalId()
            << " state=IDLE"
            << " role=" << static_cast<int>(app_->getPlatoonRole())
            << "\n";
}

void CarlaJoinAtBack::onPlatoonBeacon(const PlatooningBeacon* pb)
{
    if (!pb) return;
    if (joinManeuverState_ != JoinManeuverState::J_MOVE_IN_POSITION) return;
    if (app_->getPlatoonRole() != PlatoonRole::JOINER) return;
    if (!targetPlatoonData_) return;
    if (targetPlatoonData_->newFormation.empty()) return;

    if (pb->getVehicleId() == targetPlatoonData_->newFormation.at(0)) {
        controllerAdapter_->setLeaderVehicleFakeData(
            pb->getControllerAcceleration(),
            pb->getAcceleration(),
            pb->getSpeed());
    }

    const int frontId = targetPlatoonData_->frontId();
    if (frontId < 0) return;
    if (pb->getVehicleId() != frontId) return;

    const veins::Coord frontPos(pb->getPositionX(), pb->getPositionY(), 0);
    const veins::Coord myPos = app_->getCurrentPosition();

    const double distance = myPos.distance(frontPos) - pb->getLength();
    const double targetGap = computeTargetJoinGapMeters(pb->getSpeed());
    const double gapError = distance - targetGap;

    controllerAdapter_->setFrontVehicleFakeData(
        pb->getControllerAcceleration(),
        pb->getAcceleration(),
        pb->getSpeed(),
        distance);

    const bool ready = updateInPositionConvergence(distance, targetGap);

    EV_INFO << "[CarlaJoinAtBack][onPlatoonBeacon]"
            << " actor=" << positionHelper_->getExternalId()
            << " frontId=" << frontId
            << " distance=" << distance
            << " targetGap=" << targetGap
            << " gapError=" << gapError
            << " samples=" << inPositionSampleCount_
            << " ready=" << (ready ? 1 : 0)
            << "\n";

    if (!ready) return;

    MoveToPositionAck* ack = createMoveToPositionAck(
        positionHelper_->getId(),
        positionHelper_->getExternalId(),
        targetPlatoonData_->platoonId,
        targetPlatoonData_->platoonLeader,
        targetPlatoonData_->platoonSpeed,
        targetPlatoonData_->platoonLane,
        targetPlatoonData_->newFormation);

    EV_INFO << "[CarlaJoinAtBack][onPlatoonBeacon]"
            << " actor=" << positionHelper_->getExternalId()
            << " frontId=" << frontId
            << " distance=" << distance
            << " targetGap=" << targetGap
            << " action=send_MoveToPositionAck"
            << "\n";

    app_->sendUnicast(ack, targetPlatoonData_->platoonLeader);

    joinManeuverState_ = JoinManeuverState::J_WAIT_JOIN;
    controllerAdapter_->setControlMode(ControlMode::JOINER_WAIT_JOIN);
    resetJoinTracking();
}

void CarlaJoinAtBack::onFailedTransmissionAttempt(const ManeuverMessage* mm)
{
    throw omnetpp::cRuntimeError(
        "Failed transmission attempt for maneuver packet: %s",
        mm ? mm->getName() : "<null>");
}

bool CarlaJoinAtBack::processJoinRequest(const JoinPlatoonRequest* msg)
{
    if (!msg) return false;

    if (msg->getPlatoonId() != positionHelper_->getPlatoonId()) return false;

    if (app_->getPlatoonRole() != PlatoonRole::LEADER &&
        app_->getPlatoonRole() != PlatoonRole::NONE) {
        return false;
    }

    const bool permission =
        ((app_->getPlatoonRole() == PlatoonRole::LEADER) ||
         (app_->getPlatoonRole() == PlatoonRole::NONE)) &&
        !app_->isInManeuver();

    JoinPlatoonResponse* response = createJoinPlatoonResponse(
        positionHelper_->getId(),
        positionHelper_->getExternalId(),
        msg->getPlatoonId(),
        msg->getVehicleId(),
        permission);

    app_->sendUnicast(response, msg->getVehicleId());

    EV_INFO << "[CarlaJoinAtBack][processJoinRequest]"
            << " actor=" << positionHelper_->getExternalId()
            << " joinerId=" << msg->getVehicleId()
            << " permitted=" << (permission ? 1 : 0)
            << "\n";

    if (!permission) return false;

    app_->setInManeuver(true, this);
    app_->setPlatoonRole(PlatoonRole::LEADER);

    positionHelper_->setController(ActiveController::CC);
    controllerAdapter_->setActiveController(ActiveController::CC);
    controllerAdapter_->setControlMode(ControlMode::LEADER_CRUISE);
    controllerAdapter_->setFixedLane(app_->getCurrentLaneIndex());
    positionHelper_->setPlatoonLane(app_->getCurrentLaneIndex());

    joinerData_ = std::make_unique<JoinerData>();
    joinerData_->from(msg);

    std::vector<int> newFormation = positionHelper_->getPlatoonFormation();
    if (newFormation.empty()) {
        newFormation.push_back(positionHelper_->getId());
    }

    if (std::find(newFormation.begin(), newFormation.end(), joinerData_->joinerId) == newFormation.end()) {
        newFormation.push_back(joinerData_->joinerId);
    }

    joinerData_->newFormation = newFormation;

    joinManeuverState_ = JoinManeuverState::L_WAIT_JOINER_IN_POSITION;
    return true;
}

void CarlaJoinAtBack::handleJoinPlatoonRequest(const JoinPlatoonRequest* msg)
{
    if (!processJoinRequest(msg)) return;
    if (!joinerData_) return;

    MoveToPosition* mtp = createMoveToPosition(
        positionHelper_->getId(),
        positionHelper_->getExternalId(),
        positionHelper_->getPlatoonId(),
        joinerData_->joinerId,
        positionHelper_->getPlatoonSpeed(),
        positionHelper_->getPlatoonLane(),
        joinerData_->newFormation);

    EV_INFO << "[CarlaJoinAtBack][handleJoinPlatoonRequest]"
            << " actor=" << positionHelper_->getExternalId()
            << " joinerId=" << joinerData_->joinerId
            << " action=send_MoveToPosition"
            << "\n";

    app_->sendUnicast(mtp, joinerData_->joinerId);
}

void CarlaJoinAtBack::handleJoinPlatoonResponse(const JoinPlatoonResponse* msg)
{
    if (!msg) return;
    if (app_->getPlatoonRole() != PlatoonRole::JOINER) return;
    if (joinManeuverState_ != JoinManeuverState::J_WAIT_REPLY) return;
    if (!targetPlatoonData_) return;

    if (msg->getPlatoonId() != targetPlatoonData_->platoonId) return;
    if (msg->getVehicleId() != targetPlatoonData_->platoonLeader) return;

    if (retryTimer_ && retryTimer_->isScheduled()) {
        cancelOnApp(app_, retryTimer_);
    }

    if (msg->getPermitted()) {
        joinManeuverState_ = JoinManeuverState::J_WAIT_INFORMATION;
        controllerAdapter_->setControlMode(ControlMode::JOINER_WAIT_INFORMATION);
        controllerAdapter_->setFixedLane(app_->getCurrentLaneIndex());
        resetJoinTracking();

        EV_INFO << "[CarlaJoinAtBack][handleJoinPlatoonResponse]"
                << " actor=" << positionHelper_->getExternalId()
                << " leaderId=" << msg->getVehicleId()
                << " permitted=1"
                << " state=J_WAIT_INFORMATION"
                << "\n";
    }
    else {
        EV_WARN << "[CarlaJoinAtBack][handleJoinPlatoonResponse]"
                << " actor=" << positionHelper_->getExternalId()
                << " leaderId=" << msg->getVehicleId()
                << " permitted=0"
                << "\n";

        joinManeuverState_ = JoinManeuverState::IDLE;
        app_->setPlatoonRole(PlatoonRole::JOINER);
        positionHelper_->setController(ActiveController::CC);
        controllerAdapter_->setActiveController(ActiveController::CC);
        controllerAdapter_->setControlMode(ControlMode::JOINER_FREE_CRUISE);
        app_->setInManeuver(false, nullptr);
        targetPlatoonData_.reset();
        joinReqRetries_ = 0;
        resetJoinTracking();
    }
}

void CarlaJoinAtBack::handleMoveToPosition(const MoveToPosition* msg)
{
    if (!msg) return;
    if (app_->getPlatoonRole() != PlatoonRole::JOINER) return;
    if (joinManeuverState_ != JoinManeuverState::J_WAIT_INFORMATION) return;
    if (!targetPlatoonData_) return;

    if (msg->getPlatoonId() != targetPlatoonData_->platoonId) return;
    if (msg->getVehicleId() != targetPlatoonData_->platoonLeader) return;

    targetPlatoonData_->from(msg);
    resetJoinTracking();

    const int currentLane = app_->getCurrentLaneIndex();
    if (currentLane != targetPlatoonData_->platoonLane) {
        controllerAdapter_->setFixedLane(targetPlatoonData_->platoonLane);
    }

    const double targetGap = computeTargetJoinGapMeters(targetPlatoonData_->platoonSpeed);
    const double approachGap = targetGap + kApproachGapBufferMeters_;

    positionHelper_->setController(ActiveController::FAKED_CACC);
    controllerAdapter_->setCACCConstantSpacing(approachGap);
    controllerAdapter_->setLeaderVehicleFakeData(0.0, 0.0, targetPlatoonData_->platoonSpeed);
    controllerAdapter_->setFrontVehicleFakeData(0.0, 0.0, targetPlatoonData_->platoonSpeed, approachGap);
    controllerAdapter_->setCruiseControlDesiredSpeed(targetPlatoonData_->platoonSpeed + kApproachDeltaV_);
    controllerAdapter_->setActiveController(ActiveController::FAKED_CACC);

    controllerAdapter_->setControlMode(ControlMode::JOINER_MOVE_IN_POSITION);

    joinManeuverState_ = JoinManeuverState::J_MOVE_IN_POSITION;

    EV_INFO << "[CarlaJoinAtBack][handleMoveToPosition]"
            << " actor=" << positionHelper_->getExternalId()
            << " platoonSpeed=" << targetPlatoonData_->platoonSpeed
            << " frontId=" << targetPlatoonData_->frontId()
            << " targetGap=" << targetGap
            << " approachGap=" << approachGap
            << " state=J_MOVE_IN_POSITION"
            << "\n";
}

void CarlaJoinAtBack::handleMoveToPositionAck(const MoveToPositionAck* msg)
{
    if (!msg) return;
    if (app_->getPlatoonRole() != PlatoonRole::LEADER) return;
    if (joinManeuverState_ != JoinManeuverState::L_WAIT_JOINER_IN_POSITION) return;
    if (!joinerData_) return;

    if (msg->getPlatoonId() != positionHelper_->getPlatoonId()) return;
    if (msg->getVehicleId() != joinerData_->joinerId) return;
    if (!formationMatches(msg, joinerData_->newFormation)) return;

    JoinFormation* jf = createJoinFormation(
        positionHelper_->getId(),
        positionHelper_->getExternalId(),
        positionHelper_->getPlatoonId(),
        joinerData_->joinerId,
        positionHelper_->getPlatoonSpeed(),
        positionHelper_->getPlatoonLane(),
        joinerData_->newFormation);

    EV_INFO << "[CarlaJoinAtBack][handleMoveToPositionAck]"
            << " actor=" << positionHelper_->getExternalId()
            << " joinerId=" << joinerData_->joinerId
            << " action=send_JoinFormation"
            << "\n";

    app_->sendUnicast(jf, joinerData_->joinerId);
    joinManeuverState_ = JoinManeuverState::L_WAIT_JOINER_TO_JOIN;
}

void CarlaJoinAtBack::handleJoinFormation(const JoinFormation* msg)
{
    if (!msg) return;
    if (app_->getPlatoonRole() != PlatoonRole::JOINER) return;
    if (joinManeuverState_ != JoinManeuverState::J_WAIT_JOIN) return;
    if (!targetPlatoonData_) return;

    if (msg->getPlatoonId() != targetPlatoonData_->platoonId) return;
    if (msg->getVehicleId() != targetPlatoonData_->platoonLeader) return;
    if (!formationMatches(msg, targetPlatoonData_->newFormation)) return;

    controllerAdapter_->setActiveController(app_->getTargetController());
    positionHelper_->setController(app_->getTargetController());
    positionHelper_->setDistance(app_->getStandstillDistance(app_->getTargetController()));
    positionHelper_->setHeadway(app_->getHeadway(app_->getTargetController()));

    if (positionHelper_->getController() == ActiveController::CACC) {
        controllerAdapter_->setCACCConstantSpacing(app_->getTargetDistance(targetPlatoonData_->platoonSpeed));
    }

    positionHelper_->setPlatoonId(msg->getPlatoonId());
    positionHelper_->setPlatoonLane(targetPlatoonData_->platoonLane);
    positionHelper_->setPlatoonSpeed(targetPlatoonData_->platoonSpeed);

    std::vector<int> formation;
    formation.reserve(msg->getNewPlatoonFormationArraySize());
    for (unsigned int i = 0; i < msg->getNewPlatoonFormationArraySize(); ++i) {
        formation.push_back(msg->getNewPlatoonFormation(i));
    }
    positionHelper_->setPlatoonFormation(formation);

    JoinFormationAck* ack = createJoinFormationAck(
        positionHelper_->getId(),
        positionHelper_->getExternalId(),
        positionHelper_->getPlatoonId(),
        positionHelper_->getLeaderId(),
        positionHelper_->getPlatoonSpeed(),
        positionHelper_->getPlatoonLane(),
        formation);

    app_->sendUnicast(ack, positionHelper_->getLeaderId());

    app_->setPlatoonRole(PlatoonRole::FOLLOWER);
    controllerAdapter_->setControlMode(ControlMode::FOLLOWER_PLATOON);

    joinManeuverState_ = JoinManeuverState::IDLE;
    app_->setInManeuver(false, nullptr);
    resetJoinTracking();

    EV_INFO << "[CarlaJoinAtBack][handleJoinFormation]"
            << " actor=" << positionHelper_->getExternalId()
            << " action=join_complete"
            << " role=FOLLOWER"
            << "\n";

    targetPlatoonData_.reset();
    joinerData_.reset();
    joinReqRetries_ = 0;
}

void CarlaJoinAtBack::handleJoinFormationAck(const JoinFormationAck* msg)
{
    if (!msg) return;
    if (app_->getPlatoonRole() != PlatoonRole::LEADER) return;
    if (joinManeuverState_ != JoinManeuverState::L_WAIT_JOINER_TO_JOIN) return;
    if (!joinerData_) return;

    if (msg->getPlatoonId() != positionHelper_->getPlatoonId()) return;
    if (msg->getVehicleId() != joinerData_->joinerId) return;
    if (!formationMatches(msg, joinerData_->newFormation)) return;

    positionHelper_->setPlatoonFormation(joinerData_->newFormation);

    for (int i = 1; i < positionHelper_->getPlatoonSize(); ++i) {
        const int dest = positionHelper_->getMemberId(i);

        UpdatePlatoonFormation* upd = app_->createUpdatePlatoonFormation(
            positionHelper_->getId(),
            positionHelper_->getExternalId(),
            positionHelper_->getPlatoonId(),
            dest,
            positionHelper_->getPlatoonSpeed(),
            positionHelper_->getPlatoonLane(),
            joinerData_->newFormation);

        app_->sendUnicast(upd, dest);
    }

    joinManeuverState_ = JoinManeuverState::IDLE;
    app_->setInManeuver(false, nullptr);

    EV_INFO << "[CarlaJoinAtBack][handleJoinFormationAck]"
            << " actor=" << positionHelper_->getExternalId()
            << " joinerId=" << joinerData_->joinerId
            << " action=broadcast_UpdatePlatoonFormation"
            << "\n";

    targetPlatoonData_.reset();
    joinerData_.reset();
    resetJoinTracking();
}

} // namespace carla