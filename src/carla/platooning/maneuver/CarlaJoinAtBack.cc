#include "carla/platooning/maneuver/CarlaJoinAtBack.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <string>

namespace {

// Checks whether a generated maneuver message carries exactly the expected formation.
// Formation equality is used as a guard so stale or unrelated ACKs do not complete a join.
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

// Converts the abstract platooning app interface back to an OMNeT++ module.
// Needed because scheduling/canceling self-messages is provided by cSimpleModule.
omnetpp::cSimpleModule* asSimpleModule(carla::ICarlaPlatooningApp* app)
{
    auto* mod = dynamic_cast<omnetpp::cSimpleModule*>(app);
    if (!mod) {
        throw omnetpp::cRuntimeError("CarlaJoinAtBack: app is not a cSimpleModule");
    }
    return mod;
}

// Schedules a maneuver-owned timer on the OMNeT++ app module.
// Timers are used for retrying the initial join request.
void scheduleOnApp(carla::ICarlaPlatooningApp* app, omnetpp::simtime_t when, omnetpp::cMessage* msg)
{
    asSimpleModule(app)->scheduleAt(when, msg);
}

// Cancels a maneuver-owned timer if it is currently scheduled.
// OMNeT++ requires cancelEvent() before deleting or reusing a scheduled self-message.
void cancelOnApp(carla::ICarlaPlatooningApp* app, omnetpp::cMessage* msg)
{
    if (!msg) return;
    if (!msg->isScheduled()) return;
    asSimpleModule(app)->cancelEvent(msg);
}

// Reads a distance parameter from the owning app module.
// Accepts OMNeT++ unit syntax such as "11m" and falls back to a raw double if no unit is present.
double readMetersParam(carla::ICarlaPlatooningApp* app, const char* name, double fallback)
{
    if (!app || !name) return fallback;

    omnetpp::cSimpleModule* mod = nullptr;
    try {
        mod = asSimpleModule(app);
    }
    catch (...) {
        return fallback;
    }

    if (!mod || mod->findPar(name) < 0) return fallback;

    try {
        return mod->par(name).doubleValueInUnit("m");
    }
    catch (...) {
        try {
            return mod->par(name).doubleValue();
        }
        catch (...) {
            return fallback;
        }
    }
}

// Reads a speed parameter from the owning app module.
// Accepts OMNeT++ unit syntax such as "2mps" and falls back to a raw double if no unit is present.
double readSpeedParam(carla::ICarlaPlatooningApp* app, const char* name, double fallback)
{
    if (!app || !name) return fallback;

    omnetpp::cSimpleModule* mod = nullptr;
    try {
        mod = asSimpleModule(app);
    }
    catch (...) {
        return fallback;
    }

    if (!mod || mod->findPar(name) < 0) return fallback;

    try {
        return mod->par(name).doubleValueInUnit("mps");
    }
    catch (...) {
        try {
            return mod->par(name).doubleValue();
        }
        catch (...) {
            return fallback;
        }
    }
}

} // anonymous namespace

namespace carla {

// Creates the join-at-back maneuver state machine for one vehicle.
// The same class runs on leaders and joiners; behavior depends on the app's PlatoonRole.
CarlaJoinAtBack::CarlaJoinAtBack(ICarlaPlatooningApp* app)
    : CarlaJoinManeuver(app)
{
    retryTimer_ = new omnetpp::cMessage("joinAtBackRetry");
}

// Cleans up the retry timer owned by this maneuver.
// The timer must be canceled before deletion if it is still scheduled in OMNeT++.
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

// Computes the final desired gap behind the front vehicle during/after joining.
// The spacing policy is delegated to the app so CDS/CTS changes stay centralized.
double CarlaJoinAtBack::computeTargetJoinGapMeters(double frontSpeedMetersPerSecond) const
{
    const double frontSpeed = std::max(0.0, frontSpeedMetersPerSecond); // m/s; negative speeds are invalid for spacing.
    const auto targetController = app_->getTargetController();

    // Use the app-level spacing policy.
    // CDS usually returns gap_min only; CTS may return gap_min + headway * speed.
    const double targetGap = app_->getTargetDistance(targetController, frontSpeed);

    if (std::isfinite(targetGap) && targetGap >= 0.0) {
        return targetGap;
    }

    // Defensive fallback only. Normal runs should use getTargetDistance().
    const double standstill = app_->getStandstillDistance(targetController); // m.
    const double headway = app_->getHeadway(targetController);               // s.
    return standstill + headway * frontSpeed;
}

// Initializes joiner-side state for a new join-at-back maneuver.
// This puts the joiner into J_WAIT_REPLY, clears stale fake vehicle data, and prepares target platoon metadata.
bool CarlaJoinAtBack::initializeJoinManeuver(const void* parameters)
{
    const auto* pars = static_cast<const JoinManeuverParameters*>(parameters);
    if (!pars) return false;

    if (joinManeuverState_ != JoinManeuverState::IDLE) return false;
    if (app_->isInManeuver()) return false;

    app_->setInManeuver(true, this);
    app_->setPlatoonRole(PlatoonRole::JOINER);

    // Fake data is maneuver-local. It should not survive across join attempts.
    controllerAdapter_->clearLeaderVehicleFakeData();
    controllerAdapter_->clearFrontVehicleFakeData();

    positionHelper_->setController(ActiveController::CC);
    controllerAdapter_->setActiveController(ActiveController::CC);
    controllerAdapter_->setControlMode(ControlMode::JOINER_WAIT_REPLY);

    targetPlatoonData_ = std::make_unique<TargetPlatoonData>();
    targetPlatoonData_->platoonId = pars->platoonId;       // Target platoon id from omnetpp.ini/start parameters.
    targetPlatoonData_->platoonLeader = pars->leaderId;    // Vehicle id that should answer the request.

    joinReqRetries_ = 0;
    joinManeuverState_ = JoinManeuverState::J_WAIT_REPLY;

    EV_INFO << "[CarlaJoinAtBack][initializeJoinManeuver]"
            << " actor=" << positionHelper_->getExternalId()
            << " platoonId=" << pars->platoonId
            << " leaderId=" << pars->leaderId
            << " position=" << pars->position
            << " in_pos_slack=" << readMetersParam(app_, "in_pos_slack", kPlexeJoinInPositionSlackMeters_)
            << " approach_spacing=" << readMetersParam(app_, "approach_spacing", kPlexeApproachSpacingMeters_)
            << " approach_delta_v=" << readSpeedParam(app_, "approach_delta_v", kApproachDeltaV_)
            << " state=J_WAIT_REPLY"
            << "\n";

    return true;
}

// Sends JoinPlatoonRequest from the joiner to the target leader.
// This is the first message in the baseline centralized join-at-back sequence.
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

// Starts the join maneuver from the joiner side.
// Sends the first request immediately, then schedules a retry timer if no response arrives.
void CarlaJoinAtBack::startManeuver(const void* parameters)
{
    if (!initializeJoinManeuver(parameters)) return;

    sendJoinRequest();

    if (retryTimer_ && !retryTimer_->isScheduled()) {
        scheduleOnApp(app_, omnetpp::simTime() + kJoinReqRetrySeconds_, retryTimer_);
    }
}

// Handles the join request retry timer.
// Retries while the joiner is waiting for JoinPlatoonResponse; aborts after kJoinReqMaxTries_.
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

// Aborts the current maneuver and restores the vehicle to a safe role-specific mode.
// This is used for request timeout and should leave no fake leader/front data behind.
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

    // Fake data is maneuver-only state. Never let it survive abort.
    controllerAdapter_->clearLeaderVehicleFakeData();
    controllerAdapter_->clearFrontVehicleFakeData();

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

// Joiner-side beacon hook used while moving into position.
// Updates fake leader/front vehicle data from V2V beacons and sends MoveToPositionAck once close enough.
void CarlaJoinAtBack::onPlatoonBeacon(const PlatooningBeacon* pb)
{
    if (!pb) return;
    if (joinManeuverState_ != JoinManeuverState::J_MOVE_IN_POSITION) return;
    if (app_->getPlatoonRole() != PlatoonRole::JOINER) return;
    if (!targetPlatoonData_) return;
    if (targetPlatoonData_->newFormation.empty()) return;

    // The first vehicle in the formation is treated as the platoon leader.
    if (pb->getVehicleId() == targetPlatoonData_->newFormation.at(0)) {
        controllerAdapter_->setLeaderVehicleFakeData(
            pb->getControllerAcceleration(),
            pb->getAcceleration(),
            pb->getSpeed());
    }

    const int frontId = targetPlatoonData_->frontId(); // Vehicle directly ahead of the joiner in the proposed formation.
    if (frontId < 0) return;
    if (pb->getVehicleId() != frontId) return;

    const veins::Coord frontPos(pb->getPositionX(), pb->getPositionY(), 0);
    const veins::Coord myPos = app_->getCurrentPosition();

    const double distance = myPos.distance(frontPos) - pb->getLength(); // m; approximate bumper gap to front vehicle.

    /*
     * Plexe-style in-position rule:
     *
     *     distance < targetDistance(platoonSpeed) + 11
     *
     * This is intentionally not exact convergence to the final targetGap.
     * Plexe sends MoveToPositionAck while the joiner is still behind the final
     * platoon gap; JoinFormation then switches it into normal follower control.
     */
    const double targetGap = computeTargetJoinGapMeters(pb->getSpeed());
    const double inPositionSlack =
        readMetersParam(app_, "in_pos_slack", kPlexeJoinInPositionSlackMeters_); // m; ACK tolerance beyond target gap.

    const double inPositionThreshold = targetGap + inPositionSlack; // m; distance below this triggers MoveToPositionAck.
    const double gapError = distance - targetGap;                   // m; positive means still farther than final gap.
    const bool ready = (distance < inPositionThreshold);

    // Fake-front data is consumed by CarlaGeneralPlatooningApp::buildControllerInputs().
    controllerAdapter_->setFrontVehicleFakeData(
        pb->getControllerAcceleration(),
        pb->getAcceleration(),
        pb->getSpeed(),
        distance);

    EV_INFO << "[CarlaJoinAtBack][onPlatoonBeacon]"
            << " actor=" << positionHelper_->getExternalId()
            << " frontId=" << frontId
            << " distance=" << distance
            << " targetGap=" << targetGap
            << " inPositionSlack=" << inPositionSlack
            << " inPositionThreshold=" << inPositionThreshold
            << " gapError=" << gapError
            << " caccSpacing=" << controllerAdapter_->getCACCConstantSpacing()
            << " ready=" << (ready ? 1 : 0)
            << " parity=PLEXE_THRESHOLD"
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
            << " inPositionSlack=" << inPositionSlack
            << " inPositionThreshold=" << inPositionThreshold
            << " action=send_MoveToPositionAck"
            << "\n";

    app_->sendUnicast(ack, targetPlatoonData_->platoonLeader);

    joinManeuverState_ = JoinManeuverState::J_WAIT_JOIN;
    controllerAdapter_->setControlMode(ControlMode::JOINER_WAIT_JOIN);
}

// Fails loudly if the maneuver layer reports a failed send.
// Current implementation assumes the lower layer accepts scheduled transmissions.
void CarlaJoinAtBack::onFailedTransmissionAttempt(const ManeuverMessage* mm)
{
    throw omnetpp::cRuntimeError(
        "Failed transmission attempt for maneuver packet: %s",
        mm ? mm->getName() : "<null>");
}

// Leader-side validation and acceptance of a JoinPlatoonRequest.
// If permitted, the leader stores joiner data and enters L_WAIT_JOINER_IN_POSITION.
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

    // Clear stale local fake data on leader side as well.
    controllerAdapter_->clearLeaderVehicleFakeData();
    controllerAdapter_->clearFrontVehicleFakeData();

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

    // Join-at-back appends the joiner to the end of the current formation.
    if (std::find(newFormation.begin(), newFormation.end(), joinerData_->joinerId) == newFormation.end()) {
        newFormation.push_back(joinerData_->joinerId);
    }

    joinerData_->newFormation = newFormation;

    joinManeuverState_ = JoinManeuverState::L_WAIT_JOINER_IN_POSITION;
    return true;
}

// Leader-side handler for JoinPlatoonRequest.
// After accepting the request, sends MoveToPosition with the proposed formation.
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

// Joiner-side handler for JoinPlatoonResponse.
// A positive response moves the joiner to J_WAIT_INFORMATION until MoveToPosition arrives.
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

        controllerAdapter_->clearLeaderVehicleFakeData();
        controllerAdapter_->clearFrontVehicleFakeData();

        app_->setInManeuver(false, nullptr);
        targetPlatoonData_.reset();
        joinReqRetries_ = 0;
    }
}

// Joiner-side handler for MoveToPosition.
// Stores the proposed formation, configures fake-CACC approach control, and enters J_MOVE_IN_POSITION.
void CarlaJoinAtBack::handleMoveToPosition(const MoveToPosition* msg)
{
    if (!msg) return;
    if (app_->getPlatoonRole() != PlatoonRole::JOINER) return;
    if (joinManeuverState_ != JoinManeuverState::J_WAIT_INFORMATION) return;
    if (!targetPlatoonData_) return;

    if (msg->getPlatoonId() != targetPlatoonData_->platoonId) return;
    if (msg->getVehicleId() != targetPlatoonData_->platoonLeader) return;

    targetPlatoonData_->from(msg);

    const int currentLane = app_->getCurrentLaneIndex();
    if (currentLane != targetPlatoonData_->platoonLane) {
        controllerAdapter_->setFixedLane(targetPlatoonData_->platoonLane);
    }

    const double targetGap = computeTargetJoinGapMeters(targetPlatoonData_->platoonSpeed); // m; final follower gap.

    /*
     * Join-at-Back split:
     *
     *   - final target gap is controlled after JoinFormation
     *   - MoveToPositionAck is sent once distance < targetGap + 11 m
     *   - fake CACC approach spacing is an approach-control value, not the ACK threshold
     *
     * For CDS setup:
     *
     *   targetGap          = 5 m
     *   inPositionSlack    = 11 m
     *   ACK threshold      = 16 m
     *   approachSpacing    = 15 m
     *
     * This prevents the joiner from stabilizing outside the ACK boundary.
     */
    const double inPositionSlack =
        readMetersParam(app_, "in_pos_slack", kPlexeJoinInPositionSlackMeters_); // m.

    const double inPositionThreshold = targetGap + inPositionSlack; // m.

    const double approachSpacing =
        readMetersParam(app_, "approach_spacing", kPlexeApproachSpacingMeters_); // m; fake-CACC spacing during approach.

    const double approachDeltaV =
        readSpeedParam(app_, "approach_delta_v", kApproachDeltaV_); // m/s; speed added to platoon speed while closing in.

    positionHelper_->setController(ActiveController::FAKED_CACC);

    controllerAdapter_->setCACCConstantSpacing(approachSpacing);

    // Initial fake leader/front data is overwritten by real beacons in onPlatoonBeacon().
    controllerAdapter_->setLeaderVehicleFakeData(
        0.0,
        0.0,
        targetPlatoonData_->platoonSpeed);

    controllerAdapter_->setFrontVehicleFakeData(
        0.0,
        0.0,
        targetPlatoonData_->platoonSpeed,
        approachSpacing);

    controllerAdapter_->setCruiseControlDesiredSpeed(
        targetPlatoonData_->platoonSpeed + approachDeltaV);

    controllerAdapter_->setActiveController(ActiveController::FAKED_CACC);
    controllerAdapter_->setControlMode(ControlMode::JOINER_MOVE_IN_POSITION);

    joinManeuverState_ = JoinManeuverState::J_MOVE_IN_POSITION;

    EV_INFO << "[CarlaJoinAtBack][handleMoveToPosition]"
            << " actor=" << positionHelper_->getExternalId()
            << " platoonSpeed=" << targetPlatoonData_->platoonSpeed
            << " frontId=" << targetPlatoonData_->frontId()
            << " targetGap=" << targetGap
            << " inPositionSlack=" << inPositionSlack
            << " inPositionThreshold=" << inPositionThreshold
            << " approachSpacing=" << approachSpacing
            << " caccSpacing=" << controllerAdapter_->getCACCConstantSpacing()
            << " approachDeltaV=" << approachDeltaV
            << " state=J_MOVE_IN_POSITION"
            << "\n";
}

// Leader-side handler for MoveToPositionAck.
// Confirms the joiner is close enough, then sends JoinFormation to make the join official.
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

// Joiner-side handler for JoinFormation.
// Converts the joiner into a normal follower and sends JoinFormationAck to the leader.
void CarlaJoinAtBack::handleJoinFormation(const JoinFormation* msg)
{
    if (!msg) return;
    if (app_->getPlatoonRole() != PlatoonRole::JOINER) return;
    if (joinManeuverState_ != JoinManeuverState::J_WAIT_JOIN) return;
    if (!targetPlatoonData_) return;

    if (msg->getPlatoonId() != targetPlatoonData_->platoonId) return;
    if (msg->getVehicleId() != targetPlatoonData_->platoonLeader) return;
    if (!formationMatches(msg, targetPlatoonData_->newFormation)) return;

    const ActiveController targetController = app_->getTargetController();

    controllerAdapter_->setActiveController(targetController);
    positionHelper_->setController(targetController);
    positionHelper_->setDistance(app_->getStandstillDistance(targetController));
    positionHelper_->setHeadway(app_->getHeadway(targetController));

    // Final follower spacing is applied here, after JoinFormation.
    // This is separate from the approach spacing used during J_MOVE_IN_POSITION.
    if (positionHelper_->getController() == ActiveController::CACC ||
        positionHelper_->getController() == ActiveController::FAKED_CACC) {
        controllerAdapter_->setCACCConstantSpacing(
            app_->getTargetDistance(targetPlatoonData_->platoonSpeed));
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

    /*
     * Fake leader/front data exists only for the join approach phase.
     * Once the joiner becomes a real follower, it must use real predecessor
     * and leader beacons. Leaving fakeFront valid here causes stale distance
     * from the ACK moment to contaminate steady CACC.
     */
    controllerAdapter_->clearLeaderVehicleFakeData();
    controllerAdapter_->clearFrontVehicleFakeData();

    joinManeuverState_ = JoinManeuverState::IDLE;
    app_->setInManeuver(false, nullptr);

    EV_INFO << "[CarlaJoinAtBack][handleJoinFormation]"
            << " actor=" << positionHelper_->getExternalId()
            << " action=join_complete"
            << " role=FOLLOWER"
            << " finalController=" << toString(targetController)
            << " finalCaccSpacing=" << controllerAdapter_->getCACCConstantSpacing()
            << " fakeFrontCleared=1"
            << " fakeLeaderCleared=1"
            << "\n";

    targetPlatoonData_.reset();
    joinerData_.reset();
    joinReqRetries_ = 0;
}

// Leader-side handler for JoinFormationAck.
// Finalizes formation membership on the leader and broadcasts UpdatePlatoonFormation to followers.
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

    // Notify each non-leader member of the updated formation.
    // The updated formation changes each follower's predecessor selection.
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

    // Defensive cleanup on leader side too.
    controllerAdapter_->clearLeaderVehicleFakeData();
    controllerAdapter_->clearFrontVehicleFakeData();

    EV_INFO << "[CarlaJoinAtBack][handleJoinFormationAck]"
            << " actor=" << positionHelper_->getExternalId()
            << " joinerId=" << joinerData_->joinerId
            << " action=broadcast_UpdatePlatoonFormation"
            << "\n";

    targetPlatoonData_.reset();
    joinerData_.reset();
}

} // namespace carla
