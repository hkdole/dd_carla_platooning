#include "carla/apps/CarlaGeneralPlatooningApp.h"
#include "carla/platooning/state/ControlOutput.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <limits>

#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"
#include "veins/modules/utility/Consts80211p.h"

using namespace omnetpp;

namespace {

// Converts INET's coordinate type into Veins' coordinate type.
// CARLA mobility is exposed through INET, while the 802.11p stack uses Veins types.
static inline veins::Coord toVeinsCoord(const inet::Coord& c)
{
    return veins::Coord(c.x, c.y, c.z);
}

// Detects Veins/DemoBaseApplLayer beacon self-messages.
// OMNeT++ self-messages are timers; this app reuses the base beacon timer name.
static inline bool looksLikeBeaconSelfMsg(const omnetpp::cMessage* msg)
{
    if (!msg || !msg->isSelfMessage()) return false;
    const char* n = msg->getName();
    return n && std::string(n).find("beacon") != std::string::npos;
}

// Saturates a value into [lo, hi].
// Used for controller gains, acceleration bounds, and actuator commands.
template <typename T>
static inline T clampValue(T v, T lo, T hi)
{
    return std::max(lo, std::min(hi, v));
}

// Copies a generated OMNeT++ message formation array into a C++ vector.
// The message types are generated from .msg files and expose array-size accessors.
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

} // namespace

namespace carla {

Define_Module(CarlaGeneralPlatooningApp);

// OMNeT++ signals exported as result vectors and consumed by BridgeApp.
// BridgeApp listens to desired/control signals and forwards them to pyCARLANeT.
simsignal_t CarlaGeneralPlatooningApp::desiredAccelerationSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::desiredSpeedSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::controlModeSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::activeControllerSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::hasControlSignal_ = SIMSIGNAL_NULL;

// Final CARLA actuator commands after acceleration tracking.
// These are plant-interface signals, not V2V protocol messages.
simsignal_t CarlaGeneralPlatooningApp::controlThrottleSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::controlBrakeSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::controlHandBrakeSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::controlReverseSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::controlManualGearShiftSignal_ = SIMSIGNAL_NULL;

// Plexe-style metrics used by parser/R analysis scripts.
simsignal_t CarlaGeneralPlatooningApp::speedSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::accelerationSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::controllerAccelerationExportSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::distanceSignal_ = SIMSIGNAL_NULL;
simsignal_t CarlaGeneralPlatooningApp::relativeSpeedSignal_ = SIMSIGNAL_NULL;

// Constructor is intentionally empty; OMNeT++ modules are configured in initialize().
CarlaGeneralPlatooningApp::CarlaGeneralPlatooningApp() = default;

// Releases OMNeT++ self-message timers and owned maneuver objects.
// cancelAndDelete() is required for scheduled self-messages to avoid use-after-free.
CarlaGeneralPlatooningApp::~CarlaGeneralPlatooningApp()
{
    // FOR TESTING PURPOSES ONLY
    if (brakeTimer_) cancelAndDelete(brakeTimer_);
    if (controlTimer_) cancelAndDelete(controlTimer_);
    if (startTimer_) cancelAndDelete(startTimer_);
    if (heuristicTimer_) cancelAndDelete(heuristicTimer_);
    if (exitTimer_) cancelAndDelete(exitTimer_);
    delete joinManeuver_;
}

// Marks whether a maneuver object currently owns maneuver-specific events.
// activeManeuver_ receives self-messages and maneuver messages before default app handling.
void CarlaGeneralPlatooningApp::setInManeuver(bool b, CarlaManeuver* maneuver)
{
    inManeuver_ = b;
    activeManeuver_ = b ? maneuver : nullptr;
}

// Returns the minimum spacing used at zero speed.
// controller is ignored here because this platform currently uses one spacing policy.
double CarlaGeneralPlatooningApp::getStandstillDistance(ActiveController controller) const
{
    (void)controller;
    return gapMin_; // m; bumper-to-bumper constant-distance spacing target.
}

// Returns the time-headway parameter configured for the controller.
// Current CDS behavior does not use it in getTargetDistance(), but it is exported to inputs.
double CarlaGeneralPlatooningApp::getHeadway(ActiveController controller) const
{
    (void)controller;
    return headway_; // s; used by time-headway controllers or alternative policies.
}

// Computes the desired bumper gap to the front vehicle.
// In this CDS version, the target is constant and independent of ego speed.
double CarlaGeneralPlatooningApp::getTargetDistance(double speed) const
{
    // Constant time spacing
    return gapMin_ + headway_ * std::max(0.0, speed);
}

// Controller-aware overload for code that follows Plexe naming conventions.
// Current implementation maps all controllers to the same CDS target.
double CarlaGeneralPlatooningApp::getTargetDistance(ActiveController controller, double speed) const
{
    (void)controller;
    return getTargetDistance(speed);
}

// Returns the normal platoon-following controller type.
// Leaders use CC during initialization; followers/joined vehicles target CACC semantics.
ActiveController CarlaGeneralPlatooningApp::getTargetController() const
{
    return ActiveController::CACC;
}

// Reads the latest CARLA-derived position from the INET mobility module.
// CarlaInetMobility is updated by CarlanetManager after each pyCARLANeT step.
veins::Coord CarlaGeneralPlatooningApp::getCurrentPosition() const
{
    if (!mobility_) return veins::Coord(0, 0, 0);
    const auto p = mobility_->getCurrentPosition();
    return veins::Coord(p.x, p.y, p.z);
}

// Returns the lane index used in maneuver messages.
// CARLA lane control is not implemented here; this is metadata for platoon protocol state.
int CarlaGeneralPlatooningApp::getCurrentLaneIndex() const
{
    if (controllerAdapter_.hasFixedLane()) return controllerAdapter_.getFixedLane();
    if (positionHelper_.getPlatoonLane() >= 0) return positionHelper_.getPlatoonLane();
    return 0;
}

// Converts a 2D velocity vector to scalar speed.
// z is ignored because longitudinal platooning is evaluated in the road plane.
double CarlaGeneralPlatooningApp::scalarSpeedFromVelocity(const veins::Coord& v) const
{
    return std::sqrt(v.x * v.x + v.y * v.y);
}

// Parses a comma-separated vehicle-id list from omnetpp.ini.
// Example: "0,1,2,3" means leader first, then followers in order.
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

// Computes bumper gap from this vehicle to a neighbor.
// The neighbor state comes from the last received PlatooningBeacon.
double CarlaGeneralPlatooningApp::computeGapToNeighbor(const PlatoonNeighborState& n) const
{
    const auto myPos = getCurrentPosition();

    const double dx = myPos.x - n.pos.x;
    const double dy = myPos.y - n.pos.y;

    const double centerDistance = std::sqrt(dx * dx + dy * dy);
    const double frontLen = (n.length > 0.0) ? n.length : vehicleLength_;

    return centerDistance - frontLen; // m; approximate bumper-to-bumper distance.
}

// Converts gap error into a speed correction with saturation.
// Positive gap error means the vehicle is too far back and may close in.
double CarlaGeneralPlatooningApp::computeSpeedBiasFromGapError(double gapError, double gain, double limit) const
{
    return clampValue(gain * gapError, -limit, limit);
}

// Finds this vehicle's predecessor in a formation vector.
// Returns -1 for the leader, unknown vehicles, or invalid formations.
int CarlaGeneralPlatooningApp::findFrontVehicleIdInFormation(const std::vector<int>& formation) const
{
    const auto it = std::find(formation.begin(), formation.end(), nodeId_);
    if (it == formation.end()) return -1;
    if (it == formation.begin()) return -1;
    return *(it - 1);
}

// Clears actuator-tracking memory.
// Call this when semantic control mode changes so old throttle/brake state cannot leak.
void CarlaGeneralPlatooningApp::resetLongitudinalActuatorTracker()
{
    actuatorThrottleCmd_ = 0.0;
    actuatorBrakeCmd_ = 0.0;

    actuatorAccelIntegral_ = 0.0;
    actuatorAccelError_ = 0.0;
    actuatorEffort_ = 0.0;

    actuatorTargetThrottle_ = 0.0;
    actuatorTargetBrake_ = 0.0;

    actuatorTrackerInitialized_ = false;
    actuatorLastControlMode_ = ControlMode::HOLD;
    actuatorLastControlModeValid_ = false;

    lastActuatorUpdateTime_ = simTime();
}

// Converts an OMNeT++ acceleration command into CARLA throttle/brake commands.
// Controllers produce desiredAcceleration; CARLA does not accept acceleration directly.
// This function is the actuator/plant adapter used after computeControllerOutput().
void CarlaGeneralPlatooningApp::fillTrackedLongitudinalActuation(
    ControlOutput& out,
    double measuredAcceleration,
    double currentSpeed)
{
    /*
     * Closed-loop acceleration tracker.
     *
     * Plexe/SUMO controllers output acceleration. CARLA accepts throttle/brake.
     * This adapter tracks the controller acceleration while preserving the V2V and
     * CACC semantics computed in OMNeT++.
     *
     * Protocol and platoon-control logic should not be added here. This function
     * only maps desired longitudinal motion to CARLA's low-level actuator fields.
     */

    if (!out.hasControl) {
        out.throttle = 0.0;
        out.brake = 1.0;
        out.handBrake = true;
        out.reverse = false;
        out.manualGearShift = false;

        resetLongitudinalActuatorTracker();
        return;
    }

    // Reset actuator memory across semantic mode changes.
    // This prevents residual throttle/brake/integral from crossing maneuver phases.
    if (actuatorLastControlModeValid_ &&
        out.controlMode != actuatorLastControlMode_) {
        resetLongitudinalActuatorTracker();
    }

    out.handBrake = false;
    out.reverse = false;
    out.manualGearShift = false;

    double dt = controlInterval_.dbl(); // s; nominal control update period from omnetpp.ini.
    if (actuatorTrackerInitialized_ && simTime() > lastActuatorUpdateTime_) {
        dt = (simTime() - lastActuatorUpdateTime_).dbl();
    }
    dt = clampValue(dt, 1e-3, 0.20); // s; prevents unstable updates after pauses or startup.

    const double desiredAccel = clampValue(
        std::isfinite(out.desiredAcceleration) ? out.desiredAcceleration : 0.0,
        aMin_, // m/s^2; configured minimum acceleration, usually braking.
        aMax_  // m/s^2; configured maximum acceleration.
    );

    double actualAccel = std::isfinite(measuredAcceleration)
        ? measuredAcceleration
        : 0.0;

    // Dynamic CARLA spawn can report vertical falling/suspension acceleration.
    // Do not let that corrupt the longitudinal actuator tracker at startup.
    if (currentSpeed < 0.20 && desiredAccel > 0.0) {
        actualAccel = 0.0;
        actuatorAccelIntegral_ = 0.0;
    }

    double error = desiredAccel - actualAccel; // m/s^2; positive means CARLA is under-accelerating.

    constexpr double kAccelDeadband = 0.05; // m/s^2; ignores sensor/finite-difference noise.
    if (std::fabs(error) < kAccelDeadband) {
        error = 0.0;
    }

    // CARLA actuator tracking gains. These are not CACC gains.
    // Feed-forward estimates throttle/brake; PI correction removes plant mismatch.
    constexpr double kThrottleFF = 0.18;     // throttle per m/s^2 of desired acceleration.
    constexpr double kBrakeFF = 0.30;        // brake per m/s^2 of desired deceleration.
    constexpr double kAccelKp = 0.10;        // proportional correction on acceleration error.
    constexpr double kAccelKi = 0.06;        // integral correction on persistent error.
    constexpr double kIntegralLimit = 5.0;   // anti-windup bound for acceleration integral.

    if (std::fabs(desiredAccel) < 0.05 && currentSpeed < 0.20) {
        actuatorAccelIntegral_ = 0.0;
    }
    else {
        actuatorAccelIntegral_ += error * dt;
        actuatorAccelIntegral_ = clampValue(
            actuatorAccelIntegral_,
            -kIntegralLimit,
            kIntegralLimit);
    }

    const double ff = desiredAccel >= 0.0
        ? kThrottleFF * desiredAccel
        : kBrakeFF * desiredAccel; // Negative effort means brake.

    double effort = ff + kAccelKp * error + kAccelKi * actuatorAccelIntegral_;

    constexpr double kEffortDeadband = 0.015; // actuator effort below this is treated as zero.
    if (std::fabs(effort) < kEffortDeadband) {
        effort = 0.0;
    }

    effort = clampValue(effort, -1.0, 1.0); // CARLA throttle/brake fields are normalized [0, 1].

    double targetThrottle = 0.0;
    double targetBrake = 0.0;

    if (effort > 0.0) {
        targetThrottle = clampActuator(effort);
        targetBrake = 0.0;
    }
    else if (effort < 0.0) {
        targetThrottle = 0.0;
        targetBrake = clampActuator(-effort);
    }

    // Limits command slope to avoid throttle/brake chatter between simulation ticks.
    auto rateLimit = [](double previous,
                        double target,
                        double upRate,
                        double downRate,
                        double stepSeconds) {
        if (target > previous) {
            return std::min(target, previous + upRate * stepSeconds);
        }
        return std::max(target, previous - downRate * stepSeconds);
    };

    constexpr double kThrottleUpPerSecond = 2.0;    // normalized throttle units per second.
    constexpr double kThrottleDownPerSecond = 4.0;  // throttle release can be faster than application.
    constexpr double kBrakeUpPerSecond = 5.0;       // normalized brake units per second.
    constexpr double kBrakeDownPerSecond = 7.0;     // brake release can be faster than application.

    actuatorThrottleCmd_ = rateLimit(
        actuatorThrottleCmd_,
        targetThrottle,
        kThrottleUpPerSecond,
        kThrottleDownPerSecond,
        dt);

    actuatorBrakeCmd_ = rateLimit(
        actuatorBrakeCmd_,
        targetBrake,
        kBrakeUpPerSecond,
        kBrakeDownPerSecond,
        dt);

    // Avoid simultaneous meaningful throttle and brake.
    // CARLA drivetrain behavior can oscillate near the zero-acceleration boundary otherwise.
    if (actuatorThrottleCmd_ > 0.02) {
        actuatorBrakeCmd_ = 0.0;
    }
    if (actuatorBrakeCmd_ > 0.02) {
        actuatorThrottleCmd_ = 0.0;
    }

    out.throttle = clampActuator(actuatorThrottleCmd_);
    out.brake = clampActuator(actuatorBrakeCmd_);

    actuatorAccelError_ = error;
    actuatorEffort_ = effort;
    actuatorTargetThrottle_ = targetThrottle;
    actuatorTargetBrake_ = targetBrake;

    actuatorTrackerInitialized_ = true;
    lastActuatorUpdateTime_ = simTime();

    actuatorLastControlMode_ = out.controlMode;
    actuatorLastControlModeValid_ = true;
}

// OMNeT++ module initialization.
// stage 0 reads NED/ini parameters, registers output signals, creates maneuver objects,
// initializes role-specific controller state, and schedules periodic control/maneuver timers.
void CarlaGeneralPlatooningApp::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);

    if (stage != 0) return;

    // Select controller type
    if (hasPar("controller_type")) 
        controllerType_ = par("controller_type").stdstringValue();

    // Apply brake timer if configured
    brakeTimer_ = new cMessage("brakeTimer");
    if (hasPar("brake_at")) {
        double brakeAt = par("brake_at").doubleValue();
        if (brakeAt > 0) {
            scheduleAt(SimTime(brakeAt), brakeTimer_);
        }
    }

    // Apply exit if enabled
    exitManeuver_ = new CarlaExitManeuver(this);
    exitEnabled_ = hasPar("exit_enabled") && par("exit_enabled").boolValue();
    if (exitEnabled_ && hasPar("exit_at")) {
        const double exitAt = par("exit_at").doubleValueInUnit("s");
        if (exitAt > 0) {
            exitTimer_ = new cMessage("exitTimer");
            scheduleAt(SimTime(exitAt), exitTimer_);
        }
    }

    // Apply sinusoidal movement if configured
    if (hasPar("sinusoidal_speed"))
        sinusoidalSpeed_ = par("sinusoidal_speed").boolValue();
    if (hasPar("sinusoidal_amplitude")) 
        sinusoidalAmplitude_ = par("sinusoidal_amplitude").doubleValueInUnit("mps");
    if (hasPar("sinusoidal_period")) 
        sinusoidalPeriod_ = par("sinusoidal_period").doubleValueInUnit("s");
    if (hasPar("sinusoidal_start_at")) 
        sinusoidalStartAt_ = par("sinusoidal_start_at").doubleValueInUnit("s");

    // Signals consumed by BridgeApp or written to result vectors.
    if (desiredAccelerationSignal_ == SIMSIGNAL_NULL)
        desiredAccelerationSignal_ = registerSignal("desired_acceleration");
    if (desiredSpeedSignal_ == SIMSIGNAL_NULL)
        desiredSpeedSignal_ = registerSignal("output_desired_speed");
    if (controlModeSignal_ == SIMSIGNAL_NULL)
        controlModeSignal_ = registerSignal("control_mode");
    if (hasControlSignal_ == SIMSIGNAL_NULL)
        hasControlSignal_ = registerSignal("has_control");
    if (activeControllerSignal_ == SIMSIGNAL_NULL)
        activeControllerSignal_ = registerSignal("active_controller");

    // Final OMNeT++ longitudinal actuator command for pyCARLANeT.
    if (controlThrottleSignal_ == SIMSIGNAL_NULL)
        controlThrottleSignal_ = registerSignal("control_throttle");
    if (controlBrakeSignal_ == SIMSIGNAL_NULL)
        controlBrakeSignal_ = registerSignal("control_brake");
    if (controlHandBrakeSignal_ == SIMSIGNAL_NULL)
        controlHandBrakeSignal_ = registerSignal("control_hand_brake");
    if (controlReverseSignal_ == SIMSIGNAL_NULL)
        controlReverseSignal_ = registerSignal("control_reverse");
    if (controlManualGearShiftSignal_ == SIMSIGNAL_NULL)
        controlManualGearShiftSignal_ = registerSignal("control_manual_gear_shift");

    // PLEXE-style exported result vectors used by analysis/map-config and parser scripts.
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

    // Steering Signal registration
    if (controlSteerSignal_ == SIMSIGNAL_NULL)
        controlSteerSignal_ = registerSignal("control_steer");
    if (lateralControlActiveSignal_ == SIMSIGNAL_NULL)
        lateralControlActiveSignal_ = registerSignal("lateral_control_active");

    // Mobility module updated by CarlanetManager from pyCARLANeT position snapshots.
    mobility_ = check_and_cast<CarlaInetMobility*>(getParentModule()->getSubmodule("mobility"));

    actorId_ = par("actor_id").stdstringValue();       // CARLA actor id/name used by the Python bridge.
    nodeId_ = par("node_id").intValue();               // OMNeT++/platoon logical vehicle id.
    platoonId_ = par("platoon_id").intValue();         // Logical platoon identifier in maneuver messages.
    leaderId_ = par("leader_id").intValue();           // Logical id of the current platoon leader.

    nominalPlatoonSpeed_ = par("nominal_platoon_speed").doubleValue(); // m/s; steady platoon cruise speed.
    leaderTargetSpeed_ = par("leader_target_speed").doubleValue();     // m/s; leader CC speed target.

    vehicleLength_ = par("vehicle_length").doubleValue();              // m; used to convert center distance to bumper gap.

    headway_ = par("headway").doubleValue();       // s; time-gap parameter for CTS-style controllers.
    gapMin_ = par("gap_min").doubleValue();        // m; constant bumper gap for CDS spacing.
    kGap_ = par("k_gap").doubleValue();            // CACC gain on spacing error.
    kDv_ = par("k_dv").doubleValue();              // CACC gain on relative speed error.
    kSpeedP_ = par("k_speed_p").doubleValue();     // CC gain on speed error.
    aMin_ = par("a_min").doubleValue();            // m/s^2; lower acceleration bound, usually negative.
    aMax_ = par("a_max").doubleValue();            // m/s^2; upper acceleration bound.

    multilaneJoinEnabled_ = par("multilane_join_enabled").boolValue();
    laneChangePenalty_ = par("lane_change_penalty").doubleValue();
    laneWidthM_ = par("lane_width_m").doubleValueInUnit("m");

    if (hasPar("alpha")) alpha_ = par("alpha").doubleValue();       // Alpha value for decentralized, beta = 1 - alpha
    if (hasPar("deviation")) p_ = par("deviation").doubleValue();   // max speed deviation fraction
    if (hasPar("range")) r_ = par("range").doubleValue();           // max position range in meters
    platoonDesiredSpeed_ = par("desired_speed").doubleValue();

    if (hasPar("k_acc_ff")) kAccFF_ = par("k_acc_ff").doubleValue();                 // Feed-forward gain on front acceleration.
    if (hasPar("k_leader_dv")) kLeaderDv_ = par("k_leader_dv").doubleValue();         // Gain on leader-relative speed.
    if (hasPar("k_gap_speed")) kGapSpeed_ = par("k_gap_speed").doubleValue();         // Converts gap error to debug target-speed bias.
    if (hasPar("max_closure_speed")) maxClosureSpeed_ = par("max_closure_speed").doubleValue(); // m/s; caps gap-closing speed bias.

    // Carried into the parsed comparison schema, matching PLEXE labels.
    if (hasPar("caccXi")) caccXi_ = par("caccXi").doubleValue();             // Damping ratio for Plexe/SUMO CACC model.
    if (hasPar("caccOmegaN")) caccOmegaN_ = par("caccOmegaN").doubleValue(); // Natural frequency for Plexe/SUMO CACC model.

    beaconInterval_ = par("beaconInterval");       // s; V2V beacon broadcast period.
    controlInterval_ = par("control_interval");    // s; controller update period.
    maxAge_ = par("max_age");                      // s; maximum accepted age for neighbor beacon data.

    if (hasPar("start_maneuver_at")) startManeuverAt_ = par("start_maneuver_at"); // simtime; joiner starts join protocol at this time.
    if (hasPar("join_position")) joinPosition_ = par("join_position").intValue(); // index/id semantics used by join maneuver.
    if (hasPar("approach_delta_v")) approachDeltaV_ = par("approach_delta_v").doubleValue(); // m/s; joiner speed offset while approaching.
    if (hasPar("in_pos_slack")) inPosSlack_ = par("in_pos_slack").doubleValue();   // m; tolerance for declaring joiner in position.

    // Steering control
    if (hasPar("lateral_control_enabled"))
        lateralControlEnabled_ = par("lateral_control_enabled").boolValue();

    // PositionHelper stores Plexe-like platoon metadata: id, leader, formation, lane, speed, spacing.
    positionHelper_.setId(nodeId_);
    positionHelper_.setExternalId(actorId_);
    positionHelper_.setPlatoonId(platoonId_);
    positionHelper_.setLeaderId(leaderId_);
    positionHelper_.setPlatoonSpeed(nominalPlatoonSpeed_);
    positionHelper_.setController(ActiveController::CACC);
    positionHelper_.setDistance(gapMin_);
    positionHelper_.setHeadway(headway_);

    initialFrontId_ = -1;
    initialBackId_ = -1;
    if (hasPar("initial_formation")) {
        const auto formation = parseFormation(par("initial_formation").stdstringValue());
        // formation[0] = front neighbor (or self if no one ahead)
        // formation[1] = self
        // formation[2] = back neighbor (optional)
        if (!formation.empty()) {
            auto it = std::find(formation.begin(), formation.end(), nodeId_);
            if (it != formation.end()) {
                if (it != formation.begin())
                    initialFrontId_ = *(it - 1);
                if (std::next(it) != formation.end())
                    initialBackId_ = *(std::next(it));
        }
    }
        if (!formation.empty()) positionHelper_.setPlatoonFormation(formation);
    }

    // Distributed should be role agnostic and therefore everyone is a joiner. This is for the purpose of setting the state as opposed to role
    const std::string roleStr = par("platoon_role").stdstringValue();
    if (roleStr == "joiner") {
        role_ = PlatoonRole::JOINER;
    } else {
        role_ = PlatoonRole::LEADER;
    }

    // Set controller based on whether or not we have someone in front of us
    if (initialFrontId_ >= 0) {
        // Someone ahead — use CACC to follow them
        positionHelper_.setController(ActiveController::CACC);
        positionHelper_.setLeaderId(initialFrontId_);
        controllerAdapter_.setActiveController(ActiveController::CACC);
        controllerAdapter_.setControlMode(ControlMode::FOLLOWER_PLATOON);
        controllerAdapter_.setCACCConstantSpacing(gapMin_);
    } else {
        // No one ahead — free cruise
        positionHelper_.setController(ActiveController::CC);
        controllerAdapter_.setActiveController(ActiveController::CC);
        controllerAdapter_.setControlMode(ControlMode::LEADER_CRUISE);
    }

    // Initialize timer for joiners to collect neighbor
    heuristicTimer_ = new cMessage("heuristicTimer");

    // Delete later
    if (hasPar("desired_speed")) platoonDesiredSpeed_ = par("desired_speed").doubleValue();
    EV_INFO << "[DEBUG] actor=" << actorId_ << " platoonDesiredSpeed_=" << platoonDesiredSpeed_ << "\n";   

    if (hasPar("desired_speed")) platoonDesiredSpeed_ = par("desired_speed").doubleValue();

    // Join-at-back maneuver state machine. See CarlaJoinAtBack.cc for message sequence handling.
    joinManeuver_ = new CarlaJoinAtBack(this);

    // Self-messages are OMNeT++ timers. They are rescheduled in handleSelfMsg().
    controlTimer_ = new cMessage("platooningControlTimer");
    startTimer_ = new cMessage("joinStartTimer");

    if (hasPar("start_maneuver_at")) {
        const double startAt = par("start_maneuver_at").doubleValueInUnit("s");
        if (startAt > 0) {
            scheduleAt(SimTime(startAt), startTimer_);
        }
    }

    // Initialize export state from current mobility state.
    {
        veins::Coord vel(0, 0, 0);
        if (mobility_) vel = toVeinsCoord(mobility_->getCurrentVelocity());
        lastExportSpeed_ = scalarSpeedFromVelocity(vel);
        lastExportTime_ = simTime();
        actualAcceleration_ = 0.0;
    }

    resetLongitudinalActuatorTracker();

    scheduleAt(simTime() + controlInterval_, controlTimer_);

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

// Dispatches OMNeT++ self-message timers.
// Control ticks, beacon broadcasts, and maneuver-start events all enter here.
void CarlaGeneralPlatooningApp::handleSelfMsg(cMessage* msg)
{
    if (activeManeuver_ && activeManeuver_->handleSelfMsg(msg)) return;

    if (msg == startTimer_) {
        startJoinManeuverIfConfigured();
        return;
    }

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

    if (msg == heuristicTimer_) { // Once we feel like we got all the neighbor beacons
        if (role_ == PlatoonRole::JOINER &&
            !inManeuver_ &&
            controllerAdapter_.getControlMode() != ControlMode::FOLLOWER_PLATOON) {
            auto candidate = evaluatePlatoonCandidates();
            const int egoLaneId = mobility_ ? mobility_->getCarlaLaneId() : 0;
            if (candidate.valid) {
                if (candidate.laneDistance > 0) {
                    // Cross-lane candidate: change lanes first, then join.
                    // laneChangeManeuver_ will hand off to CarlaJoinAtBack via
                    // LANE_CHANGE_COMPLETE once the vehicle reaches the target lane.
                    const JoinSlot slot{candidate.vehicleId, -1};
                    laneChangeManeuver_.start(
                        egoLaneId,             // mobility_->getCarlaLaneId() - compute before this block
                        candidate.laneId,
                        laneWidthM_,           // from NED param, read in initialize()
                        slot);
                    setInManeuver(true, nullptr);

                    EV_INFO << "[CarlaGeneralPlatooningApp][heuristicEvaluation]"
                        << " actor=" << actorId_
                        << " candidateId=" << candidate.vehicleId
                        << " cost=" << candidate.cost
                        << " laneDistance=" << candidate.laneDistance
                        << " action=lane_change_first"
                        << "\n";
                } else {
                    // Same-lane candidate: existing path, completely unchanged.
                    JoinManeuverParameters params;
                    params.platoonId = candidate.vehicleId;
                    params.leaderId = candidate.vehicleId;
                    params.position = -1;

                    EV_INFO << "[CarlaGeneralPlatooningApp][heuristicEvaluation]"
                        << " actor=" << actorId_
                        << " candidateId=" << candidate.vehicleId
                        << " cost=" << candidate.cost
                        << " action=join_direct"
                        << "\n";

                    joinManeuver_->startManeuver(&params);
                }
            }
        }
        // Reschedule regardless — keep evaluating until joined
        if (!inManeuver_ && 
            controllerAdapter_.getControlMode() != ControlMode::FOLLOWER_PLATOON) {
            scheduleAt(simTime() + 1.0, heuristicTimer_);
        }
        return;
    }

    // For testing timers
    if (msg == brakeTimer_) {
        leaderTargetSpeed_ = 0.0;
        EV_INFO << "[CarlaGeneralPlatooningApp][brakeTest]"
                << " actor=" << actorId_
                << " action=full_stop"
                << " at t=" << simTime()
                << "\n";
        return;
    }

    if (msg == exitTimer_) {
        setInManeuver(true, exitManeuver_);
        exitManeuver_->startManeuver(nullptr);
        return;
    }
    // end

    DemoBaseApplLayer::handleSelfMsg(msg);
}

// Starts the join-at-back maneuver for vehicles configured as leaders (because you shouldn't have a car in front of you when you join)
// Called by startTimer_; actual message sequence is implemented in CarlaJoinAtBack.cc.
void CarlaGeneralPlatooningApp::startJoinManeuverIfConfigured()
{
    if (!joinManeuver_) return;
    if (role_ == PlatoonRole::JOINER) {
        // For joiners, trigger heuristic evaluation now
        if (!inManeuver_ && heuristicTimer_ && !heuristicTimer_->isScheduled()) {
            scheduleAt(simTime() + 1.0, heuristicTimer_);
        }
        return;
    }

    if (role_ != PlatoonRole::LEADER) return;

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

// Sends a logical unicast maneuver packet over a broadcast 802.11p channel.
// The physical frame broadcasts; destinationId is checked by receivers in handleLowerMsg().
void CarlaGeneralPlatooningApp::sendUnicast(cPacket* msg, int destination)
{
    take(msg); // Transfer ownership to this OMNeT++ module before encapsulation.

    auto* mm = dynamic_cast<ManeuverMessage*>(msg);
    if (mm) {
        mm->setDestinationId(destination);
    }

    auto* frame = new veins::BaseFrame1609_4(msg->getName(), msg->getKind());
    frame->setRecipientAddress(-1); // -1 is broadcast at the Veins MAC layer.
    frame->setChannelNumber(static_cast<int>(veins::Channel::cch)); // CCH = control channel.
    frame->encapsulate(msg);
    sendDown(frame);

    EV_INFO << "[CarlaGeneralPlatooningApp][sendUnicast]"
            << " actor=" << actorId_
            << " logical_dst=" << destination
            << " packet=" << msg->getName()
            << " mode=broadcast_with_payload_filter"
            << "\n";
}

// Fills common fields shared by all maneuver messages.
// Specific message constructors add maneuver-specific fields after this call.
void CarlaGeneralPlatooningApp::fillManeuverMessage(
    ManeuverMessage* msg,
    int vehicleId,
    const std::string& externalId,
    int platoonId,
    int destinationId)
{
    msg->setKind(0);
    msg->setVehicleId(vehicleId);          // Logical sender vehicle id.
    msg->setExternalId(externalId.c_str()); // CARLA actor id/name for tracing.
    msg->setPlatoonId(platoonId);          // Sender's current platoon id.
    msg->setDestinationId(destinationId);  // Logical receiver id; physical transmission is broadcast.
}

// Creates a formation-update message.
// Used after a maneuver changes the ordering/membership of the platoon.
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
    msg->setPlatoonSpeed(platoonSpeed); // m/s; common platoon speed metadata.
    msg->setPlatoonLane(platoonLane);   // lane index metadata; CARLA lane control is elsewhere.
    msg->setPlatoonFormationArraySize(platoonFormation.size());
    for (unsigned int i = 0; i < platoonFormation.size(); ++i) {
        msg->setPlatoonFormation(i, platoonFormation[i]);
    }
    return msg;
}

// Creates a platoon-data update message.
// Similar to formation update, but also carries a new platoon id.
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

// Updates local neighbor cache from a received V2V beacon.
// Controller inputs are built from this cache; stale entries are rejected by maxAge_.
void CarlaGeneralPlatooningApp::refreshNeighborFromBeacon(const PlatooningBeacon* pb)
{
    if (!pb) return;

    const int srcId = pb->getVehicleId();
    if (srcId < 0) return;
    if (srcId == nodeId_) return; // Ignore self beacons if looped back by the channel.

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
    n.desiredSpeed =  pb->getDesiredSpeed();
    n.laneId = pb->getLaneId();

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

// Retrieves cached neighbor state and reports its age.
// The caller decides whether the age is acceptable for control or maneuver logic.
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

// Handles packets received from the Veins lower layer.
// Decapsulates 802.11p frames, filters logical destinations, updates beacon state,
// and forwards maneuver messages to the active maneuver object.
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

// Gives maneuver logic a chance to react to each received platooning beacon.
void CarlaGeneralPlatooningApp::onPlatoonBeacon(const PlatooningBeacon* pb)
{
    if (joinManeuver_) joinManeuver_->onPlatoonBeacon(pb);

    if (role_ == PlatoonRole::JOINER && 
        !inManeuver_ &&
        heuristicTimer_ && 
        !heuristicTimer_->isScheduled()) {
        const bool waitingForStart = startTimer_ && startTimer_->isScheduled();
        if (!waitingForStart) {
            scheduleAt(simTime() + 1.0, heuristicTimer_);
        }
    }
}

// Iterate through neighbors to find optimal candidates
PlatoonCandidate CarlaGeneralPlatooningApp::evaluatePlatoonCandidates() const {
    const veins::Coord pos = getCurrentPosition();

    veins::Coord vel(0, 0, 0);
    if (mobility_) vel = toVeinsCoord(mobility_->getCurrentVelocity());
    const double speed = scalarSpeedFromVelocity(vel);

    const int egoLaneId = mobility_ ? mobility_->getCarlaLaneId() : 0;

    PlatoonCandidate best;
    for (const auto& [srcId, n] : neighborsByVehicleId_) {
        if (!n.valid) continue;
        if (srcId == nodeId_) continue;

        const double dx = n.pos.x - pos.x;
        const double dy = n.pos.y - pos.y;

        // Skip candidates behind us
        if (speed > 0.5) {
            const double dotProduct = (dx * vel.x + dy * vel.y) / speed;
            if (dotProduct < 0) continue;
        }

        // Exclude opposite-direction lanes by sign convention.
        if (egoLaneId != 0 && n.laneId != 0) {
            const bool sameDirection =
                (egoLaneId > 0) == (n.laneId > 0);
            if (!sameDirection) continue;
        }

        const double ds = std::fabs(platoonDesiredSpeed_ - n.desiredSpeed);
        const double dp = pos.distance(n.pos);

        if (ds > p_ * platoonDesiredSpeed_) continue;
        if (dp > r_) continue;

        const int laneDelta = std::abs(n.laneId - egoLaneId);
        const bool sameLane = (laneDelta == 0);

        double cost = alpha_ * ds + (1.0 - alpha_) * dp;

        // Scale penalty by lane distance so a 2-lane jump always costs
        // more than a 1-lane jump, and a same-lane candidate never loses
        // on cost alone to any cross-lane candidate.
        if (!sameLane) {
            if (!multilaneJoinEnabled_) continue;
            cost += laneChangePenalty_ * static_cast<double>(laneDelta);
        }

        if (cost < best.cost) {
            best.cost = cost;
            best.vehicleId = srcId;
            best.laneId = n.laneId;
            best.laneDistance = laneDelta;
            best.valid = true;
        }
    }

    // Fallback to known front neighbor if heuristic finds nothing
    if (!best.valid && initialFrontId_ >= 0) {
        best.vehicleId = initialFrontId_;
        best.laneDistance = 0; // assume same lane for configured front
        best.valid = true;
    }

    return best;
}

// Handles maneuver messages and updates app-level state needed by controllers.
// The maneuver object owns the protocol state machine; this function extracts formation/front-vehicle
// information needed to build joiner controller inputs during MoveToPosition/JoinFormation phases.
void CarlaGeneralPlatooningApp::onManeuverMessage(const ManeuverMessage* mm)
{
    if (!mm) return;

    if (auto* mtp = dynamic_cast<const MoveToPosition*>(mm)) {
        if (mtp->getDestinationId() == nodeId_) { // Check if it's our own message or we already have our rear occupied
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
        if (jf->getDestinationId() == nodeId_) {
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

    const std::string className = mm->getClassName();
    if (className.find("ExitRequest") != std::string::npos ||
        className.find("ExitAck") != std::string::npos) {
        exitManeuver_->onManeuverMessage(mm);
        return;
    }

    if (activeManeuver_) activeManeuver_->onManeuverMessage(mm);
    else if (joinManeuver_) joinManeuver_->onManeuverMessage(mm);

    // Once a vehicle is a normal follower, predecessor/leader come from PositionHelper formation.
    if (role_ == PlatoonRole::FOLLOWER ||
        controllerAdapter_.getControlMode() == ControlMode::FOLLOWER_PLATOON) {
        joinFrontVehicleId_ = -1;
        platoonId_ = positionHelper_.getPlatoonId();
        leaderId_ = positionHelper_.getLeaderId();
    }
}

// Applies a leader-issued formation update to a follower.
// This is centralized membership synchronization in the baseline join-at-back protocol.
void CarlaGeneralPlatooningApp::handleUpdatePlatoonFormation(const UpdatePlatoonFormation* msg)
{
    //if (msg->getPlatoonId() != positionHelper_.getPlatoonId()) return;
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

// Applies leader-issued platoon metadata changes to a follower.
// This extends formation updates with a possible platoon-id change.
void CarlaGeneralPlatooningApp::handleUpdatePlatoonData(const UpdatePlatoonData* msg)
{
    //if (msg->getPlatoonId() != positionHelper_.getPlatoonId()) return;
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

// Broadcasts this vehicle's current state for V2V control and maneuver logic.
// Receivers store the data in neighborsByVehicleId_ and use it in buildControllerInputs().
void CarlaGeneralPlatooningApp::sendPlatooningBeacon()
{
    const auto pos = getCurrentPosition();

    veins::Coord vel(0, 0, 0);
    if (mobility_) vel = toVeinsCoord(mobility_->getCurrentVelocity());

    const double speed = scalarSpeedFromVelocity(vel);

    auto* pb = new PlatooningBeacon("PlatooningBeacon");
    pb->setVehicleId(positionHelper_.getId());

    // Match PLEXE semantics:
    // acceleration = measured/actual longitudinal acceleration.
    // controllerAcceleration = controller command emitted by OMNeT++.
    pb->setAcceleration(actualAcceleration_);
    pb->setControllerAcceleration(desiredAcceleration_);

    pb->setSpeed(speed);                  // m/s; scalar speed.
    pb->setPositionX(pos.x);              // m; INET/CARLA transformed x position.
    pb->setPositionY(pos.y);              // m; INET/CARLA transformed y position.
    pb->setTime(simTime().dbl());         // s; sender simulation time.
    pb->setSequenceNumber(++beaconSequence_);
    pb->setLength(vehicleLength_);         // m; vehicle length for bumper-gap computation.
    pb->setSpeedX(vel.x);                 // m/s; x velocity component.
    pb->setSpeedY(vel.y);                 // m/s; y velocity component.
    pb->setAngle(0.0);                    // rad; placeholder heading field.
    pb->setDesiredSpeed(platoonDesiredSpeed_);   // As per the paper, we broadcast our desired speed
    pb->setLaneId(mobility_ ? mobility_->getCarlaLaneId() : 0); // Lane ID

    auto* frame = new veins::BaseFrame1609_4("PlatooningBeaconFrame", 0);
    frame->setRecipientAddress(-1);       // broadcast.
    frame->setChannelNumber(static_cast<int>(veins::Channel::cch));
    frame->encapsulate(pb);

    EV_INFO << "[CarlaGeneralPlatooningApp][sendPlatooningBeacon]"
            << " actor=" << actorId_
            << " nodeId=" << positionHelper_.getId()
            << " seq=" << beaconSequence_
            << " speed=" << speed
            << " desiredSpeed=" << platoonDesiredSpeed_
            << " laneId=" << (mobility_ ? mobility_->getCarlaLaneId() : 0)
            << " actualAcceleration=" << actualAcceleration_
            << " controllerAcceleration=" << desiredAcceleration_
            << " pos=(" << pos.x << "," << pos.y << ")"
            << "\n";

    sendDown(frame);
}

// Builds the complete input structure for the controller dispatcher.
// This is the main bridge from V2V protocol state to longitudinal control:
// formation -> predecessor/leader ids -> fresh beacon data -> ControllerInputs.
ControllerInputs CarlaGeneralPlatooningApp::buildControllerInputs() const
{
    ControllerInputs in;
    in.controlMode = controllerAdapter_.getControlMode();
    in.activeController = controllerAdapter_.getActiveController();

    // Force semantic controller identity from mode.
    // This avoids stale adapter state, especially for joiners that start as CC and later enter FAKED_CACC.
    switch (in.controlMode) {
        case ControlMode::LEADER_CRUISE:
            in.activeController = ActiveController::CC;
            break;
        case ControlMode::JOINER_FREE_CRUISE:
        case ControlMode::JOINER_WAIT_REPLY:
        case ControlMode::JOINER_WAIT_INFORMATION:
            in.activeController = ActiveController::CC;
            break;

        case ControlMode::FOLLOWER_PLATOON:
            in.activeController = (controllerType_ == "ACC")
                ? ActiveController::ACC
                : ActiveController::CACC;
            break;

        case ControlMode::JOINER_MOVE_IN_POSITION:
        case ControlMode::JOINER_WAIT_JOIN:
            in.activeController = ActiveController::FAKED_CACC;
            break;

        default:
            break;
    }

    veins::Coord vel(0, 0, 0);
    if (mobility_) vel = toVeinsCoord(mobility_->getCurrentVelocity());
    const double mySpeed = scalarSpeedFromVelocity(vel);

    in.egoSpeed = mySpeed;                       // m/s; current speed from CARLA mobility.
    in.targetSpeed = leaderTargetSpeed_;         // m/s; default target before mode-specific override.
    in.standstillDistance = gapMin_;             // m; minimum desired bumper gap.
    in.headway = headway_;                       // s; time-headway parameter.
    in.targetGap = getTargetDistance(mySpeed);   // m; desired bumper gap for current spacing policy.
    in.approachDeltaV = approachDeltaV_;         // m/s; joiner approach speed offset.
    in.maxClosureSpeed = maxClosureSpeed_;       // m/s; cap on gap-closing speed bias.

    in.kGap = kGap_;         // spacing-error gain.
    in.kDv = kDv_;           // relative-speed gain.
    in.kSpeedP = kSpeedP_;   // cruise-control speed gain.
    in.kAccFF = kAccFF_;     // front-vehicle acceleration feed-forward gain.
    in.kLeaderDv = kLeaderDv_;   // leader-relative speed gain.
    in.kGapSpeed = kGapSpeed_;   // gap-error to target-speed bias gain.

    in.aMin = aMin_; // m/s^2; output lower bound.
    in.aMax = aMax_; // m/s^2; output upper bound.

    // Plexe/SUMO MSCFModel_CC CACC parameters.
    // caccC1 default in SUMO/Plexe is 0.5.
    in.caccC1 = 0.5;
    in.caccXi = caccXi_;
    in.caccOmegaN = caccOmegaN_;

    // Plexe/SUMO CACC feed-forward semantics.
    // true  = use predecessor/leader controllerAcceleration from beacons.
    // false = use measured actualAcceleration from CARLA state instead.
    in.useControllerAcceleration = true;

    // CARLA V2V alternative:
    // Disable controller feed-forward if the experiment should rely only on
    // measured acceleration from received vehicle state.
    // in.useControllerAcceleration = false;

    const int myId = positionHelper_.getId();
    const int predId = positionHelper_.getPredecessorId(); // Vehicle directly ahead in current formation.
    const int leaderId = positionHelper_.getLeaderId();    // Platoon leader id.

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
        in.predecessor.controllerAcceleration = std::isfinite(pred.controllerAcceleration)
            ? pred.controllerAcceleration
            : pred.actualAcceleration;
        in.predecessor.distance = computeGapToNeighbor(pred);
    }

    PlatoonNeighborState leader;
    simtime_t leaderAge = SIMTIME_ZERO;

    const bool predIsLeader = (predId == leaderId);

    const bool leaderFresh =
        predIsLeader
            ? predFresh
            : ((leaderId >= 0) &&
               (leaderId != myId) &&
               getNeighbor(leaderId, leader, leaderAge) &&
               (leaderAge <= maxAge_));

    if (leaderFresh) {
        const PlatoonNeighborState& leaderRef = predIsLeader ? pred : leader;

        in.leader.valid = true;
        in.leader.speed = leaderRef.scalarSpeed;
        in.leader.actualAcceleration = leaderRef.actualAcceleration;
        in.leader.controllerAcceleration = std::isfinite(leaderRef.controllerAcceleration)
            ? leaderRef.controllerAcceleration
            : leaderRef.actualAcceleration;
        in.leader.distance = computeGapToNeighbor(leaderRef);
    }

    const bool allowFakeVehicleInputs =
        in.controlMode == ControlMode::JOINER_MOVE_IN_POSITION ||
        in.controlMode == ControlMode::JOINER_WAIT_JOIN;

    if (allowFakeVehicleInputs) {
        // Joiner fake leader/front inputs emulate Plexe join behavior while the vehicle is not yet a normal follower.
        const auto& fakeLeader = controllerAdapter_.getLeaderVehicleFakeData();
        if (fakeLeader.valid) {
            in.fakeLeader.valid = true;
            in.fakeLeader.speed = fakeLeader.speed;
            in.fakeLeader.actualAcceleration = fakeLeader.actualAcceleration;
            in.fakeLeader.controllerAcceleration = std::isfinite(fakeLeader.controllerAcceleration)
                ? fakeLeader.controllerAcceleration
                : fakeLeader.actualAcceleration;
        }
        else if (leaderFresh) {
            const PlatoonNeighborState& leaderRef = predIsLeader ? pred : leader;

            in.fakeLeader.valid = true;
            in.fakeLeader.speed = leaderRef.scalarSpeed;
            in.fakeLeader.actualAcceleration = leaderRef.actualAcceleration;
            in.fakeLeader.controllerAcceleration = std::isfinite(leaderRef.controllerAcceleration)
                ? leaderRef.controllerAcceleration
                : leaderRef.actualAcceleration;
            in.fakeLeader.distance = computeGapToNeighbor(leaderRef);
        }

        const auto& fakeFront = controllerAdapter_.getFrontVehicleFakeData();
        if (fakeFront.valid) {
            in.fakeFront.valid = true;
            in.fakeFront.speed = fakeFront.speed;
            in.fakeFront.actualAcceleration = fakeFront.actualAcceleration;
            in.fakeFront.controllerAcceleration = std::isfinite(fakeFront.controllerAcceleration)
                ? fakeFront.controllerAcceleration
                : fakeFront.actualAcceleration;
            in.fakeFront.distance = fakeFront.distance;
        }
        else if (joinFrontVehicleId_ >= 0) {
            PlatoonNeighborState front;
            simtime_t frontAge = SIMTIME_ZERO;
            if (getNeighbor(joinFrontVehicleId_, front, frontAge) && frontAge <= maxAge_) {
                in.fakeFront.valid = true;
                in.fakeFront.speed = front.scalarSpeed;
                in.fakeFront.actualAcceleration = front.actualAcceleration;
                in.fakeFront.controllerAcceleration = std::isfinite(front.controllerAcceleration)
                    ? front.controllerAcceleration
                    : front.actualAcceleration;
                in.fakeFront.distance = computeGapToNeighbor(front);
            }
        }
    }

    // Only apply unified gap control when not in an active maneuver phase
    const bool inManeuverPhase = 
        in.controlMode == ControlMode::JOINER_MOVE_IN_POSITION ||
        in.controlMode == ControlMode::JOINER_WAIT_JOIN ||
        in.controlMode == ControlMode::JOINER_WAIT_REPLY ||
        in.controlMode == ControlMode::JOINER_WAIT_INFORMATION;

    if (!inManeuverPhase) {
        if (initialFrontId_ >= 0) {
            PlatoonNeighborState front;
            simtime_t frontAge = SIMTIME_ZERO;
            const bool frontFresh = getNeighbor(initialFrontId_, front, frontAge) 
                                    && (frontAge <= maxAge_);

            if (frontFresh) {
                in.predecessor.valid = true;
                in.predecessor.speed = front.scalarSpeed;
                in.predecessor.actualAcceleration = front.actualAcceleration;
                in.predecessor.controllerAcceleration =
                    std::isfinite(front.controllerAcceleration)
                    ? front.controllerAcceleration
                    : front.actualAcceleration;
                in.predecessor.distance = computeGapToNeighbor(front);
                in.targetGap = getTargetDistance(front.scalarSpeed);

                const double gapError = in.predecessor.distance - in.targetGap;
                const double speedBias = clampValue(
                    gapError * kGapSpeed_,
                    -maxClosureSpeed_,
                    maxClosureSpeed_);
                in.targetSpeed = std::max(0.0, front.scalarSpeed + speedBias);
            } else {
                in.targetSpeed = platoonDesiredSpeed_;
                in.targetGap = getTargetDistance(in.egoSpeed);
            }

            // Select controller based on type
            if (controllerType_ == "ACC") {
                in.activeController = ActiveController::ACC;
            } else if (controllerType_ == "CC") {
                in.activeController = ActiveController::CC;
            } else {
                in.activeController = ActiveController::CACC;  // default
            }

        } else {
            in.activeController = ActiveController::CC;
            in.targetSpeed = leaderTargetSpeed_;
            in.targetGap = getTargetDistance(in.egoSpeed);
        }
    } else {
        in.targetSpeed = controllerAdapter_.getCruiseControlDesiredSpeed();
    }

    return in;
}

// Runs the selected controller and returns desired speed/acceleration.
// This function does not emit signals and does not convert to throttle/brake.
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

    ControlOutput out = controllerDispatcher_.compute(in);

    EV_INFO << "[CarlaGeneralPlatooningApp][computeControllerOutput]"
            << " actor=" << actorId_
            << " role=" << static_cast<int>(role_)
            << " mode=" << static_cast<int>(in.controlMode)
            << " activeController=" << static_cast<int>(in.activeController)
            << " egoSpeed=" << in.egoSpeed
            << " targetSpeed=" << in.targetSpeed
            << " targetGap=" << in.targetGap
            << " predValid=" << in.predecessor.valid
            << " predDistance=" << (in.predecessor.valid ? in.predecessor.distance : -1.0)
            << " predSpeed=" << (in.predecessor.valid ? in.predecessor.speed : -1.0)
            << " leaderValid=" << in.leader.valid
            << " leaderSpeed=" << (in.leader.valid ? in.leader.speed : -1.0)
            << " fakeFrontValid=" << in.fakeFront.valid
            << " fakeFrontDistance=" << (in.fakeFront.valid ? in.fakeFront.distance : -1.0)
            << " fakeLeaderValid=" << in.fakeLeader.valid
            << " desiredSpeed=" << out.desiredSpeed
            << " desiredAcceleration=" << out.desiredAcceleration
            << "\n";

    return out;
}

// Periodic longitudinal-control tick.
// Reads current CARLA state, computes controller acceleration, tracks it with CARLA actuators,
// emits bridge/control signals, and exports Plexe-compatible result vectors.
void CarlaGeneralPlatooningApp::onControlTick()
{
    // Read current CARLA/INET state first.
    // The actuator tracker needs measured acceleration before it chooses throttle/brake for this tick.
    veins::Coord vel(0, 0, 0);
    veins::Coord accelVec(0, 0, 0);
    if (mobility_) {
        vel = toVeinsCoord(mobility_->getCurrentVelocity());
        accelVec = toVeinsCoord(mobility_->getCurrentAcceleration());
    }

    const double currentSpeed = scalarSpeedFromVelocity(vel);

    double measuredAcceleration = 0.0;
    bool usedSpeedDerivativeAcceleration = false;
    bool usedCarlaAcceleration = false;

    // Primary: signed longitudinal acceleration from the same speed signal exported to the parser.
    if (lastExportTime_ >= SIMTIME_ZERO && simTime() > lastExportTime_) {
        const double dt = (simTime() - lastExportTime_).dbl();
        if (dt > 1e-9) {
            measuredAcceleration = (currentSpeed - lastExportSpeed_) / dt;
            usedSpeedDerivativeAcceleration = std::isfinite(measuredAcceleration);
        }
    }

    // Fallback: projection of CARLA acceleration onto direction of travel.
    if (!usedSpeedDerivativeAcceleration && currentSpeed > 1e-6) {
        measuredAcceleration =
            (accelVec.x * vel.x + accelVec.y * vel.y) / currentSpeed;
        usedCarlaAcceleration = std::isfinite(measuredAcceleration);
    }

    if (!std::isfinite(measuredAcceleration)) {
        measuredAcceleration = 0.0;
        usedCarlaAcceleration = false;
    }

    actualAcceleration_ = measuredAcceleration;

    // For sinusoidal speed if configured
    if (sinusoidalSpeed_ && simTime().dbl() >= sinusoidalStartAt_) {
        const double t = simTime().dbl() - sinusoidalStartAt_;
        leaderTargetSpeed_ = nominalPlatoonSpeed_ + 
            sinusoidalAmplitude_ * std::sin(2.0 * M_PI * t / sinusoidalPeriod_);
        leaderTargetSpeed_ = std::max(0.0, leaderTargetSpeed_);
    }

    // Compute Plexe-style controller acceleration.
    ControlOutput out = computeControllerOutput();

    const double rawAcceleration = out.desiredAcceleration;

    desiredAcceleration_ = clampValue(rawAcceleration, aMin_, aMax_);
    desiredSpeed_ = out.desiredSpeed;
    hasControl_ = out.hasControl;
    controlMode_ = out.controlMode;

    // Convert controller acceleration to CARLA throttle/brake.
    // pyCARLANeT applies these fields; it should not implement platoon control.
    out.desiredAcceleration = desiredAcceleration_;
    out.desiredSpeed = desiredSpeed_;
    out.hasControl = hasControl_;
    out.controlMode = controlMode_;

    fillTrackedLongitudinalActuation(out, actualAcceleration_, currentSpeed);

    // Plexe-style exported metrics.
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

            // Match control-law sign convention: predecessor speed minus ego speed.
            exportedRelativeSpeed = pred.scalarSpeed - currentSpeed;
        }
    }

    // Lateral control
    if (lateralControlEnabled_ && mobility_) {
        const ObservedVehicleState obsState = buildObservedVehicleState();
        if (!obsState.routePolyline.empty()) {
            lateralController_.initialize(
                obsState.routePolyline,
                obsState.wheelbaseM,
                obsState.maximumFrontWheelSteeringRad);

            double referenceOffsetM = 0.0;
            if (laneChangeManeuver_.isActive()) {
                const int hopLaneId = laneChangeManeuver_.currentHopTargetLaneId();
                const double liveOffset = mobility_->getLateralOffsetToLaneId(hopLaneId);
                laneChangeManeuver_.tick(obsState, liveOffset);
                // Use live offset scaled to prevent aggressive steering
                const double scaledOffset = liveOffset * 0.85; // 85% seems to be the sweetspot
                referenceOffsetM = scaledOffset;

                // Pass live offset to tick() only for settling/completion detection
                laneChangeManeuver_.tick(obsState, liveOffset);

                if (laneChangeManeuver_.state() ==
                    LaneChangeState::LANE_CHANGE_COMPLETE) {
                    referenceOffsetM = laneChangeManeuver_.completedOffsetM();
                    setInManeuver(false, nullptr);
                    const JoinSlot slot = laneChangeManeuver_.targetSlot();
                    JoinManeuverParameters params;
                    params.platoonId = slot.predecessorActorId;
                    params.leaderId = slot.predecessorActorId;
                    params.position = -1;
                    joinManeuver_->startManeuver(&params);
                    laneChangeManeuver_.reset();
                } else if (laneChangeManeuver_.state() ==
                        LaneChangeState::ABORTED) {
                    setInManeuver(false, nullptr);
                    laneChangeManeuver_.reset();
                }
            }

            if (laneChangeManeuver_.isActive()) {
                lateralController_.setLookaheadM(15.0);
            } else {
                lateralController_.setLookaheadM(6.0);
            }

            const LateralSteeringResult result =
                lateralController_.computeSteering(obsState, referenceOffsetM);
            if (result.status == LateralSteeringStatus::Valid) {
                lastSteerRad_ = result.steeringRad;
                lastValidSteeringAvailable_ = true;
            }
            const double maxSteer = obsState.maximumFrontWheelSteeringRad > 0.0
                ? obsState.maximumFrontWheelSteeringRad : 0.7;
            lastNormalizedSteer_ = std::max(-1.0, std::min(1.0,
                lastSteerRad_ / maxSteer));
            lateralControlActive_ = true;
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
            << " accelTrackError=" << actuatorAccelError_
            << " accelTrackIntegral=" << actuatorAccelIntegral_
            << " accelTrackEffort=" << actuatorEffort_
            << " targetThrottle=" << actuatorTargetThrottle_
            << " targetBrake=" << actuatorTargetBrake_
            << " throttle=" << out.throttle
            << " brake=" << out.brake
            << " handBrake=" << (out.handBrake ? 1 : 0)
            << " reverse=" << (out.reverse ? 1 : 0)
            << " usedSpeedDerivativeAcceleration=" << (usedSpeedDerivativeAcceleration ? 1 : 0)
            << " autopilotSteering=1"
            << " usedCarlaAcceleration=" << (usedCarlaAcceleration ? 1 : 0)
            << " accelX=" << accelVec.x
            << " accelY=" << accelVec.y
            << " accelZ=" << accelVec.z
            << " distance=" << exportedDistance
            << " relativeSpeed=" << exportedRelativeSpeed
            << "\n";

    // Bridge/control signals consumed by BridgeApp and pyCARLANeT path.
    emit(desiredAccelerationSignal_, desiredAcceleration_);
    emit(desiredSpeedSignal_, desiredSpeed_);
    emit(hasControlSignal_, hasControl_ ? 1.0 : 0.0);
    emit(controlModeSignal_, static_cast<long>(static_cast<int>(controlMode_)));
    emit(activeControllerSignal_, static_cast<long>(static_cast<int>(controllerAdapter_.getActiveController())));

    emit(controlThrottleSignal_, out.throttle);
    emit(controlBrakeSignal_, out.brake);
    emit(controlHandBrakeSignal_, out.handBrake ? 1.0 : 0.0);
    emit(controlReverseSignal_, out.reverse ? 1.0 : 0.0);
    emit(controlManualGearShiftSignal_, out.manualGearShift ? 1.0 : 0.0);

    // Plexe-style exported vectors for post-processing.
    emit(speedSignal_, currentSpeed);
    emit(accelerationSignal_, actualAcceleration_);
    emit(controllerAccelerationExportSignal_, desiredAcceleration_);
    emit(distanceSignal_, exportedDistance);
    emit(relativeSpeedSignal_, exportedRelativeSpeed);

    // Steering controller
    emit(controlSteerSignal_, lastNormalizedSteer_);
    emit(lateralControlActiveSignal_, lateralControlActive_ ? 1.0 : 0.0);
}

// Updates the semantic control mode and keeps adapter/helper controller labels consistent.
// Maneuver code calls this when a vehicle changes phase or becomes a normal follower.
void CarlaGeneralPlatooningApp::maneuverSetControlMode(ControlMode mode)
{
    controllerAdapter_.setControlMode(mode);
    controlMode_ = mode;

    switch (mode) {
        case ControlMode::LEADER_CRUISE:
        case ControlMode::JOINER_FREE_CRUISE:
        case ControlMode::JOINER_WAIT_REPLY:
        case ControlMode::JOINER_WAIT_INFORMATION:
            controllerAdapter_.setActiveController(ActiveController::CC);
            positionHelper_.setController(ActiveController::CC);
            break;

        case ControlMode::FOLLOWER_PLATOON:
            controllerAdapter_.setActiveController(ActiveController::CACC);
            positionHelper_.setController(ActiveController::CACC);
            break;

        case ControlMode::JOINER_MOVE_IN_POSITION:
        case ControlMode::JOINER_WAIT_JOIN:
            controllerAdapter_.setActiveController(ActiveController::FAKED_CACC);
            positionHelper_.setController(ActiveController::FAKED_CACC);
            break;

        case ControlMode::HOLD:
        default:
            break;
    }
}

// Sets the vehicle id that the joiner should track while moving into position.
// Usually derived from the formation carried in MoveToPosition or JoinFormation.
void CarlaGeneralPlatooningApp::maneuverSetJoinFrontVehicleId(int frontId)
{
    joinFrontVehicleId_ = frontId;
}

// Clears temporary joiner front-vehicle tracking after the vehicle becomes a normal follower.
void CarlaGeneralPlatooningApp::maneuverClearJoinFrontVehicleId()
{
    joinFrontVehicleId_ = -1;
}

// Updates the stored platoon speed used by controller inputs and maneuver metadata.
void CarlaGeneralPlatooningApp::maneuverSetPlatoonSpeed(double platoonSpeed)
{
    nominalPlatoonSpeed_ = platoonSpeed;
    positionHelper_.setPlatoonSpeed(platoonSpeed);
}

// Replaces the local formation view.
// The formation order determines predecessor selection through PositionHelper.
void CarlaGeneralPlatooningApp::maneuverSetFormation(const std::vector<int>& formation)
{
    positionHelper_.setPlatoonFormation(formation);
}

// Completes a join by converting the joiner into a normal CACC follower.
// After this call, predecessor/leader state comes from the formation instead of joinFrontVehicleId_.
void CarlaGeneralPlatooningApp::maneuverCompleteJoinAsFollower(
    const std::vector<int>& formation,
    int platoonId,
    int leaderId)
{
    role_ = PlatoonRole::FOLLOWER;

    positionHelper_.setPlatoonFormation(formation);
    positionHelper_.setPlatoonId(platoonId);
    positionHelper_.setLeaderId(leaderId);
    positionHelper_.setController(ActiveController::CACC);

    controllerAdapter_.setControlMode(ControlMode::FOLLOWER_PLATOON);
    controllerAdapter_.setActiveController(ActiveController::CACC);

    controlMode_ = ControlMode::FOLLOWER_PLATOON;
    joinFrontVehicleId_ = -1;
    platoonId_ = platoonId;
    leaderId_ = leaderId;
}

ObservedVehicleState CarlaGeneralPlatooningApp::buildObservedVehicleState() const
{
    ObservedVehicleState state;
    if (!mobility_) return state;

    const auto pos = toVeinsCoord(mobility_->getCurrentPosition());
    const auto vel = toVeinsCoord(mobility_->getCurrentVelocity());
    const auto acc = toVeinsCoord(mobility_->getCurrentAcceleration());

    state.positionM     = pos;
    state.velocityMps   = vel;
    state.accelerationMps2 = acc;
    state.speedMps      = scalarSpeedFromVelocity(vel);
    state.yawRad        = mobility_->getYawRad();
    state.wheelbaseM    = mobility_->getWheelbaseM();
    state.maximumFrontWheelSteeringRad = mobility_->getMaxSteerRad();

    // Build route polyline from raw vectors
    const auto& rx    = mobility_->getRouteX();
    const auto& ry    = mobility_->getRouteY();
    const auto& ryaw  = mobility_->getRouteYaw();
    const auto& rroad = mobility_->getRouteRoadId();
    const auto& rlane = mobility_->getRouteLaneId();

    state.routePolyline.clear();
    for (size_t i = 0; i < rx.size(); ++i) {
        LateralRoutePoint pt;
        pt.xM        = rx[i];
        pt.yM        = ry[i];
        pt.yawRad    = ryaw[i];
        pt.roadId    = rroad[i];
        pt.laneId    = rlane[i];
        pt.routeIndex = (int)i;
        state.routePolyline.push_back(pt);
    }

    state.routeProjectionValid = !state.routePolyline.empty();

    return state;
}

} // namespace carla
