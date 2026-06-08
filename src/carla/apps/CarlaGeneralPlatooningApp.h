#pragma once

#include <map>
#include <string>
#include <vector>

#include <omnetpp.h>

#include "veins/base/utils/Coord.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

#include "carlanet/CarlaInetMobility.h"
#include "carla/platooning/adapters/CarlaControllerAdapter.h"
#include "carla/platooning/adapters/CarlaPositionHelper.h"
#include "carla/platooning/adapters/ICarlaPlatooningApp.h"
#include "carla/platooning/controller/ControllerDispatcher.h"
#include "carla/platooning/controller/ControllerInputs.h"
#include "carla/platooning/maneuver/CarlaJoinAtBack.h"
#include "carla/platooning/state/ControlOutput.h"
#include "carla/platooning/state/PlatoonNeighborState.h"

#include "plexe/messages/JoinFormation_m.h"
#include "plexe/messages/JoinPlatoonRequest_m.h"
#include "plexe/messages/JoinPlatoonResponse_m.h"
#include "plexe/messages/ManeuverMessage_m.h"
#include "plexe/messages/MoveToPosition_m.h"
#include "plexe/messages/PlatooningBeacon_m.h"
#include "plexe/messages/UpdatePlatoonData_m.h"
#include "plexe/messages/UpdatePlatoonFormation_m.h"

namespace carla {
// Id, cost needed for selecting a car to follow
struct PlatoonCandidate {
    bool valid = false;
    int vehicleId = -1;
    double cost = std::numeric_limits<double>::infinity();
};
/*
 * Main OMNeT++ application module for CARLA platooning nodes.
 *
 * This class is the central connection point between:
 *   - Veins/802.11p packets and beacon reception
 *   - maneuver protocol state machines such as CarlaJoinAtBack
 *   - local neighbor state derived from V2V beacons
 *   - longitudinal controller input/output construction
 *   - BridgeApp signals that eventually reach pyCARLANeT/CARLA
 *
 * Protocol decisions should live here or in maneuver classes. CARLA-side Python
 * code should apply actuation and return positions, not decide platoon membership.
 */
class CarlaGeneralPlatooningApp : public veins::DemoBaseApplLayer, public ICarlaPlatooningApp {
public:
    // Constructs the app object. OMNeT++ parameter loading happens in initialize().
    CarlaGeneralPlatooningApp();

    // Deletes timers and maneuver objects owned by this module.
    ~CarlaGeneralPlatooningApp() override;

    // Current logical role of this vehicle in the platoon protocol.
    const PlatoonRole& getPlatoonRole() const override { return role_; }
    void setPlatoonRole(PlatoonRole r) override { role_ = r; }

    // True while a maneuver state machine currently owns protocol progress for this vehicle.
    bool isInManeuver() const override { return inManeuver_; }
    void setInManeuver(bool b, CarlaManeuver* maneuver) override;

    // Provides maneuver code access to persistent platoon metadata.
    CarlaPositionHelper* getPositionHelper() override { return &positionHelper_; }

    // Provides maneuver code access to controller mode and temporary fake vehicle data.
    CarlaControllerAdapter* getControllerAdapter() override { return &controllerAdapter_; }

    // Spacing-policy accessors used by maneuver code and controller input construction.
    double getStandstillDistance(ActiveController controller) const override;
    double getHeadway(ActiveController controller) const override;
    double getTargetDistance(double speed) const override;
    double getTargetDistance(ActiveController controller, double speed) const override;
    ActiveController getTargetController() const override;

    // Reads the current CARLA-derived position from CarlaInetMobility.
    veins::Coord getCurrentPosition() const override;

    // Returns the lane metadata used in maneuver messages.
    int getCurrentLaneIndex() const override;

    // Sends a logical unicast over the broadcast 802.11p channel.
    // The destination id is stored in the message and checked by receivers.
    void sendUnicast(omnetpp::cPacket* msg, int destination) override;

    // Fills common fields shared by all maneuver messages.
    void fillManeuverMessage(
        ManeuverMessage* msg,
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId) override;

    // Factory for leader-to-member formation updates after a maneuver changes ordering.
    UpdatePlatoonFormation* createUpdatePlatoonFormation(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& platoonFormation) override;

    // Factory for platoon metadata updates that also change platoon id.
    UpdatePlatoonData* createUpdatePlatoonData(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& platoonFormation,
        int newPlatoonId) override;

    // Maneuver-facing state mutation helpers.
    // These keep app state, PositionHelper, and ControllerAdapter synchronized.
    void maneuverSetControlMode(ControlMode mode);
    void maneuverSetJoinFrontVehicleId(int frontId);
    void maneuverClearJoinFrontVehicleId();
    void maneuverSetPlatoonSpeed(double platoonSpeed);
    void maneuverSetFormation(const std::vector<int>& formation);
    void maneuverCompleteJoinAsFollower(const std::vector<int>& formation, int platoonId, int leaderId);

protected:
    // OMNeT++ lifecycle hook. Reads NED/ini parameters and schedules timers.
    void initialize(int stage) override;

    // Handles this module's timers: beacon timer, control timer, and maneuver-start timer.
    void handleSelfMsg(omnetpp::cMessage* msg) override;

    // Handles packets from the Veins lower layer.
    // Decapsulates frames, updates beacon state, and dispatches maneuver messages.
    void handleLowerMsg(omnetpp::cMessage* msg) override;

    // Starts join-at-back when this node is configured as a joiner and the start timer fires.
    void startJoinManeuverIfConfigured();

    // Updates local neighbor cache from one received PlatooningBeacon.
    void refreshNeighborFromBeacon(const PlatooningBeacon* pb);

    // Retrieves cached neighbor state and reports the age of the data.
    bool getNeighbor(int id, PlatoonNeighborState& out, omnetpp::simtime_t& age) const;

    // Gives maneuver code a chance to react to every received platoon beacon.
    void onPlatoonBeacon(const PlatooningBeacon* pb);

    // Dispatches maneuver messages into the active maneuver state machine.
    void onManeuverMessage(const ManeuverMessage* mm);

    // Applies leader-issued formation changes to followers.
    void handleUpdatePlatoonFormation(const UpdatePlatoonFormation* msg);

    // Applies leader-issued platoon metadata changes to followers.
    void handleUpdatePlatoonData(const UpdatePlatoonData* msg);

    // Broadcasts local vehicle state for V2V control and maneuver logic.
    void sendPlatooningBeacon();

    // Periodic controller loop. Computes acceleration, tracks it with CARLA actuators, and emits signals.
    void onControlTick();

    // Converts 2D velocity vector to scalar speed in m/s.
    double scalarSpeedFromVelocity(const veins::Coord& v) const;

    // Parses omnetpp.ini formation strings such as "0,1,2,3".
    std::vector<int> parseFormation(const std::string& csv) const;

    // Computes approximate bumper gap to a neighbor from beacon position and vehicle length.
    double computeGapToNeighbor(const PlatoonNeighborState& n) const;

    // Converts spacing error to a bounded speed bias for target-speed diagnostics/control support.
    double computeSpeedBiasFromGapError(double gapError, double gain, double limit) const;

    // Finds the predecessor of this vehicle in a formation vector.
    int findFrontVehicleIdInFormation(const std::vector<int>& formation) const;

    // Clears internal throttle/brake tracking state.
    void resetLongitudinalActuatorTracker();

    // Converts desired acceleration into CARLA throttle/brake using measured acceleration feedback.
    void fillTrackedLongitudinalActuation(
        ControlOutput& out,
        double measuredAcceleration,
        double currentSpeed);

    // Builds the full controller input bundle from role, mode, formation, and V2V beacon cache.
    ControllerInputs buildControllerInputs() const;

    // Runs the configured longitudinal controller and returns desired speed/acceleration.
    ControlOutput computeControllerOutput() const;

protected:
    // Mobility module updated by CarlanetManager from pyCARLANeT/CARLA snapshots.
    CarlaInetMobility* mobility_ = nullptr;

    // Identity and platoon metadata loaded from omnetpp.ini.
    std::string actorId_; // CARLA actor id/name used by the Python bridge and logs.
    int nodeId_ = -1;     // Logical OMNeT++ vehicle id used in V2V messages.
    int platoonId_ = -1;  // Logical platoon id.
    int leaderId_ = -1;   // Logical id of this vehicle's platoon leader.

    // Basic vehicle and speed parameters.
    double nominalPlatoonSpeed_ = 0.0; // m/s; steady platoon cruise speed.
    double leaderTargetSpeed_ = 0.0;   // m/s; leader cruise-control target.
    double vehicleLength_ = 4.5;       // m; used to convert center distance to bumper gap.

    // Spacing and longitudinal controller parameters.
    double headway_ = 0.5;  // s; time-headway parameter for CTS-style policies.
    double gapMin_ = 5.0;   // m; constant bumper gap for CDS-style spacing.
    double kGap_ = 0.2;     // gain on spacing error.
    double kDv_ = 0.7;      // gain on relative-speed error.
    double kSpeedP_ = 0.7;  // gain on cruise speed error.
    double aMin_ = -4.0;    // m/s^2; minimum allowed acceleration, usually braking.
    double aMax_ = 2.5;     // m/s^2; maximum allowed acceleration.

    // Additional CACC/feed-forward and debug target-speed gains.
    double kAccFF_ = 1.0;          // gain on predecessor/leader controller acceleration.
    double kLeaderDv_ = 0.05;      // gain on leader-relative speed.
    double kGapSpeed_ = 0.03;      // maps gap error to target-speed bias.
    double maxClosureSpeed_ = 1.0; // m/s; cap on closing-speed bias.

    // Plexe/SUMO CACC model parameters exported for parity analysis.
    double caccXi_ = 1.0;      // damping ratio.
    double caccOmegaN_ = 0.2;  // natural frequency.

    // Decentralized parameters
    double alpha_ = 0.6;
    double p_ = 0.4;
    double r_ = 400.0;
    int initialFrontId_ = -1;
    int initialBackId_ = -1;
    double platoonDesiredSpeed_ = 10.0;
    omnetpp::cMessage* heuristicTimer_ = nullptr; // needed to collect entries for the neighbor table

    // FOR TESTING PURPOSES ONLY
    omnetpp::cMessage* brakeTimer_ = nullptr;

    // OMNeT++ timing parameters.
    simtime_t beaconInterval_ = SIMTIME_ZERO;    // interval between V2V beacons.
    simtime_t controlInterval_ = SIMTIME_ZERO;   // interval between controller updates.
    simtime_t maxAge_ = SIMTIME_ZERO;            // maximum acceptable age of neighbor beacon data.
    simtime_t startManeuverAt_ = SIMTIME_ZERO;   // simulation time when a joiner starts its maneuver.

    // Joiner-specific maneuver parameters.
    int joinPosition_ = -1;        // requested join position; join-at-back typically appends at tail.
    double approachDeltaV_ = 0.0;  // m/s; speed offset used while joiner approaches platoon.
    double inPosSlack_ = 0.5;      // m; tolerance used when deciding if joiner is close enough.

    // Persistent platoon metadata helper: formation, leader, lane, spacing, controller label.
    CarlaPositionHelper positionHelper_;

    // Mutable controller/maneuver adapter state: mode, active controller, fake front/leader data.
    CarlaControllerAdapter controllerAdapter_;

    // Selects and runs the actual CC/CACC/faked-CACC controller implementation.
    ControllerDispatcher controllerDispatcher_;

    // High-level role in the platooning protocol.
    PlatoonRole role_ = PlatoonRole::NONE;

    // Whether a maneuver state machine is currently active for this node.
    bool inManeuver_ = false;

    // Current maneuver object receiving self-messages and maneuver messages.
    CarlaManeuver* activeManeuver_ = nullptr;

    // Join-at-back maneuver object owned by this app.
    CarlaJoinAtBack* joinManeuver_ = nullptr;

    // OMNeT++ self-message timers.
    omnetpp::cMessage* controlTimer_ = nullptr; // periodic longitudinal-control timer.
    omnetpp::cMessage* startTimer_ = nullptr;   // one-shot maneuver-start timer for joiners.

    // Local world model built from received V2V beacons, keyed by logical vehicle id.
    std::map<int, PlatoonNeighborState> neighborsByVehicleId_;

    // Latest controller outputs emitted toward BridgeApp/pyCARLANeT.
    double desiredAcceleration_ = 0.0; // m/s^2; desired controller acceleration before actuator mapping.
    double desiredSpeed_ = 0.0;        // m/s; target/reference speed.
    bool hasControl_ = false;          // false means hold/brake instead of applying longitudinal control.
    ControlMode controlMode_ = ControlMode::HOLD;

    // Measured/exported motion state used for analysis and actuator tracking.
    double actualAcceleration_ = 0.0;              // m/s^2; estimated longitudinal acceleration.
    double lastExportSpeed_ = 0.0;                 // m/s; previous scalar speed sample.
    simtime_t lastExportTime_ = SIMTIME_ZERO;      // time of previous speed sample.

    // Closed-loop CARLA acceleration tracking.
    // The CACC/CC controller still outputs a Plexe-style acceleration command.
    // These fields belong to the actuator adapter that converts that command
    // into CARLA throttle/brake while tracking measured longitudinal acceleration.
    double actuatorThrottleCmd_ = 0.0;      // normalized CARLA throttle command [0, 1].
    double actuatorBrakeCmd_ = 0.0;         // normalized CARLA brake command [0, 1].
    double actuatorAccelIntegral_ = 0.0;    // integral term for acceleration-tracking PI loop.
    double actuatorAccelError_ = 0.0;       // latest desired-minus-measured acceleration error.
    double actuatorEffort_ = 0.0;           // signed effort; positive throttle, negative brake.
    double actuatorTargetThrottle_ = 0.0;   // pre-rate-limit throttle target.
    double actuatorTargetBrake_ = 0.0;      // pre-rate-limit brake target.
    bool actuatorTrackerInitialized_ = false;
    omnetpp::simtime_t lastActuatorUpdateTime_ = SIMTIME_ZERO;

    // Tracks semantic control-mode transitions so actuator memory does not leak
    // across JOINER/FOLLOWER/LEADER mode changes.
    ControlMode actuatorLastControlMode_ = ControlMode::HOLD;
    bool actuatorLastControlModeValid_ = false;

    // Temporary front-vehicle id used by a joiner before it becomes a normal follower.
    int joinFrontVehicleId_ = -1;

    // Monotonic sequence number placed in PlatooningBeacon messages.
    long beaconSequence_ = 0;

    // Signals carrying controller intent to BridgeApp and result vectors.
    static omnetpp::simsignal_t desiredAccelerationSignal_;
    static omnetpp::simsignal_t desiredSpeedSignal_;
    static omnetpp::simsignal_t controlModeSignal_;
    static omnetpp::simsignal_t activeControllerSignal_;
    static omnetpp::simsignal_t hasControlSignal_;

    // Signals carrying final CARLA actuator commands after acceleration tracking.
    static omnetpp::simsignal_t controlThrottleSignal_;
    static omnetpp::simsignal_t controlBrakeSignal_;
    static omnetpp::simsignal_t controlHandBrakeSignal_;
    static omnetpp::simsignal_t controlReverseSignal_;
    static omnetpp::simsignal_t controlManualGearShiftSignal_;

    // Plexe-style result signals consumed by parser/analysis tooling.
    static omnetpp::simsignal_t speedSignal_;
    static omnetpp::simsignal_t accelerationSignal_;
    static omnetpp::simsignal_t controllerAccelerationExportSignal_;
    static omnetpp::simsignal_t distanceSignal_;
    static omnetpp::simsignal_t relativeSpeedSignal_;


    // helper function
    PlatoonCandidate evaluatePlatoonCandidates() const;
};

} // namespace carla
