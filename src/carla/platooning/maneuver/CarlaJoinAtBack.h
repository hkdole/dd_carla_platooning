#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <omnetpp.h>

#include "veins/base/utils/Coord.h"

#include "carla/platooning/maneuver/CarlaJoinManeuver.h"

namespace carla {

/*
 * Join-at-back maneuver state machine.
 *
 * This class implements the V2V protocol for adding one joiner vehicle to the
 * rear of an existing platoon. The same class runs on both the joiner and the
 * leader; the active behavior depends on the current PlatoonRole stored in the
 * owning CarlaGeneralPlatooningApp.
 *
 * Baseline centralized message sequence:
 *
 *   Joiner -> Leader : JoinPlatoonRequest
 *   Leader -> Joiner : JoinPlatoonResponse
 *   Leader -> Joiner : MoveToPosition
 *   Joiner -> Leader : MoveToPositionAck
 *   Leader -> Joiner : JoinFormation
 *   Joiner -> Leader : JoinFormationAck
 *   Leader -> Followers : UpdatePlatoonFormation
 *
 * The maneuver only decides protocol state and temporary joiner control phase.
 * Periodic beacons, neighbor cache updates, and controller input construction
 * are handled by CarlaGeneralPlatooningApp.
 */
class CarlaJoinAtBack : public CarlaJoinManeuver {
public:
    // Creates maneuver state for the owning platooning app module.
    // The app interface provides message sending, role changes, controller state,
    // current position/lane access, and OMNeT++ module ownership.
    explicit CarlaJoinAtBack(ICarlaPlatooningApp* app);

    // Cancels and deletes the retry timer used by joiner-side request retries.
    ~CarlaJoinAtBack() override;

    // Starts the joiner-side maneuver.
    // Expects JoinManeuverParameters with target platoon id, leader id, and join position.
    void startManeuver(const void* parameters) override;

    // Stops the maneuver and restores role-specific safe control state.
    // Also clears temporary fake leader/front data used during the join approach.
    void abortManeuver() override;

    // Receives every PlatooningBeacon seen by the app while this maneuver is active.
    // Joiner uses this during J_MOVE_IN_POSITION to update fake-front/fake-leader data
    // and decide when it is close enough to send MoveToPositionAck.
    void onPlatoonBeacon(const PlatooningBeacon* pb) override;

    // Called if the lower communication layer reports a failed maneuver packet.
    // Current implementation treats this as fatal because baseline runs assume delivery attempts are accepted.
    void onFailedTransmissionAttempt(const ManeuverMessage* mm) override;

    // Handles maneuver-owned OMNeT++ self-messages.
    // Currently used for retrying JoinPlatoonRequest while waiting for JoinPlatoonResponse.
    bool handleSelfMsg(omnetpp::cMessage* msg) override;

    // Leader-side handler for the first join request.
    // Decides whether the joiner is permitted and, if so, sends MoveToPosition.
    void handleJoinPlatoonRequest(const JoinPlatoonRequest* msg) override;

    // Joiner-side handler for the leader's permission response.
    // Positive response moves the joiner from J_WAIT_REPLY to J_WAIT_INFORMATION.
    void handleJoinPlatoonResponse(const JoinPlatoonResponse* msg) override;

    // Joiner-side handler for the leader's proposed formation and target lane/speed.
    // Configures fake-CACC approach behavior and moves to J_MOVE_IN_POSITION.
    void handleMoveToPosition(const MoveToPosition* msg) override;

    // Leader-side handler for the joiner's “close enough” acknowledgment.
    // Sends JoinFormation to switch the joiner into real follower membership.
    void handleMoveToPositionAck(const MoveToPositionAck* msg) override;

    // Joiner-side handler for the final join command.
    // Converts the joiner into a normal follower and sends JoinFormationAck.
    void handleJoinFormation(const JoinFormation* msg) override;

    // Leader-side handler for the joiner's final acknowledgment.
    // Updates leader formation and broadcasts UpdatePlatoonFormation to existing followers.
    void handleJoinFormationAck(const JoinFormationAck* msg) override;

private:
    /*
     * Maneuver protocol state.
     *
     * Prefix convention:
     *   J_ = state used by the joiner
     *   L_ = state used by the leader
     *
     * This is separate from ControlMode. JoinManeuverState describes message
     * protocol progress; ControlMode describes longitudinal control behavior.
     */
    enum class JoinManeuverState {
        IDLE, // No active join-at-back protocol on this vehicle.

        // Joiner states.
        J_WAIT_REPLY,       // Join request sent; waiting for JoinPlatoonResponse.
        J_WAIT_INFORMATION, // Permission received; waiting for MoveToPosition.
        J_MOVE_IN_POSITION, // Driving toward the rear of the platoon using fake-CACC approach control.
        J_WAIT_JOIN,        // MoveToPositionAck sent; waiting for JoinFormation.

        // Leader states.
        L_WAIT_JOINER_IN_POSITION, // Joiner accepted; waiting for MoveToPositionAck.
        L_WAIT_JOINER_TO_JOIN,     // JoinFormation sent; waiting for JoinFormationAck.
    };

    /*
     * Joiner-side view of the target platoon.
     *
     * Filled from JoinManeuverParameters at start, then updated from MoveToPosition.
     * The newFormation vector determines which vehicle the joiner should track as
     * its front vehicle while moving into position.
     */
    struct TargetPlatoonData {
        int platoonId = -1;       // Target platoon id.
        int platoonLeader = -1;   // Leader vehicle id of the target platoon.
        int platoonLane = -1;     // Lane index advertised by the leader.
        double platoonSpeed = 0.0; // m/s; platoon speed advertised by the leader.
        int joinIndex = -1;       // Index of this joiner inside newFormation.
        std::vector<int> newFormation; // Proposed platoon order after the join.

        // Copies target-platoon metadata from MoveToPosition.
        // DestinationId is the joiner id; its location in newFormation identifies joinIndex.
        void from(const MoveToPosition* msg)
        {
            platoonId = msg->getPlatoonId();
            platoonLeader = msg->getVehicleId();
            platoonLane = msg->getPlatoonLane();
            platoonSpeed = msg->getPlatoonSpeed();

            newFormation.resize(msg->getNewPlatoonFormationArraySize());
            for (unsigned int i = 0; i < msg->getNewPlatoonFormationArraySize(); ++i) {
                newFormation[i] = msg->getNewPlatoonFormation(i);
            }

            auto it = std::find(newFormation.begin(), newFormation.end(), msg->getDestinationId());
            if (it != newFormation.end()) {
                joinIndex = static_cast<int>(std::distance(newFormation.begin(), it));
            }
        }

        // Returns the vehicle directly ahead of the joiner in the proposed formation.
        // Returns -1 if the joiner is not found, is first, or the formation is invalid.
        int frontId() const
        {
            if (joinIndex <= 0 || joinIndex >= static_cast<int>(newFormation.size())) return -1;
            return newFormation.at(joinIndex - 1);
        }
    };

    /*
     * Leader-side view of the joiner.
     *
     * Filled from JoinPlatoonRequest and later extended with the proposed formation.
     * The leader owns formation construction in this centralized baseline.
     */
    struct JoinerData {
        int joinerId = -1;     // Logical vehicle id of the joining vehicle.
        int joinerLane = -1;   // Lane reported by the joiner in JoinPlatoonRequest.
        std::vector<int> newFormation; // Formation after appending the joiner at the back.

        // Copies joiner metadata from JoinPlatoonRequest.
        void from(const JoinPlatoonRequest* msg)
        {
            joinerId = msg->getVehicleId();
            joinerLane = msg->getCurrentLaneIndex();
        }
    };

    // Initializes joiner-side maneuver state and validates that no maneuver is already active.
    bool initializeJoinManeuver(const void* parameters);

    // Leader-side validation/acceptance for JoinPlatoonRequest.
    // Sends JoinPlatoonResponse and stores JoinerData when permission is granted.
    bool processJoinRequest(const JoinPlatoonRequest* msg);

    // Sends or retries JoinPlatoonRequest from joiner to target leader.
    void sendJoinRequest();

    // Computes the final follower gap using the app-level spacing policy.
    // This is not necessarily the same as approach spacing during J_MOVE_IN_POSITION.
    double computeTargetJoinGapMeters(double frontSpeedMetersPerSecond) const;

private:
    JoinManeuverState joinManeuverState_ = JoinManeuverState::IDLE;

    // Joiner-side target platoon data; valid during joiner states.
    std::unique_ptr<TargetPlatoonData> targetPlatoonData_;

    // Leader-side joiner data; valid during leader states.
    std::unique_ptr<JoinerData> joinerData_;

    // Join request retry timer. Scheduled only while the joiner is in J_WAIT_REPLY.
    omnetpp::cMessage* retryTimer_ = nullptr;

    // Number of JoinPlatoonRequest retries already sent after the initial request.
    int joinReqRetries_ = 0;

    // m; Plexe-style slack added to the final target gap for declaring “in position”.
    // Example: final target gap 5 m + slack 11 m => ACK threshold 16 m.
    static constexpr double kPlexeJoinInPositionSlackMeters_ = 11.0;

    /*
     * m; fake-CACC approach spacing used while the joiner moves into position.
     *
     * This is not the same as ACK slack. The joiner may control toward about
     * 15 m while the ACK threshold is targetGap + 11 m. Keeping these separate
     * prevents the joiner from stabilizing outside the threshold.
     */
    static constexpr double kPlexeApproachSpacingMeters_ = 15.0;

    // m/s; default extra speed used by the joiner while approaching the platoon.
    // 30 km/h converted to m/s.
    static constexpr double kApproachDeltaV_ = 30.0 / 3.6;

    // s; time between JoinPlatoonRequest retries when no response arrives.
    static constexpr double kJoinReqRetrySeconds_ = 0.5;

    // Maximum number of JoinPlatoonRequest attempts before aborting the maneuver.
    static constexpr int kJoinReqMaxTries_ = 20;
};

} // namespace carla