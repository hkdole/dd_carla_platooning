#pragma once
 
#include "carla/platooning/state/ObservedVehicleState.h"
 
namespace carla {
 
/**
 * Local (no-messaging) lane-change maneuver executed by a JOINER before
 * handing off to the existing same-lane CarlaJoinAtBack maneuver (today),
 * or a future mid-platoon join maneuver (not yet implemented).
 *
 * This does not talk to CARLA or OMNeT directly. It only decides what
 * `referenceLateralOffsetM` to hand LateralController::computeSteering()
 * each tick, and reports state transitions for the app layer to react to.
 *
 * Supports crossing any number of lanes (not just adjacent), executed as a
 * sequence of single-lane hops rather than one large offset jump — see
 * MULTILANE_JOIN_DESIGN.md "Scope update: any lane, not just adjacent" for
 * why. Known limitations (no target-lane occupancy/coordination check,
 * opposite-direction-lane sign convention unvalidated) are also there.
 */
enum class LaneChangeState {
    IDLE,
    EVALUATING,      // per-hop: deciding whether the next single-lane hop is safe
    CHANGING_LANE,    // per-hop: ramping offset across one lane boundary
    SETTLING,         // per-hop: holding steady before starting the next hop (or finishing)
    LANE_CHANGE_COMPLETE,
    POST_COMPLETE_SETTLE,  // hold offset until centered
    ABORTED,
};
 
/**
 * Target of a lane-change-then-join sequence. successorActorId == -1 means
 * "join at the tail" (the only case actually implemented/used today, via
 * CarlaJoinAtBack). A future mid-platoon join maneuver would set both
 * fields and consume them once LANE_CHANGE_COMPLETE is reached — this
 * maneuver itself never inspects or acts on successorActorId, it only
 * carries it through for the app layer.
 */
struct JoinSlot {
    int predecessorActorId = -1;
    int successorActorId = -1;
};
 
class CarlaLaneChangeManeuver {
public:
    /**
     * Begin a lane change toward targetLaneId, for any |targetLaneId -
     * egoLaneId| >= 1 (not just adjacent). egoLaneId/targetLaneId use the
     * same CARLA lane-id convention as LateralRoutePoint::laneId /
     * ObservedVehicleState::currentLaneId.
     *
     * laneWidthM is read from NED (`lane_width_m`) by the caller and passed
     * in rather than hardcoded here, since it may vary by road segment.
     */
    void start(int egoLaneId, int targetLaneId, double laneWidthM, JoinSlot targetSlot);
 
    /** Advance the state machine by one control tick. Safe to call every
     *  tick while isActive() is true; no-ops when IDLE. */
    void tick(const ObservedVehicleState& state, double liveOffsetToHopLane);
 
    bool isActive() const
    {
        return state_ != LaneChangeState::IDLE &&
               state_ != LaneChangeState::LANE_CHANGE_COMPLETE &&
               state_ != LaneChangeState::ABORTED;
    }
 
    LaneChangeState state() const { return state_; }
 
    /** Value to pass as referenceLateralOffsetM this tick. */
    double currentOffsetM() const { return currentOffsetM_; }
    double completedOffsetM() const { return currentOffsetM_; }
 
    /** How many single-lane hops remain, including the one in progress.
     *  0 once LANE_CHANGE_COMPLETE. Exposed mainly for logging/telemetry. */
    int hopsRemaining() const { return hopsRemaining_; }
 
    /** Slot this lane change was performed in service of; caller uses this
     *  to know what to hand off (today: always to CarlaJoinAtBack via
     *  targetSlot().predecessorActorId) once LANE_CHANGE_COMPLETE is
     *  reached. */
    JoinSlot targetSlot() const { return targetSlot_; }

    int currentHopTargetLaneId() const { return egoLaneId_ + hopDirection_; }
 
    /** Reset to IDLE. Call after consuming LANE_CHANGE_COMPLETE/ABORTED. */
    void reset();
 
private:
    // TODO(you): validate this sign empirically in Town04 before trusting
    // it — CARLA lane_id sign convention is map/road-segment dependent.
    // See MULTILANE_JOIN_DESIGN.md "Determining target lane offset".
    // Returns the offset for ONE hop, using hopDirection_ (fixed for the
    // whole sequence at start()) for sign and laneWidthM for magnitude.
    // Not static: reads hopDirection_.
    double computeHopOffsetM(int egoLaneId, int targetLaneId, double laneWidthM) const;
 
    // TODO(you): wire an actual gap-safety check here using whatever
    // PlatoonNeighborState / neighbor-lookup the app already exposes.
    // Called before EACH hop, not just the first — a hazard that appears
    // partway through a multi-lane change can still abort it.
    // v1 conservatively returns true (see design doc "Safety gap check").
    bool isLaneChangeSafe(const ObservedVehicleState& state) const;
 
    LaneChangeState state_ = LaneChangeState::IDLE;
 
    int egoLaneId_ = -999;       // current lane at the start of the *current* hop
    int targetLaneId_ = -999;    // final destination lane (not the current hop's lane)
    int hopDirection_ = 0;       // +1 or -1, sign of travel toward targetLaneId_
    int hopsRemaining_ = 0;
 
    JoinSlot targetSlot_;
 
    double laneWidthM_ = 3.5;
    double hopTargetOffsetM_ = 0.0;   // offset target for the CURRENT hop only
    double currentOffsetM_ = 0.0;     // cumulative offset from the ORIGINAL lane's centerline
 
    // Ramp rate for currentOffsetM_ toward hopTargetOffsetM_, in m per tick.
    // Tune alongside LateralController's kLookaheadM — too fast a ramp
    // fights the same understeer/oscillation tradeoff discussed for
    // lookahead distance.
    double offsetRampMPerTick_ = 0.01;
 
    // SETTLING bookkeeping (per hop).
    int settledTickCount_ = 0;
    static constexpr int kRequiredSettledTicks = 60;
    static constexpr double kSettledLateralErrorM = 0.15;
    static constexpr int kPostCompleteSettleTicks = 40;
};
 
} // namespace carla