#include "carla/platooning/maneuver/CarlaLaneChangeManeuver.h"

#include <algorithm>
#include <cmath>

namespace carla {

void CarlaLaneChangeManeuver::start(
    int egoLaneId,
    int targetLaneId,
    double laneWidthM,
    JoinSlot targetSlot)
{
    egoLaneId_ = egoLaneId;
    targetLaneId_ = targetLaneId;
    laneWidthM_ = laneWidthM;
    targetSlot_ = targetSlot;

    const int laneDelta = targetLaneId_ - egoLaneId_;
    hopDirection_ = (laneDelta > 0) ? 1 : ((laneDelta < 0) ? -1 : 0);
    hopsRemaining_ = std::abs(laneDelta);

    currentOffsetM_ = 0.0;
    hopTargetOffsetM_ = 0.0;
    settledTickCount_ = 0;

    if (hopsRemaining_ == 0) {
        // Already in the target lane — nothing to do. Caller shouldn't
        // normally hit this (same-lane joins go straight to
        // CarlaJoinAtBack), but handle it defensively rather than
        // asserting.
        state_ = LaneChangeState::LANE_CHANGE_COMPLETE;
        return;
    }

    state_ = LaneChangeState::EVALUATING;
}

void CarlaLaneChangeManeuver::tick(const ObservedVehicleState& state, double liveOffsetToHopLane)
{
    switch (state_) {
        case LaneChangeState::IDLE:
            return;

        case LaneChangeState::EVALUATING: {
            // Re-checked before every hop, not just the first — see
            // isLaneChangeSafe() TODO for what's actually checked today
            // (nothing yet; always true).
            if (!isLaneChangeSafe(state)) {
                state_ = LaneChangeState::ABORTED;
                return;
            }
            // computeHopOffsetM uses hopDirection_, which is fixed for the
            // whole multi-hop sequence at start() — each hop moves one
            // more lane-width in that same direction.
            hopTargetOffsetM_ = currentOffsetM_ +
                computeHopOffsetM(egoLaneId_, targetLaneId_, laneWidthM_);
            state_ = LaneChangeState::CHANGING_LANE;
            return;
        }

        case LaneChangeState::CHANGING_LANE: {
            const double remaining = hopTargetOffsetM_ - currentOffsetM_;
            const double step = std::clamp(
                remaining,
                -offsetRampMPerTick_,
                offsetRampMPerTick_);
            currentOffsetM_ += step;

            if (std::abs(hopTargetOffsetM_ - currentOffsetM_) < kSettledLateralErrorM) {
                currentOffsetM_ = hopTargetOffsetM_;
                state_ = LaneChangeState::SETTLING;
                settledTickCount_ = 0;
            }
            return;
        }

        case LaneChangeState::SETTLING: {
            // Re-derive actual lateral error from the live observed state
            // once routeLateralOffsetM is populated (see project state:
            // "Route Progress" pending item). Until then this only checks
            // that we've held the commanded offset steady for long enough,
            // not that the vehicle has physically converged onto it.
            //
            // TODO(you): once ObservedVehicleState::routeLateralOffsetM is
            // populated by buildObservedVehicleState(), replace this with
            // a real convergence check, e.g.:
            //   if (std::abs(state.routeLateralOffsetM - hopTargetOffsetM_)
            //       < kSettledLateralErrorM) { ++settledTickCount_; }
            //   else { settledTickCount_ = 0; }
            ++settledTickCount_;

            if (settledTickCount_ < kRequiredSettledTicks) {
                return;
            }

            // This hop is done. Either move on to the next hop or finish.
            --hopsRemaining_;
            settledTickCount_ = 0;

            if (hopsRemaining_ <= 0) {
                state_ = LaneChangeState::POST_COMPLETE_SETTLE;
                settledTickCount_ = 0;
            } else {
                // egoLaneId_ advances by one hop so the next EVALUATING
                // pass computes the correct next hop offset/direction.
                egoLaneId_ += hopDirection_;
                state_ = LaneChangeState::EVALUATING;
            }
            return;
        }

        case LaneChangeState::POST_COMPLETE_SETTLE: {
        // Hold currentOffsetM_ steady and wait for the vehicle to
        // physically arrive in the target lane before declaring done.
        // TODO: replace tick count with routeLateralOffsetM check
        // once ObservedVehicleState::routeLateralOffsetM is populated.
        if (std::abs(liveOffsetToHopLane) < kSettledLateralErrorM) {
            state_ = LaneChangeState::LANE_CHANGE_COMPLETE;
        } else {
            ++settledTickCount_;
            if (settledTickCount_ >= kPostCompleteSettleTicks) {
                state_ = LaneChangeState::LANE_CHANGE_COMPLETE;
            }
        }
        return;
    }

        case LaneChangeState::LANE_CHANGE_COMPLETE:
        case LaneChangeState::ABORTED:
            // Terminal; caller must call reset() before starting again.
            return;
    }
}

void CarlaLaneChangeManeuver::reset()
{
    state_ = LaneChangeState::IDLE;
    egoLaneId_ = -999;
    targetLaneId_ = -999;
    hopDirection_ = 0;
    hopsRemaining_ = 0;
    targetSlot_ = JoinSlot{};
    currentOffsetM_ = 0.0;
    hopTargetOffsetM_ = 0.0;
    settledTickCount_ = 0;
}

double CarlaLaneChangeManeuver::computeHopOffsetM(
    int /*egoLaneId*/,
    int /*targetLaneId*/,
    double laneWidthM) const
{
    return -static_cast<double>(hopDirection_) * laneWidthM;
}

bool CarlaLaneChangeManeuver::isLaneChangeSafe(const ObservedVehicleState& /*state*/) const
{
    // v1: conservatively always true. See MULTILANE_JOIN_DESIGN.md
    // "Safety gap check" for what's missing here — no target-lane
    // occupancy check against non-platoon traffic yet, and no gap check
    // against the candidate's own predecessor beyond whatever the caller
    // already validated when evaluatePlatoonCandidates() picked this
    // candidate.
    //
    // TODO(you): wire a real check once neighbor lane-id is available
    // (see design doc prerequisite on the beacon message).
    return true;
}

} // namespace carla