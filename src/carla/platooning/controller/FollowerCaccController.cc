#include "carla/platooning/controller/FollowerCaccController.h"

#include <algorithm>
#include <cmath>
#include <omnetpp.h>

using namespace omnetpp;

namespace carla {

// Computes normal follower CACC output.
// The follower uses fresh predecessor/leader V2V state. desiredAcceleration is authoritative;
// desiredSpeed is exported only for logs, parser compatibility, and diagnostics.
ControlOutput FollowerCaccController::compute(const ControllerInputs& in) const
{
    ControlOutput out;
    out.controlMode = ControlMode::FOLLOWER_PLATOON;
    out.hasControl = true;
    out.desiredSpeed = std::max(0.0, finiteOr(in.egoSpeed, 0.0));
    out.desiredAcceleration = 0.0;

    const double egoSpeed = std::max(0.0, finiteOr(in.egoSpeed, 0.0));

    // No fresh predecessor means there is no valid vehicle-following input.
    // Fall back to CC rather than silently producing stale CACC.
    if (!in.predecessor.valid || in.predecessor.distance < 0.0) {
        if (in.leader.valid) {
            out.desiredSpeed = std::max(0.0, finiteOr(in.leader.speed, egoSpeed));
        }
        else {
            // Usually buildControllerInputs() sets targetSpeed to egoSpeed in this degraded case.
            out.desiredSpeed = std::max(0.0, finiteOr(in.targetSpeed, egoSpeed));
        }

        out.desiredAcceleration = computePlexeCcAcceleration(in, out.desiredSpeed);

        EV_INFO << "[FollowerCaccController][compute]"
                << " event=CACC_FALLBACK_CC"
                << " reason=no_valid_predecessor"
                << " egoSpeed=" << egoSpeed
                << " leaderValid=" << in.leader.valid
                << " leaderSpeed=" << (in.leader.valid ? in.leader.speed : -1.0)
                << " targetSpeed=" << in.targetSpeed
                << " desiredSpeed=" << out.desiredSpeed
                << " desiredAcceleration=" << out.desiredAcceleration
                << "\n";

        return out;
    }

    // For the first follower, predecessor and leader may be the same vehicle.
    // If the app layer only filled predecessor, use predecessor as leader-equivalent.
    const ControllerNeighborInput& leaderLike =
        in.leader.valid ? in.leader : in.predecessor;

    const double bumperGap =
        std::max(0.0, finiteOr(in.predecessor.distance, 0.0));

    const double spacing =
        std::max(0.0, finiteOr(in.targetGap, in.standstillDistance));

    PlexeCaccTrace trace = computePlexeCaccTrace(
        in,
        in.predecessor,
        leaderLike,
        bumperGap,
        spacing);

    out.desiredAcceleration = trace.clippedAcceleration;

    // Reference/export speed only.
    // pyCARLANeT must not use this as a hidden speed controller.
    out.desiredSpeed = computeExportSpeedFromFrontGap(
        in,
        in.predecessor,
        bumperGap,
        spacing);

    EV_INFO << "[FollowerCaccController][compute]"
            << " event=CACC_TRACE"
            << " egoSpeed=" << trace.egoSpeed
            << " predSpeed=" << trace.predSpeed
            << " leaderSpeed=" << trace.leaderSpeed
            << " bumperGap=" << trace.bumperGap
            << " spacing=" << trace.spacing
            << " gapError=" << (trace.bumperGap - trace.spacing)
            << " epsilon=" << trace.epsilon
            << " epsilonDot=" << trace.epsilonDot
            << " leaderClosingRate=" << trace.leaderClosingRate
            << " predAccelerationFF=" << trace.predAccelerationFF
            << " leaderAccelerationFF=" << trace.leaderAccelerationFF
            << " alpha1=" << trace.alpha1
            << " alpha2=" << trace.alpha2
            << " alpha3=" << trace.alpha3
            << " alpha4=" << trace.alpha4
            << " alpha5=" << trace.alpha5
            << " termPredAccel=" << trace.termPredAccel
            << " termLeaderAccel=" << trace.termLeaderAccel
            << " termPredSpeed=" << trace.termPredSpeed
            << " termLeaderSpeed=" << trace.termLeaderSpeed
            << " termGap=" << trace.termGap
            << " rawAcceleration=" << trace.rawAcceleration
            << " clippedAcceleration=" << trace.clippedAcceleration
            << " exportedDesiredSpeed=" << out.desiredSpeed
            << " predecessorValid=" << in.predecessor.valid
            << " leaderValid=" << in.leader.valid
            << " useControllerAcceleration=" << in.useControllerAcceleration
            << "\n";

    return out;
}

} // namespace carla