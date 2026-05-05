#include "carla/platooning/controller/JoinerFakedCaccController.h"

#include <algorithm>
#include <cmath>
#include <omnetpp.h>

using namespace omnetpp;

namespace carla {

// Computes joiner approach control using maneuver-injected fake front/leader data.
// This is used before the joiner becomes a normal follower. Once JoinFormation completes,
// the vehicle should switch to FOLLOWER_PLATOON and use real predecessor/leader beacons.
ControlOutput JoinerFakedCaccController::compute(const ControllerInputs& in) const
{
    ControlOutput out;
    out.controlMode = in.controlMode;
    out.hasControl = true;
    out.desiredSpeed = std::max(0.0, finiteOr(in.targetSpeed, 0.0));
    out.desiredAcceleration = 0.0;

    // Without fake front data, there is no join-specific following target.
    // Treat the joiner as a CC vehicle until maneuver/beacon data arrives.
    if (!in.fakeFront.valid || in.fakeFront.distance < 0.0) {
        out.desiredAcceleration = computePlexeCcAcceleration(in, out.desiredSpeed);

        EV_INFO << "[JoinerFakedCaccController][compute]"
                << " event=FAKED_CACC_FALLBACK_CC"
                << " reason=no_valid_fake_front"
                << " egoSpeed=" << in.egoSpeed
                << " targetSpeed=" << out.desiredSpeed
                << " desiredAcceleration=" << out.desiredAcceleration
                << "\n";

        return out;
    }

    // Plexe FAKED_CACC receives front/leader data from the maneuver layer.
    // If fakeLeader is missing, fakeFront is used as a leader-equivalent to keep the law defined.
    const ControllerNeighborInput& leaderLike =
        in.fakeLeader.valid ? in.fakeLeader : in.fakeFront;

    const double bumperGap =
        std::max(0.0, finiteOr(in.fakeFront.distance, 0.0));

    const double spacing =
        std::max(0.0, finiteOr(in.targetGap, in.standstillDistance));

    PlexeCaccTrace trace = computePlexeCaccTrace(
        in,
        in.fakeFront,
        leaderLike,
        bumperGap,
        spacing);

    // Export/reference speed only.
    // During MOVE_IN_POSITION, add approachDeltaV for catch-up behavior but still cap by targetSpeed.
    double exportSpeed = computeExportSpeedFromFrontGap(
        in,
        in.fakeFront,
        bumperGap,
        spacing);

    if (in.controlMode == ControlMode::JOINER_MOVE_IN_POSITION) {
        exportSpeed += std::max(0.0, finiteOr(in.approachDeltaV, 0.0));
    }

    out.desiredSpeed =
        std::max(0.0, std::min(std::max(0.0, finiteOr(in.targetSpeed, 0.0)), exportSpeed));

    // Plexe MSCFModel_CC::FAKED_CACC behavior:
    //
    //   ccAcceleration   = _cc(egoSpeed, ccDesiredSpeed)
    //   caccAcceleration = _cacc(fakeFront, fakeLeader)
    //   command          = min(ccAcceleration, caccAcceleration)
    //
    // The min() keeps the joiner from exceeding the intended maneuver/cruise speed
    // while still letting spacing dynamics command braking when needed.
    const double ccAcceleration =
        computePlexeCcAcceleration(in, std::max(0.0, finiteOr(in.targetSpeed, 0.0)));

    out.desiredAcceleration = clampControllerValue(
        std::min(ccAcceleration, trace.clippedAcceleration),
        in.aMin,
        in.aMax);

    EV_INFO << "[JoinerFakedCaccController][compute]"
            << " event=FAKED_CACC_TRACE"
            << " mode=" << static_cast<int>(in.controlMode)
            << " egoSpeed=" << trace.egoSpeed
            << " fakeFrontSpeed=" << trace.predSpeed
            << " fakeLeaderSpeed=" << trace.leaderSpeed
            << " bumperGap=" << trace.bumperGap
            << " spacing=" << trace.spacing
            << " gapError=" << (trace.bumperGap - trace.spacing)
            << " epsilon=" << trace.epsilon
            << " epsilonDot=" << trace.epsilonDot
            << " leaderClosingRate=" << trace.leaderClosingRate
            << " fakeFrontAccelerationFF=" << trace.predAccelerationFF
            << " fakeLeaderAccelerationFF=" << trace.leaderAccelerationFF
            << " rawCaccAcceleration=" << trace.rawAcceleration
            << " clippedCaccAcceleration=" << trace.clippedAcceleration
            << " ccAcceleration=" << ccAcceleration
            << " finalAcceleration=" << out.desiredAcceleration
            << " targetSpeed=" << in.targetSpeed
            << " exportedDesiredSpeed=" << out.desiredSpeed
            << " useControllerAcceleration=" << in.useControllerAcceleration
            << "\n";

    return out;
}

} // namespace carla