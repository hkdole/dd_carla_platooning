#include "carla/platooning/controller/FollowerCaccController.h"

#include <algorithm>

namespace carla {

ControlOutput FollowerCaccController::compute(const ControllerInputs& in) const
{
    ControlOutput out;
    out.controlMode = ControlMode::FOLLOWER_PLATOON;
    out.hasControl = true;
    out.desiredSpeed = in.egoSpeed;
    out.desiredAcceleration = 0.0;

    if (!in.predecessor.valid) {
        if (in.leader.valid) {
            out.desiredSpeed = std::max(0.0, in.leader.speed);
            out.desiredAcceleration =
                in.kAccFF * in.leader.controllerAcceleration +
                0.15 * in.kSpeedP * (in.leader.speed - in.egoSpeed);
        }
        return out;
    }

    const double gap = in.predecessor.distance;
    const double targetGap = in.targetGap;
    const double eGap = gap - targetGap;
    const double dv = in.predecessor.speed - in.egoSpeed;
    const double ff = chooseFeedforwardAcceleration(in.predecessor);
    const double leaderTerm = in.leader.valid ? (0.05 * in.kLeaderDv * (in.leader.speed - in.egoSpeed)) : 0.0;
    const double speedBias = clampControllerValue(in.kGapSpeed * eGap, -in.maxClosureSpeed, in.maxClosureSpeed);

    out.desiredSpeed = std::max(0.0, in.predecessor.speed + speedBias);
    out.desiredAcceleration = in.kAccFF * ff + in.kGap * eGap + in.kDv * dv + leaderTerm;
    return out;
}

} // namespace carla
