#include "carla/platooning/controller/JoinerFakedCaccController.h"

#include <algorithm>

namespace carla {

ControlOutput JoinerFakedCaccController::compute(const ControllerInputs& in) const
{
    ControlOutput out;
    out.controlMode = in.controlMode;
    out.hasControl = true;
    out.desiredSpeed = std::max(0.0, in.targetSpeed);
    out.desiredAcceleration = 0.0;

    if (!in.fakeFront.valid) {
        out.desiredAcceleration = in.kSpeedP * (out.desiredSpeed - in.egoSpeed);
        return out;
    }

    const double gap = in.fakeFront.distance;
    const double targetGap = in.targetGap;
    const double eGap = gap - targetGap;
    const double dv = in.fakeFront.speed - in.egoSpeed;
    const double ff = chooseFeedforwardAcceleration(in.fakeFront);

    double closureBias = clampControllerValue(in.kGapSpeed * eGap, -in.maxClosureSpeed, in.maxClosureSpeed);
    double leaderTerm = 0.0;
    if (in.fakeLeader.valid) {
        leaderTerm =
            0.15 * in.kLeaderDv * (in.fakeLeader.speed - in.egoSpeed) +
            0.25 * in.kAccFF * in.fakeLeader.controllerAcceleration;
    }

    if (in.controlMode == ControlMode::JOINER_WAIT_JOIN) {
        closureBias = clampControllerValue(0.5 * in.kGapSpeed * eGap, -0.75, 0.75);
    }

    const double frontBasedTarget = in.fakeFront.speed + closureBias +
        ((in.controlMode == ControlMode::JOINER_MOVE_IN_POSITION) ? in.maxClosureSpeed : 0.0);

    out.desiredSpeed = std::max(0.0, std::min(in.targetSpeed, frontBasedTarget));
    out.desiredAcceleration =
        in.kAccFF * ff +
        in.kGap * eGap +
        in.kDv * dv +
        leaderTerm +
        0.25 * in.kSpeedP * (out.desiredSpeed - in.egoSpeed);

    return out;
}

} // namespace carla
