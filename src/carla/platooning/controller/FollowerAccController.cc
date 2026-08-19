#include "carla/platooning/controller/FollowerAccController.h"
#include <algorithm>

namespace carla {

ControlOutput FollowerAccController::compute(const ControllerInputs& in) const
{
    ControlOutput out;
    out.controlMode  = ControlMode::FOLLOWER_PLATOON;
    out.hasControl   = true;
    out.desiredSpeed = std::max(0.0, finiteOr(in.egoSpeed, 0.0));

    out.desiredAcceleration = computePlexeAccAcceleration(
        in,
        in.predecessor,
        in.predecessor.distance,
        in.targetGap);

    return out;
}

} // namespace carla