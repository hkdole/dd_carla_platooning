#include "carla/platooning/controller/LeaderCruiseController.h"

namespace carla {

ControlOutput LeaderCruiseController::compute(const ControllerInputs& in, ControlMode outMode) const
{
    ControlOutput out;
    out.controlMode = outMode;
    out.hasControl = true;
    out.desiredSpeed = std::max(0.0, in.targetSpeed);
    out.desiredAcceleration = in.kSpeedP * (out.desiredSpeed - in.egoSpeed);
    return out;
}

} // namespace carla
