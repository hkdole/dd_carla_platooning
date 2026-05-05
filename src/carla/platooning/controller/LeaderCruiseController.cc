#include "carla/platooning/controller/LeaderCruiseController.h"

#include <algorithm>

namespace carla {

// Computes leader cruise-control output.
// The leader is the only vehicle that should directly consume an external cruise/speed target.
ControlOutput LeaderCruiseController::compute(const ControllerInputs& in, ControlMode outMode) const
{
    ControlOutput out;
    out.controlMode = outMode;
    out.hasControl = true;

    // Desired speed is a nonnegative cruise/reference target.
    out.desiredSpeed = std::max(0.0, finiteOr(in.targetSpeed, 0.0));

    // Desired acceleration is the authoritative output used downstream.
    out.desiredAcceleration = computePlexeCcAcceleration(in, out.desiredSpeed);

    return out;
}

} // namespace carla