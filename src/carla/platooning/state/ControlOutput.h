#pragma once

#include "carla/platooning/state/ControlMode.h"

namespace carla {

/**
 * Output of the OMNeT++ longitudinal controller before BridgeApp forwards it.
 */
struct ControlOutput {
    double desiredAcceleration = 0.0;
    double desiredSpeed = 0.0;
    bool hasControl = true;
    ControlMode controlMode = ControlMode::HOLD;
};

} // namespace carla