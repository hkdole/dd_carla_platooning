#pragma once

#include "carla/platooning/controller/ControllerInputs.h"
#include "carla/platooning/state/ControlOutput.h"

namespace carla {

class LeaderCruiseController {
public:
    ControlOutput compute(const ControllerInputs& in, ControlMode outMode) const;
};

} // namespace carla
