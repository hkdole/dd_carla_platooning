#pragma once

#include "carla/platooning/controller/ControllerInputs.h"
#include "carla/platooning/state/ControlOutput.h"

namespace carla {

class JoinerFakedCaccController {
public:
    ControlOutput compute(const ControllerInputs& in) const;
};

} // namespace carla
