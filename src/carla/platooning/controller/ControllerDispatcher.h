#pragma once

#include "carla/platooning/controller/FollowerCaccController.h"
#include "carla/platooning/controller/JoinerFakedCaccController.h"
#include "carla/platooning/controller/LeaderCruiseController.h"

namespace carla {

class ControllerDispatcher {
public:
    ControlOutput compute(const ControllerInputs& in) const;

private:
    LeaderCruiseController leaderCruiseController_;
    FollowerCaccController followerCaccController_;
    JoinerFakedCaccController joinerFakedCaccController_;
};

} // namespace carla
