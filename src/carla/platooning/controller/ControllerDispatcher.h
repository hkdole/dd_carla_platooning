#pragma once

#include "carla/platooning/controller/FollowerCaccController.h"
#include "carla/platooning/controller/JoinerFakedCaccController.h"
#include "carla/platooning/controller/LeaderCruiseController.h"

namespace carla {

// Dispatches a ControllerInputs bundle to the controller selected by ControlMode.
// ControlMode is semantic state: leader cruise, follower CACC, joiner approach, or hold.
class ControllerDispatcher {
public:
    ControlOutput compute(const ControllerInputs& in) const;

private:
    LeaderCruiseController leaderCruiseController_;
    FollowerCaccController followerCaccController_;
    JoinerFakedCaccController joinerFakedCaccController_;
};

} // namespace carla