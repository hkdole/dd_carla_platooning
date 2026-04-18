#pragma once

#include <algorithm>

#include "carla/platooning/state/ControlMode.h"
#include "carla/platooning/state/PlatoonTypes.h"

namespace carla {

struct ControllerNeighborInput {
    bool valid = false;
    double speed = 0.0;
    double actualAcceleration = 0.0;
    double controllerAcceleration = 0.0;
    double distance = -1.0;
};

struct ControllerInputs {
    ControlMode controlMode = ControlMode::HOLD;
    ActiveController activeController = ActiveController::UNKNOWN;

    double egoSpeed = 0.0;
    double targetSpeed = 0.0;
    double targetGap = 5.0;
    double standstillDistance = 5.0;
    double headway = 0.5;

    double approachDeltaV = 0.0;
    double maxClosureSpeed = 1.0;

    double kGap = 0.2;
    double kDv = 0.7;
    double kSpeedP = 0.7;
    double kAccFF = 1.0;
    double kLeaderDv = 0.05;
    double kGapSpeed = 0.03;

    double aMin = -4.0;
    double aMax = 2.5;

    ControllerNeighborInput predecessor;
    ControllerNeighborInput leader;
    ControllerNeighborInput fakeFront;
    ControllerNeighborInput fakeLeader;
};

inline double clampControllerValue(double value, double lo, double hi)
{
    return std::max(lo, std::min(hi, value));
}

inline double chooseFeedforwardAcceleration(const ControllerNeighborInput& n)
{
    return n.valid ? n.controllerAcceleration : 0.0;
}

} // namespace carla
