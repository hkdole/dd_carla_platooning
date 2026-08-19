#pragma once

#include <string>
#include <vector>

#include "veins/base/utils/Coord.h"

namespace carla {

struct LateralRoutePoint {
    double xM = 0.0;
    double yM = 0.0;
    double yawRad = 0.0;
    int roadId = -1;
    int laneId = 0;
    int routeIndex = -1;
};

/** Read-only CARLA state and physical metadata consumed by OMNeT++ policy. */
struct ObservedVehicleState {
    veins::Coord positionM;
    veins::Coord velocityMps;
    veins::Coord accelerationMps2;
    double yawRad = 0.0;
    double speedMps = 0.0;
    std::string routeId;
    bool routeProjectionValid = false;
    double routeProgressM = 0.0;
    double routeLateralOffsetM = 0.0;
    double routeHeadingErrorRad = 0.0;
    double wheelbaseM = 0.0;
    double maximumFrontWheelSteeringRad = 0.0;
    std::vector<LateralRoutePoint> routePolyline;
};

} // namespace carla
