#pragma once

#include <omnetpp.h>
#include "veins/base/utils/Coord.h"

namespace carla {

/**
 * Neighbor state cached from platooning beacons.
 * This is the local OMNeT++-side view of a nearby platoon member.
 */
struct PlatoonNeighborState {
    veins::Coord pos = veins::Coord(0, 0, 0);
    veins::Coord vel = veins::Coord(0, 0, 0);

    double actualAcceleration = 0.0;
    double controllerAcceleration = 0.0;
    double scalarSpeed = 0.0;
    double length = 4.5;
    double angle = 0.0;

    simtime_t last = SIMTIME_ZERO;
    bool valid = false;
};

} // namespace carla
