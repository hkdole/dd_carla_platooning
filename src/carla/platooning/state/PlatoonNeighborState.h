#pragma once

#include <omnetpp.h>
#include "veins/base/utils/Coord.h"

namespace carla {

/**
 * Cached state for one neighboring vehicle, populated from PlatooningBeacon messages.
 *
 * This is the local V2V world model used by the OMNeT++ controller layer.
 * The controller should only use this data if valid == true and the age
 * simTime() - last is less than the configured max_age parameter.
 *
 * Source path:
 *   PlatooningBeacon.msg
 *     -> CarlaGeneralPlatooningApp::refreshNeighborFromBeacon()
 *     -> neighborsByVehicleId_[vehicleId]
 *     -> CarlaGeneralPlatooningApp::buildControllerInputs()
 */
struct PlatoonNeighborState {
    veins::Coord pos = veins::Coord(0, 0, 0);  // m; last advertised x/y/z position in simulation coordinates.
    veins::Coord vel = veins::Coord(0, 0, 0);  // m/s; last advertised velocity vector.

    double actualAcceleration = 0.0;      // m/s^2; measured longitudinal acceleration from the sender.
    double controllerAcceleration = 0.0;  // m/s^2; acceleration command computed by the sender's controller.
    double scalarSpeed = 0.0;             // m/s; speed magnitude, usually sqrt(vx^2 + vy^2).
    double length = 4.5;                  // m; sender vehicle length, used to estimate bumper-to-bumper gap.
    double angle = 0.0;                   // rad; sender heading/yaw if populated by the beacon.

    simtime_t last = SIMTIME_ZERO;        // OMNeT++ sim time when this beacon state was received.
    bool valid = false;                   // True after at least one usable beacon has been received.

    double desiredSpeed = 10.0;           // Defeault desired speed
};

} // namespace carla