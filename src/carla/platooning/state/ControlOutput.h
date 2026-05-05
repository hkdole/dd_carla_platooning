#pragma once

#include <algorithm>
#include <cmath>

#include "carla/platooning/state/ControlMode.h"

namespace carla {

/**
 * Output of the OMNeT++ longitudinal controller.
 *
 * This struct is produced by the controller dispatcher and then finalized by
 * CarlaGeneralPlatooningApp::fillTrackedLongitudinalActuation().
 *
 * Architecture rule:
 *   - OMNeT++ owns V2V logic, maneuver state, topology, controller acceleration,
 *     and final longitudinal actuator intent.
 *   - pyCARLANeT applies the actuator fields to CARLA and returns raw vehicle state.
 *   - pyCARLANeT should not decide platoon membership or run hidden platoon control.
 *
 * The desiredAcceleration field is the controller-level command.
 * The throttle/brake fields are the CARLA actuator-level command.
 */
struct ControlOutput {
    // Controller/export metadata.
    double desiredAcceleration = 0.0;  // m/s^2; acceleration requested by CC/ACC/CACC/etc.
    double desiredSpeed = 0.0;         // m/s; target/reference speed, useful for logging and bridge compatibility.
    bool hasControl = true;            // False means hold/stop instead of applying normal control.
    ControlMode controlMode = ControlMode::HOLD; // Semantic mode that produced this output.

    // Final longitudinal actuator command for CARLA.
    double throttle = 0.0;             // Normalized CARLA throttle command in [0, 1].
    double brake = 0.0;                // Normalized CARLA brake command in [0, 1].
    bool handBrake = false;            // CARLA hand brake flag; normally false during active control.
    bool reverse = false;              // CARLA reverse gear flag; normally false for platooning.
    bool manualGearShift = false;      // CARLA manual gear flag; normally false for automatic driving.
};

/**
 * Clamps a CARLA actuator command to a safe normalized range.
 *
 * Non-finite values are treated as the lower bound so NaN/Inf cannot reach
 * pyCARLANeT or CARLA.
 */
inline double clampActuator(double value, double lo = 0.0, double hi = 1.0)
{
    if (!std::isfinite(value)) return lo;
    return std::max(lo, std::min(hi, value));
}

} // namespace carla