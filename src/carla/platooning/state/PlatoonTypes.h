#pragma once

#include <cstddef>

namespace carla {

/**
 * Logical role in the platoon or maneuver protocol.
 *
 * Role answers "what is this vehicle in the protocol?"
 * ControlMode answers "what is this vehicle currently doing?"
 * ActiveController answers "which control law should compute acceleration?"
 *
 * Example:
 *   A JOINER may temporarily use FAKED_CACC during JOINER_MOVE_IN_POSITION,
 *   then become a FOLLOWER using CACC after JoinFormation completes.
 */
enum class PlatoonRole : std::size_t {
    NONE = 0,    // Not currently assigned to a platoon role.
    LEADER,      // First vehicle in the platoon; usually owns cruise speed and formation authority.
    FOLLOWER,    // Normal platoon member behind another vehicle.
    JOINER       // Vehicle attempting to join the platoon.
};

/**
 * Converts a PlatoonRole to a stable log/debug string.
 * Keep this synchronized with the enum values above.
 */
inline const char* toString(PlatoonRole r)
{
    switch (r) {
        case PlatoonRole::NONE: return "NONE";
        case PlatoonRole::LEADER: return "LEADER";
        case PlatoonRole::FOLLOWER: return "FOLLOWER";
        case PlatoonRole::JOINER: return "JOINER";
        default: return "UNKNOWN";
    }
}

/**
 * Controller family currently assigned to the vehicle.
 *
 * This mirrors Plexe-style controller selection while staying independent
 * from SUMO/TraCI. The controller dispatcher maps these values to the
 * CARLA-compatible longitudinal controller implementation.
 */
enum class ActiveController : std::size_t {
    DRIVER = 0,     // Human/manual placeholder; normally not used in automated platoon runs.
    CC,             // Cruise Control; tracks a target speed only.
    ACC,            // Adaptive Cruise Control; tracks a front vehicle without cooperative feed-forward.
    CACC,           // Cooperative ACC; uses V2V predecessor/leader data and controller acceleration feed-forward.
    PLOEG,          // Ploeg-style platooning controller.
    FAKED_CACC,     // Temporary joiner approach mode using fake leader/front data before real follower membership.
    UNKNOWN         // Invalid/unset controller state.
};

/**
 * Converts an ActiveController to a stable log/debug string.
 * Keep this synchronized with the enum values above.
 */
inline const char* toString(ActiveController c)
{
    switch (c) {
        case ActiveController::DRIVER: return "DRIVER";
        case ActiveController::CC: return "CC";
        case ActiveController::ACC: return "ACC";
        case ActiveController::CACC: return "CACC";
        case ActiveController::PLOEG: return "PLOEG";
        case ActiveController::FAKED_CACC: return "FAKED_CACC";
        case ActiveController::UNKNOWN: return "UNKNOWN";
        default: return "UNKNOWN";
    }
}

} // namespace carla