#pragma once

namespace carla {

/**
 * Semantic longitudinal-control mode for the vehicle.
 *
 * ControlMode describes what the vehicle is currently trying to do.
 * It is more specific than PlatoonRole and is used by
 * CarlaGeneralPlatooningApp::buildControllerInputs() to decide which
 * controller inputs are valid.
 *
 * Typical flow for a joiner:
 *   JOINER_FREE_CRUISE
 *     -> JOINER_WAIT_REPLY
 *     -> JOINER_WAIT_INFORMATION
 *     -> JOINER_MOVE_IN_POSITION
 *     -> JOINER_WAIT_JOIN
 *     -> FOLLOWER_PLATOON
 */
enum class ControlMode {
    HOLD = 0,                 // No normal longitudinal control; usually safe/default stopped state.
    LEADER_CRUISE,            // Leader uses cruise control toward leader_target_speed.
    FOLLOWER_PLATOON,         // Normal platoon follower; uses predecessor/leader beacon data.
    JOINER_FREE_CRUISE,       // Joiner exists outside the platoon and cruises independently.
    JOINER_WAIT_REPLY,        // Joiner has sent JoinPlatoonRequest and is waiting for JoinPlatoonResponse.
    JOINER_WAIT_INFORMATION,  // Joiner was accepted and is waiting for MoveToPosition details.
    JOINER_MOVE_IN_POSITION,  // Joiner uses fake-CACC/approach control to move behind the target front vehicle.
    JOINER_WAIT_JOIN          // Joiner sent MoveToPositionAck and waits for JoinFormation.
};

/**
 * Converts a ControlMode to a stable log/debug string.
 * Keep this synchronized with the enum values above.
 */
inline const char* toString(ControlMode m)
{
    switch (m) {
        case ControlMode::HOLD: return "HOLD";
        case ControlMode::LEADER_CRUISE: return "LEADER_CRUISE";
        case ControlMode::FOLLOWER_PLATOON: return "FOLLOWER_PLATOON";
        case ControlMode::JOINER_FREE_CRUISE: return "JOINER_FREE_CRUISE";
        case ControlMode::JOINER_WAIT_REPLY: return "JOINER_WAIT_REPLY";
        case ControlMode::JOINER_WAIT_INFORMATION: return "JOINER_WAIT_INFORMATION";
        case ControlMode::JOINER_MOVE_IN_POSITION: return "JOINER_MOVE_IN_POSITION";
        case ControlMode::JOINER_WAIT_JOIN: return "JOINER_WAIT_JOIN";
        default: return "UNKNOWN";
    }
}

} // namespace carla