#pragma once

namespace carla {

enum class ControlMode {
    HOLD = 0,
    LEADER_CRUISE,
    FOLLOWER_PLATOON,
    JOINER_FREE_CRUISE,
    JOINER_WAIT_REPLY,
    JOINER_WAIT_INFORMATION,
    JOINER_MOVE_IN_POSITION,
    JOINER_WAIT_JOIN
};

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
