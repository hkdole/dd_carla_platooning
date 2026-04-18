#pragma once

#include <cstddef>

namespace carla {

/**
 * Logical role in the platoon / maneuver protocol.
 * Role is not the same as control mode.
 */
enum class PlatoonRole : std::size_t {
    NONE = 0,
    LEADER,
    FOLLOWER,
    JOINER
};

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
 * Controller family / active controller concept.
 * This mirrors the role played by Plexe controller selection,
 * but remains CARLA-stack friendly.
 */
enum class ActiveController : std::size_t {
    DRIVER = 0,
    CC,
    ACC,
    CACC,
    PLOEG,
    FAKED_CACC,
    UNKNOWN
};

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