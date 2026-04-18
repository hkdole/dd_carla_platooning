#include "carla/platooning/controller/ControllerDispatcher.h"

namespace carla {

namespace {

ControlOutput buildHold(ControlMode mode, double speed = 0.0, bool hasControl = false)
{
    ControlOutput out;
    out.controlMode = mode;
    out.hasControl = hasControl;
    out.desiredSpeed = speed;
    out.desiredAcceleration = 0.0;
    return out;
}

} // namespace

ControlOutput ControllerDispatcher::compute(const ControllerInputs& in) const
{
    switch (in.controlMode) {
        case ControlMode::LEADER_CRUISE:
            return leaderCruiseController_.compute(in, ControlMode::LEADER_CRUISE);

        case ControlMode::FOLLOWER_PLATOON:
            return followerCaccController_.compute(in);

        case ControlMode::JOINER_FREE_CRUISE:
        case ControlMode::JOINER_WAIT_REPLY:
        case ControlMode::JOINER_WAIT_INFORMATION:
            return leaderCruiseController_.compute(in, in.controlMode);

        case ControlMode::JOINER_MOVE_IN_POSITION:
        case ControlMode::JOINER_WAIT_JOIN:
            return joinerFakedCaccController_.compute(in);

        case ControlMode::HOLD:
        default:
            return buildHold(in.controlMode, 0.0, false);
    }
}

} // namespace carla
