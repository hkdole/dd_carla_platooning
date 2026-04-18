#pragma once

#include "carla/platooning/state/ControlMode.h"
#include "carla/platooning/state/PlatoonTypes.h"

namespace carla {

/**
 * Fake leader/front-vehicle data injected by the maneuver layer while the joiner
 * is not yet in normal steady-state platoon following.
 */
struct FakeVehicleData {
    double controllerAcceleration = 0.0;
    double actualAcceleration = 0.0;
    double speed = 0.0;
    double distance = -1.0; // only meaningful for front vehicle
    bool valid = false;
};

/**
 * CARLA-side replacement for the subset of plexeTraciVehicle controller hooks
 * that JoinAtBack needs.
 *
 * This class does not command CARLA directly.
 * It only updates OMNeT++ app/controller state so onControlTick() can compute
 * the proper per-vehicle command.
 */
class CarlaControllerAdapter {
public:
    void setFixedLane(int lane)
    {
        fixedLane_ = lane;
        hasFixedLane_ = true;
    }

    bool hasFixedLane() const { return hasFixedLane_; }
    int getFixedLane() const { return fixedLane_; }

    void clearFixedLane()
    {
        hasFixedLane_ = false;
        fixedLane_ = -1;
    }

    void setLeaderVehicleFakeData(double ctrlAccel, double accel, double speed)
    {
        leader_.controllerAcceleration = ctrlAccel;
        leader_.actualAcceleration = accel;
        leader_.speed = speed;
        leader_.valid = true;
    }

    void setFrontVehicleFakeData(double ctrlAccel, double accel, double speed, double distance)
    {
        front_.controllerAcceleration = ctrlAccel;
        front_.actualAcceleration = accel;
        front_.speed = speed;
        front_.distance = distance;
        front_.valid = true;
    }

    const FakeVehicleData& getLeaderVehicleFakeData() const { return leader_; }
    const FakeVehicleData& getFrontVehicleFakeData() const { return front_; }

    void clearLeaderVehicleFakeData() { leader_ = FakeVehicleData{}; }
    void clearFrontVehicleFakeData() { front_ = FakeVehicleData{}; }

    void setCruiseControlDesiredSpeed(double v) { cruiseDesiredSpeed_ = v; }
    double getCruiseControlDesiredSpeed() const { return cruiseDesiredSpeed_; }

    void setCACCConstantSpacing(double s) { caccConstantSpacing_ = s; }
    double getCACCConstantSpacing() const { return caccConstantSpacing_; }

    void setActiveController(ActiveController c)
    {
        activeController_ = c;

        switch (c) {
            case ActiveController::FAKED_CACC:
                controlMode_ = ControlMode::JOINER_MOVE_IN_POSITION;
                break;
            case ActiveController::CACC:
                controlMode_ = ControlMode::FOLLOWER_PLATOON;
                break;
            case ActiveController::CC:
                controlMode_ = ControlMode::LEADER_CRUISE;
                break;
            default:
                break;
        }
    }

    ActiveController getActiveController() const { return activeController_; }

    void setControlMode(ControlMode m) { controlMode_ = m; }
    ControlMode getControlMode() const { return controlMode_; }

private:
    bool hasFixedLane_ = false;
    int fixedLane_ = -1;

    FakeVehicleData leader_;
    FakeVehicleData front_;

    double cruiseDesiredSpeed_ = 0.0;
    double caccConstantSpacing_ = 5.0;

    ActiveController activeController_ = ActiveController::UNKNOWN;
    ControlMode controlMode_ = ControlMode::HOLD;
};

} // namespace carla