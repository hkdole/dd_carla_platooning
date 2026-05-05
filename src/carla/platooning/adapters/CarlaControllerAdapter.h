#pragma once

#include "carla/platooning/state/ControlMode.h"
#include "carla/platooning/state/PlatoonTypes.h"

namespace carla {

/**
 * Temporary vehicle state injected by the maneuver layer.
 *
 * Join-at-back uses this while a joiner is approaching the platoon but has not
 * yet become a normal follower. During that phase, the joiner may need to track
 * a logical leader/front vehicle before its final predecessor relationship is
 * committed in the formation.
 *
 * This data is consumed by CarlaGeneralPlatooningApp::buildControllerInputs().
 * It must be cleared when the maneuver aborts or when the joiner becomes a real
 * follower; otherwise stale approach-phase distances can contaminate steady CACC.
 */
struct FakeVehicleData {
    double controllerAcceleration = 0.0; // m/s^2; acceleration command advertised by the other vehicle.
    double actualAcceleration = 0.0;     // m/s^2; measured acceleration advertised by the other vehicle.
    double speed = 0.0;                  // m/s; scalar speed of the fake leader/front vehicle.
    double distance = -1.0;              // m; bumper gap to fake front vehicle; meaningful only for front_.
    bool valid = false;                  // true only after maneuver code has populated this struct.
};

/**
 * Small adapter that replaces the subset of Plexe/TraCI vehicle-controller hooks
 * needed by the CARLA join-at-back implementation.
 *
 * This class does not send commands to CARLA and does not implement a controller.
 * It stores controller metadata that is later read by
 * CarlaGeneralPlatooningApp::buildControllerInputs() and executed by the
 * ControllerDispatcher/onControlTick path.
 *
 * The main distinction is:
 *   - activeController_: which controller family should be used, e.g., CC/CACC
 *   - controlMode_: what semantic maneuver/control phase the vehicle is in
 *   - fake leader/front data: temporary join-approach state only
 */
class CarlaControllerAdapter {
public:
    /**
     * Locks the vehicle to a logical platoon lane.
     *
     * This is maneuver metadata used by the OMNeT++ protocol. It does not force
     * CARLA lane following by itself.
     */
    void setFixedLane(int lane)
    {
        fixedLane_ = lane;
        hasFixedLane_ = true;
    }

    /** Returns true if a maneuver has explicitly set a logical lane. */
    bool hasFixedLane() const { return hasFixedLane_; }

    /** Returns the fixed logical lane id; valid only when hasFixedLane() is true. */
    int getFixedLane() const { return fixedLane_; }

    /** Clears the maneuver-imposed lane metadata. */
    void clearFixedLane()
    {
        hasFixedLane_ = false;
        fixedLane_ = -1;
    }

    /**
     * Stores temporary fake-leader state for a joiner.
     *
     * The joiner uses this during JOINER_MOVE_IN_POSITION/JOINER_WAIT_JOIN so
     * the controller can receive leader-like input before normal follower state
     * is available from the committed formation.
     */
    void setLeaderVehicleFakeData(double ctrlAccel, double accel, double speed)
    {
        leader_.controllerAcceleration = ctrlAccel;
        leader_.actualAcceleration = accel;
        leader_.speed = speed;
        leader_.valid = true;
    }

    /**
     * Stores temporary fake-front-vehicle state for a joiner.
     *
     * distance is the current estimated bumper gap to the vehicle the joiner is
     * approaching. This value is updated from V2V beacons during the maneuver.
     */
    void setFrontVehicleFakeData(double ctrlAccel, double accel, double speed, double distance)
    {
        front_.controllerAcceleration = ctrlAccel;
        front_.actualAcceleration = accel;
        front_.speed = speed;
        front_.distance = distance;
        front_.valid = true;
    }

    /** Returns temporary fake-leader state for controller input construction. */
    const FakeVehicleData& getLeaderVehicleFakeData() const { return leader_; }

    /** Returns temporary fake-front state for controller input construction. */
    const FakeVehicleData& getFrontVehicleFakeData() const { return front_; }

    /** Clears temporary fake-leader data after join completion or abort. */
    void clearLeaderVehicleFakeData() { leader_ = FakeVehicleData{}; }

    /** Clears temporary fake-front data after join completion or abort. */
    void clearFrontVehicleFakeData() { front_ = FakeVehicleData{}; }

    /** Sets the maneuver-provided cruise target speed. */
    void setCruiseControlDesiredSpeed(double v) { cruiseDesiredSpeed_ = v; } // m/s.

    /** Returns the current cruise target speed override, or 0 if none is set. */
    double getCruiseControlDesiredSpeed() const { return cruiseDesiredSpeed_; }

    /** Sets constant-spacing target used by CACC/faked-CACC modes. */
    void setCACCConstantSpacing(double s) { caccConstantSpacing_ = s; } // m.

    /** Returns the constant-spacing target used by controller input construction. */
    double getCACCConstantSpacing() const { return caccConstantSpacing_; }

    /**
     * Sets the active controller family and applies a default semantic mode.
     *
     * Some callers immediately override controlMode_ afterward. That is valid:
     * controller family and semantic maneuver phase are related but not identical.
     */
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

    /** Returns the currently selected controller family. */
    ActiveController getActiveController() const { return activeController_; }

    /** Sets the semantic control/maneuver phase without changing controller family. */
    void setControlMode(ControlMode m) { controlMode_ = m; }

    /** Returns the semantic control/maneuver phase. */
    ControlMode getControlMode() const { return controlMode_; }

private:
    bool hasFixedLane_ = false; // true when fixedLane_ contains maneuver-provided lane metadata.
    int fixedLane_ = -1;        // logical lane id; -1 means unset.

    FakeVehicleData leader_; // temporary fake leader data for join approach.
    FakeVehicleData front_;  // temporary fake front/predecessor data for join approach.

    double cruiseDesiredSpeed_ = 0.0;   // m/s; 0 means no explicit cruise override.
    double caccConstantSpacing_ = 5.0;  // m; default constant bumper-gap target.

    ActiveController activeController_ = ActiveController::UNKNOWN; // selected controller family.
    ControlMode controlMode_ = ControlMode::HOLD;                   // current semantic control phase.
};

} // namespace carla
