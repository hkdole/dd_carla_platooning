#pragma once

#include <string>
#include <vector>

#include "veins/base/utils/Coord.h"

#include "carla/platooning/state/PlatoonTypes.h"

#include "plexe/messages/ManeuverMessage_m.h"
#include "plexe/messages/UpdatePlatoonFormation_m.h"
#include "plexe/messages/UpdatePlatoonData_m.h"

namespace omnetpp {
class cPacket;
}

namespace carla {

class CarlaPositionHelper;
class CarlaControllerAdapter;
class CarlaManeuver;

/**
 * Narrow interface exposed by the application layer to maneuver implementations.
 *
 * Maneuver code such as CarlaJoinAtBack should depend on this interface instead
 * of directly depending on CarlaGeneralPlatooningApp. This keeps protocol logic
 * separate from OMNeT++ module lifecycle, CARLA bridge logic, signal emission,
 * and controller execution.
 *
 * Typical flow:
 *   - CarlaGeneralPlatooningApp owns vehicle role, formation state, controller state,
 *     and V2V send helpers.
 *   - CarlaJoinManeuver / CarlaJoinAtBack use this interface to read or update only
 *     the state needed for maneuver protocol execution.
 *   - BridgeApp / CarlanetManager / pyCARLANeT remain outside this interface because
 *     maneuver code should not directly control CARLA or the simulation loop.
 */
class ICarlaPlatooningApp {
public:
    virtual ~ICarlaPlatooningApp() = default;

    // Current semantic role of this vehicle in the platooning scenario.
    // Role answers "what kind of vehicle is this?" and is separate from both
    // maneuver state and controller type.
    virtual const PlatoonRole& getPlatoonRole() const = 0;

    // Updates the semantic role of this vehicle.
    // Example: a joiner becomes FOLLOWER only after JoinFormation is accepted.
    virtual void setPlatoonRole(PlatoonRole r) = 0;

    // True while a maneuver object currently owns protocol state for this vehicle.
    // This prevents overlapping maneuvers from using the same app state at once.
    virtual bool isInManeuver() const = 0;

    // Marks the active maneuver owner.
    // maneuver is usually the object that should receive maneuver self-messages
    // and protocol callbacks until the maneuver completes or aborts.
    virtual void setInManeuver(bool b, CarlaManeuver* maneuver) = 0;

    // Access to Plexe-like platoon metadata:
    // vehicle id, external CARLA id, platoon id, leader id, lane, speed, and formation.
    virtual CarlaPositionHelper* getPositionHelper() = 0;

    // Access to controller-selection and temporary maneuver-control state.
    // This includes active controller, control mode, fake leader/front data,
    // fixed lane, approach spacing, and cruise override speed.
    virtual CarlaControllerAdapter* getControllerAdapter() = 0;

    // Minimum bumper gap at standstill for the selected controller.
    // Units: meters.
    virtual double getStandstillDistance(ActiveController controller) const = 0;

    // Time-headway parameter for the selected controller.
    // Units: seconds. Used by constant-time-spacing policies.
    virtual double getHeadway(ActiveController controller) const = 0;

    // Desired bumper gap under the app's current spacing policy.
    // Units: meters. For CDS this may ignore speed; for CTS it may depend on speed.
    virtual double getTargetDistance(double speed) const = 0;

    // Desired bumper gap for a specific controller under the app's spacing policy.
    // Units: meters. This overload exists so maneuver code can query the same
    // policy used by normal follower control.
    virtual double getTargetDistance(ActiveController controller, double speed) const = 0;

    // Controller that should be used after a vehicle becomes a normal platoon follower.
    // In this platform this is typically CACC.
    virtual ActiveController getTargetController() const = 0;

    // Current vehicle position from CARLA through CarlaInetMobility.
    // Units: meters in the transformed INET/Veins coordinate frame.
    virtual veins::Coord getCurrentPosition() const = 0;

    // Current lane metadata used in maneuver messages.
    // This is not direct CARLA steering; it is protocol state shared between vehicles.
    virtual int getCurrentLaneIndex() const = 0;

    // Sends a maneuver packet to a logical destination vehicle id.
    // The lower 802.11p frame may be broadcast; receivers filter by destinationId.
    virtual void sendUnicast(omnetpp::cPacket* msg, int destination) = 0;

    // Allow the ability to change leader and platoon desired speed
    virtual double getPlatoonDesiredSpeed() const = 0;
    virtual void setLeaderTargetSpeed(double speed) = 0;

    // Remove front vehicle for CACC
    virtual void clearFrontVehicle() = 0;

    // Fills fields shared by all maneuver messages.
    // Specific message factory functions add message-specific fields after this.
    virtual void fillManeuverMessage(
        ManeuverMessage* msg,
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId) = 0;

    // Builds a formation-update message for an existing platoon.
    // Used when the formation order changes but the platoon id remains the same.
    virtual UpdatePlatoonFormation* createUpdatePlatoonFormation(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& platoonFormation) = 0;

    // Builds a platoon-data update message.
    // Used when formation metadata and the platoon id may both change.
    virtual UpdatePlatoonData* createUpdatePlatoonData(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& platoonFormation,
        int newPlatoonId) = 0;
};

} // namespace carla