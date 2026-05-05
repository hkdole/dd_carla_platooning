#pragma once

#include <omnetpp.h>

#include "carla/platooning/adapters/ICarlaPlatooningApp.h"
#include "carla/platooning/adapters/CarlaPositionHelper.h"
#include "carla/platooning/adapters/CarlaControllerAdapter.h"

#include "plexe/messages/PlatooningBeacon_m.h"
#include "plexe/messages/ManeuverMessage_m.h"

namespace carla {

/**
 * Base class for adapted platoon maneuvers.
 *
 * This is the CARLA-side equivalent of Plexe's Maneuver abstraction,
 * but it talks to ICarlaPlatooningApp instead of GeneralPlatooningApp.
 */
class CarlaManeuver {
public:
    explicit CarlaManeuver(ICarlaPlatooningApp* app);
    virtual ~CarlaManeuver() = default;

    /**
     * Start maneuver with maneuver-specific parameter struct.
     */
    virtual void startManeuver(const void* parameters) = 0;

    /**
     * Abort maneuver if needed.
     */
    virtual void abortManeuver() = 0;

    /**
     * Handle platooning beacon. Maneuver does not own/free the packet.
     */
    virtual void onPlatoonBeacon(const PlatooningBeacon* pb) = 0;

    /**
     * Handle typed maneuver message. Maneuver does not own/free the packet.
     */
    virtual void onManeuverMessage(const ManeuverMessage* mm) = 0;

    /**
     * Handle failed unicast send if app chooses to surface it.
     */
    virtual void onFailedTransmissionAttempt(const ManeuverMessage* mm) = 0;

    /**
     * Non-module maneuvers may still need timers. Return true if consumed.
     */
    virtual bool handleSelfMsg(omnetpp::cMessage* msg);

protected:
    ICarlaPlatooningApp* app_ = nullptr;
    CarlaPositionHelper* positionHelper_ = nullptr;
    CarlaControllerAdapter* controllerAdapter_ = nullptr;
};

} // namespace carla