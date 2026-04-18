#include "carla/platooning/maneuver/CarlaManeuver.h"

namespace carla {

CarlaManeuver::CarlaManeuver(ICarlaPlatooningApp* app)
    : app_(app)
{
    if (!app_) {
        throw omnetpp::cRuntimeError("CarlaManeuver constructed with null app");
    }

    positionHelper_ = app_->getPositionHelper();
    controllerAdapter_ = app_->getControllerAdapter();

    if (!positionHelper_) {
        throw omnetpp::cRuntimeError("CarlaManeuver: app returned null CarlaPositionHelper");
    }
    if (!controllerAdapter_) {
        throw omnetpp::cRuntimeError("CarlaManeuver: app returned null CarlaControllerAdapter");
    }
}

bool CarlaManeuver::handleSelfMsg(omnetpp::cMessage* msg)
{
    (void)msg;
    return false;
}

} // namespace carla