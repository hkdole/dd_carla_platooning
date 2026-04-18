#pragma once

#include <string>
#include <vector>

#include "veins/base/utils/Coord.h"

#include "carla/platooning/state/PlatoonTypes.h"

// generated OMNeT++ message headers from your local copied .msg files
#include "carla/platooning/messages/ManeuverMessage_m.h"
#include "carla/platooning/messages/UpdatePlatoonFormation_m.h"
#include "carla/platooning/messages/UpdatePlatoonData_m.h"

namespace omnetpp {
class cPacket;
}

namespace carla {

class CarlaPositionHelper;
class CarlaControllerAdapter;
class CarlaManeuver;

/**
 * Thin abstract interface that the adapted maneuver layer talks to.
 * This replaces the parts of GeneralPlatooningApp that
 * JoinManeuver / JoinAtBack logic needs.
 */
class ICarlaPlatooningApp {
public:
    virtual ~ICarlaPlatooningApp() = default;

    virtual const PlatoonRole& getPlatoonRole() const = 0;
    virtual void setPlatoonRole(PlatoonRole r) = 0;

    virtual bool isInManeuver() const = 0;
    virtual void setInManeuver(bool b, CarlaManeuver* maneuver) = 0;

    virtual CarlaPositionHelper* getPositionHelper() = 0;
    virtual CarlaControllerAdapter* getControllerAdapter() = 0;

    virtual double getStandstillDistance(ActiveController controller) const = 0;
    virtual double getHeadway(ActiveController controller) const = 0;
    virtual double getTargetDistance(double speed) const = 0;
    virtual double getTargetDistance(ActiveController controller, double speed) const = 0;
    virtual ActiveController getTargetController() const = 0;

    virtual veins::Coord getCurrentPosition() const = 0;
    virtual int getCurrentLaneIndex() const = 0;

    virtual void sendUnicast(omnetpp::cPacket* msg, int destination) = 0;

    virtual void fillManeuverMessage(
        ManeuverMessage* msg,
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId) = 0;

    virtual UpdatePlatoonFormation* createUpdatePlatoonFormation(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& platoonFormation) = 0;

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