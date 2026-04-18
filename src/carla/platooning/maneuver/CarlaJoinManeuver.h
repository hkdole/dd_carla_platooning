#pragma once

#include <string>
#include <vector>

#include "carla/platooning/maneuver/CarlaManeuver.h"

#include "carla/platooning/messages/JoinFormationAck_m.h"
#include "carla/platooning/messages/JoinFormation_m.h"
#include "carla/platooning/messages/JoinPlatoonRequest_m.h"
#include "carla/platooning/messages/JoinPlatoonResponse_m.h"
#include "carla/platooning/messages/MoveToPositionAck_m.h"
#include "carla/platooning/messages/MoveToPosition_m.h"
#include "carla/platooning/messages/UpdatePlatoonData_m.h"
#include "carla/platooning/messages/UpdatePlatoonFormation_m.h"

namespace carla {

struct JoinManeuverParameters {
    int platoonId = -1;
    int leaderId = -1;
    int position = -1;
};

/**
 * Adapted base class for join maneuvers.
 *
 * This is the CARLA-side equivalent of Plexe JoinManeuver.
 * It provides typed dispatch and typed packet constructors.
 */
class CarlaJoinManeuver : public CarlaManeuver {
public:
    explicit CarlaJoinManeuver(ICarlaPlatooningApp* app);
    ~CarlaJoinManeuver() override = default;

    void onManeuverMessage(const ManeuverMessage* mm) override;

protected:
    JoinPlatoonRequest* createJoinPlatoonRequest(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        int currentLaneIndex,
        double xPos,
        double yPos);

    JoinPlatoonResponse* createJoinPlatoonResponse(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        bool permitted);

    MoveToPosition* createMoveToPosition(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& newPlatoonFormation);

    MoveToPositionAck* createMoveToPositionAck(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& newPlatoonFormation);

    JoinFormation* createJoinFormation(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& newPlatoonFormation);

    JoinFormationAck* createJoinFormationAck(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& newPlatoonFormation);

    virtual void handleJoinPlatoonRequest(const JoinPlatoonRequest* msg) = 0;
    virtual void handleJoinPlatoonResponse(const JoinPlatoonResponse* msg) = 0;
    virtual void handleMoveToPosition(const MoveToPosition* msg) = 0;
    virtual void handleMoveToPositionAck(const MoveToPositionAck* msg) = 0;
    virtual void handleJoinFormation(const JoinFormation* msg) = 0;
    virtual void handleJoinFormationAck(const JoinFormationAck* msg) = 0;
};

} // namespace carla