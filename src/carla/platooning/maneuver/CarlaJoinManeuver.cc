#include "carla/platooning/maneuver/CarlaJoinManeuver.h"

namespace carla {

CarlaJoinManeuver::CarlaJoinManeuver(ICarlaPlatooningApp* app)
    : CarlaManeuver(app)
{
}

void CarlaJoinManeuver::onManeuverMessage(const ManeuverMessage* mm)
{
    if (const auto* msg = dynamic_cast<const JoinPlatoonRequest*>(mm)) {
        handleJoinPlatoonRequest(msg);
    }
    else if (const auto* msg = dynamic_cast<const JoinPlatoonResponse*>(mm)) {
        handleJoinPlatoonResponse(msg);
    }
    else if (const auto* msg = dynamic_cast<const MoveToPosition*>(mm)) {
        handleMoveToPosition(msg);
    }
    else if (const auto* msg = dynamic_cast<const MoveToPositionAck*>(mm)) {
        handleMoveToPositionAck(msg);
    }
    else if (const auto* msg = dynamic_cast<const JoinFormation*>(mm)) {
        handleJoinFormation(msg);
    }
    else if (const auto* msg = dynamic_cast<const JoinFormationAck*>(mm)) {
        handleJoinFormationAck(msg);
    }
}

JoinPlatoonRequest* CarlaJoinManeuver::createJoinPlatoonRequest(
    int vehicleId,
    const std::string& externalId,
    int platoonId,
    int destinationId,
    int currentLaneIndex,
    double xPos,
    double yPos)
{
    auto* msg = new JoinPlatoonRequest("JoinPlatoonRequest");
    app_->fillManeuverMessage(msg, vehicleId, externalId, platoonId, destinationId);
    msg->setCurrentLaneIndex(currentLaneIndex);
    msg->setXPos(xPos);
    msg->setYPos(yPos);
    return msg;
}

JoinPlatoonResponse* CarlaJoinManeuver::createJoinPlatoonResponse(
    int vehicleId,
    const std::string& externalId,
    int platoonId,
    int destinationId,
    bool permitted)
{
    auto* msg = new JoinPlatoonResponse("JoinPlatoonResponse");
    app_->fillManeuverMessage(msg, vehicleId, externalId, platoonId, destinationId);
    msg->setPermitted(permitted);
    return msg;
}

MoveToPosition* CarlaJoinManeuver::createMoveToPosition(
    int vehicleId,
    const std::string& externalId,
    int platoonId,
    int destinationId,
    double platoonSpeed,
    int platoonLane,
    const std::vector<int>& newPlatoonFormation)
{
    auto* msg = new MoveToPosition("MoveToPosition");
    app_->fillManeuverMessage(msg, vehicleId, externalId, platoonId, destinationId);
    msg->setPlatoonSpeed(platoonSpeed);
    msg->setPlatoonLane(platoonLane);
    msg->setNewPlatoonFormationArraySize(newPlatoonFormation.size());
    for (unsigned int i = 0; i < newPlatoonFormation.size(); ++i) {
        msg->setNewPlatoonFormation(i, newPlatoonFormation[i]);
    }
    return msg;
}

MoveToPositionAck* CarlaJoinManeuver::createMoveToPositionAck(
    int vehicleId,
    const std::string& externalId,
    int platoonId,
    int destinationId,
    double platoonSpeed,
    int platoonLane,
    const std::vector<int>& newPlatoonFormation)
{
    auto* msg = new MoveToPositionAck("MoveToPositionAck");
    app_->fillManeuverMessage(msg, vehicleId, externalId, platoonId, destinationId);
    msg->setPlatoonSpeed(platoonSpeed);
    msg->setPlatoonLane(platoonLane);
    msg->setNewPlatoonFormationArraySize(newPlatoonFormation.size());
    for (unsigned int i = 0; i < newPlatoonFormation.size(); ++i) {
        msg->setNewPlatoonFormation(i, newPlatoonFormation[i]);
    }
    return msg;
}

JoinFormation* CarlaJoinManeuver::createJoinFormation(
    int vehicleId,
    const std::string& externalId,
    int platoonId,
    int destinationId,
    double platoonSpeed,
    int platoonLane,
    const std::vector<int>& newPlatoonFormation)
{
    auto* msg = new JoinFormation("JoinFormation");
    app_->fillManeuverMessage(msg, vehicleId, externalId, platoonId, destinationId);
    msg->setPlatoonSpeed(platoonSpeed);
    msg->setPlatoonLane(platoonLane);
    msg->setNewPlatoonFormationArraySize(newPlatoonFormation.size());
    for (unsigned int i = 0; i < newPlatoonFormation.size(); ++i) {
        msg->setNewPlatoonFormation(i, newPlatoonFormation[i]);
    }
    return msg;
}

JoinFormationAck* CarlaJoinManeuver::createJoinFormationAck(
    int vehicleId,
    const std::string& externalId,
    int platoonId,
    int destinationId,
    double platoonSpeed,
    int platoonLane,
    const std::vector<int>& newPlatoonFormation)
{
    auto* msg = new JoinFormationAck("JoinFormationAck");
    app_->fillManeuverMessage(msg, vehicleId, externalId, platoonId, destinationId);
    msg->setPlatoonSpeed(platoonSpeed);
    msg->setPlatoonLane(platoonLane);
    msg->setNewPlatoonFormationArraySize(newPlatoonFormation.size());
    for (unsigned int i = 0; i < newPlatoonFormation.size(); ++i) {
        msg->setNewPlatoonFormation(i, newPlatoonFormation[i]);
    }
    return msg;
}

} // namespace carla