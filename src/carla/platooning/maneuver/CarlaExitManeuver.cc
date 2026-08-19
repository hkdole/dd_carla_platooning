#include "carla/platooning/maneuver/CarlaExitManeuver.h"
#include "carla/platooning/state/PlatoonTypes.h"
#include "carla/platooning/state/ControlMode.h"

using namespace omnetpp;

namespace carla {

CarlaExitManeuver::CarlaExitManeuver(ICarlaPlatooningApp* app)
    : CarlaManeuver(app)
{
}

void CarlaExitManeuver::startManeuver(const void* parameters)
{
    // Called on the exiting vehicle (veh3).
    // parameters unused — exit target is always the predecessor.
    if (state_ != ExitManeuverState::IDLE) return;

    predecessorId_ = positionHelper_->getPredecessorId();
    if (predecessorId_ < 0) {
        EV_WARN << "[CarlaExitManeuver] No predecessor — nothing to notify.\n";
        // No predecessor, just switch to CC directly
        controllerAdapter_->setControlMode(ControlMode::LEADER_CRUISE);
        positionHelper_->setPlatoonFormation({});
        app_->setInManeuver(false, nullptr);
        return;
    }

    ExitRequest* msg = new ExitRequest();
    msg->setKind(MANEUVER_TYPE);
    msg->setVehicleId(positionHelper_->getId());
    msg->setExternalId(positionHelper_->getExternalId().c_str());
    msg->setPlatoonId(positionHelper_->getPlatoonId());
    msg->setDestinationId(predecessorId_);

    app_->sendUnicast(msg, predecessorId_);

    state_ = ExitManeuverState::WAIT_ACK;

    EV_INFO << "[CarlaExitManeuver][startManeuver]"
            << " actor=" << positionHelper_->getExternalId()
            << " predecessorId=" << predecessorId_
            << " state=WAIT_ACK\n";
}

void CarlaExitManeuver::abortManeuver()
{
    state_ = ExitManeuverState::IDLE;
    app_->setInManeuver(false, nullptr);
}

void CarlaExitManeuver::onPlatoonBeacon(const PlatooningBeacon* pb)
{
    // Nothing to do during exit maneuver
}

void CarlaExitManeuver::onManeuverMessage(const ManeuverMessage* mm)
{
    const std::string className = mm->getClassName();
    if (className.find("ExitRequest") != std::string::npos) {
        handleExitRequest(static_cast<const ExitRequest*>(mm));
    } else if (className.find("ExitAck") != std::string::npos) {
        handleExitAck(static_cast<const ExitAck*>(mm));
    }
}

void CarlaExitManeuver::handleExitRequest(const ExitRequest* msg)
{
    EV_INFO << "[CarlaExitManeuver][handleExitRequest]"
            << " actor=" << positionHelper_->getExternalId()
            << " exiterId=" << msg->getVehicleId()
            << "\n";

    // Remove exiter from our formation
    std::vector<int> formation = positionHelper_->getPlatoonFormation();
    formation.erase(
        std::remove(formation.begin(), formation.end(), msg->getVehicleId()),
        formation.end());
    positionHelper_->setPlatoonFormation(formation);

    // Send ExitAck back to exiter — create a NEW message, don't modify received one
    ExitAck* ack = new ExitAck();
    ack->setKind(MANEUVER_TYPE);  // ← set kind on the NEW message
    ack->setVehicleId(positionHelper_->getId());
    ack->setExternalId(positionHelper_->getExternalId().c_str());
    ack->setPlatoonId(positionHelper_->getPlatoonId());
    ack->setDestinationId(msg->getVehicleId());

    app_->sendUnicast(ack, msg->getVehicleId());

    EV_INFO << "[CarlaExitManeuver][handleExitRequest]"
            << " actor=" << positionHelper_->getExternalId()
            << " action=sent_ExitAck"
            << " newFormationSize=" << formation.size()
            << "\n";
}

void CarlaExitManeuver::handleExitAck(const ExitAck* msg)
{
    // Called on exiter (veh3) — predecessor confirmed exit.
    if (state_ != ExitManeuverState::WAIT_ACK) return;

    EV_INFO << "[CarlaExitManeuver][handleExitAck]"
            << " actor=" << positionHelper_->getExternalId()
            << " action=exit_complete\n";

    // Clear formation and switch to CC
    positionHelper_->setPlatoonFormation({});
    controllerAdapter_->setControlMode(ControlMode::LEADER_CRUISE);
    controllerAdapter_->setActiveController(ActiveController::CC);

    app_->clearFrontVehicle();

    state_ = ExitManeuverState::EXIT_COMPLETE;
    app_->setInManeuver(false, nullptr);
}

void CarlaExitManeuver::onFailedTransmissionAttempt(const ManeuverMessage* mm)
{
    EV_WARN << "[CarlaExitManeuver] Failed transmission: "
            << mm->getClassName() << "\n";
}

} // namespace carla