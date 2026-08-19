#pragma once
#include "carla/platooning/maneuver/CarlaManeuver.h"
#include "carla/platooning/messages/ExitRequest_m.h"
#include "carla/platooning/messages/ExitAck_m.h"

namespace carla {

enum class ExitManeuverState {
    IDLE,
    WAIT_ACK,       // exiter sent ExitRequest, waiting for ExitAck
    EXIT_COMPLETE
};

class CarlaExitManeuver : public CarlaManeuver {
public:
    explicit CarlaExitManeuver(ICarlaPlatooningApp* app);

    void startManeuver(const void* parameters) override;
    void abortManeuver() override;
    void onPlatoonBeacon(const PlatooningBeacon* pb) override;
    void onManeuverMessage(const ManeuverMessage* mm) override;
    void onFailedTransmissionAttempt(const ManeuverMessage* mm) override;

private:
    void handleExitRequest(const ExitRequest* msg);   // called on predecessor
    void handleExitAck(const ExitAck* msg);           // called on exiter

    ExitManeuverState state_ = ExitManeuverState::IDLE;
    int predecessorId_ = -1;  // stored by exiter when maneuver starts
};

} // namespace carla