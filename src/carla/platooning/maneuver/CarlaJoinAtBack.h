#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <omnetpp.h>

#include "veins/base/utils/Coord.h"

#include "carla/platooning/maneuver/CarlaJoinManeuver.h"

namespace carla {

class CarlaJoinAtBack : public CarlaJoinManeuver {
public:
    explicit CarlaJoinAtBack(ICarlaPlatooningApp* app);
    ~CarlaJoinAtBack() override;

    void startManeuver(const void* parameters) override;
    void abortManeuver() override;

    void onPlatoonBeacon(const PlatooningBeacon* pb) override;
    void onFailedTransmissionAttempt(const ManeuverMessage* mm) override;

    bool handleSelfMsg(omnetpp::cMessage* msg) override;

    void handleJoinPlatoonRequest(const JoinPlatoonRequest* msg) override;
    void handleJoinPlatoonResponse(const JoinPlatoonResponse* msg) override;
    void handleMoveToPosition(const MoveToPosition* msg) override;
    void handleMoveToPositionAck(const MoveToPositionAck* msg) override;
    void handleJoinFormation(const JoinFormation* msg) override;
    void handleJoinFormationAck(const JoinFormationAck* msg) override;

private:
    enum class JoinManeuverState {
        IDLE,

        // joiner
        J_WAIT_REPLY,
        J_WAIT_INFORMATION,
        J_MOVE_IN_POSITION,
        J_WAIT_JOIN,

        // leader
        L_WAIT_JOINER_IN_POSITION,
        L_WAIT_JOINER_TO_JOIN,
    };

    struct TargetPlatoonData {
        int platoonId = -1;
        int platoonLeader = -1;
        int platoonLane = -1;
        double platoonSpeed = 0.0;
        int joinIndex = -1;
        std::vector<int> newFormation;
        veins::Coord lastFrontPos = veins::Coord(0, 0, 0);

        void from(const MoveToPosition* msg)
        {
            platoonId = msg->getPlatoonId();
            platoonLeader = msg->getVehicleId();
            platoonLane = msg->getPlatoonLane();
            platoonSpeed = msg->getPlatoonSpeed();

            newFormation.resize(msg->getNewPlatoonFormationArraySize());
            for (unsigned int i = 0; i < msg->getNewPlatoonFormationArraySize(); ++i) {
                newFormation[i] = msg->getNewPlatoonFormation(i);
            }

            auto it = std::find(newFormation.begin(), newFormation.end(), msg->getDestinationId());
            if (it != newFormation.end()) {
                joinIndex = static_cast<int>(std::distance(newFormation.begin(), it));
            }
        }

        int frontId() const
        {
            if (joinIndex <= 0 || joinIndex >= static_cast<int>(newFormation.size())) return -1;
            return newFormation.at(joinIndex - 1);
        }
    };

    struct JoinerData {
        int joinerId = -1;
        int joinerLane = -1;
        std::vector<int> newFormation;

        void from(const JoinPlatoonRequest* msg)
        {
            joinerId = msg->getVehicleId();
            joinerLane = msg->getCurrentLaneIndex();
        }
    };

    bool initializeJoinManeuver(const void* parameters);
    bool processJoinRequest(const JoinPlatoonRequest* msg);
    void sendJoinRequest();

    void resetJoinTracking();
    double computeTargetJoinGapMeters(double frontSpeedMetersPerSecond) const;
    bool updateInPositionConvergence(double actualGapMeters, double targetGapMeters);

private:
    JoinManeuverState joinManeuverState_ = JoinManeuverState::IDLE;
    std::unique_ptr<TargetPlatoonData> targetPlatoonData_;
    std::unique_ptr<JoinerData> joinerData_;

    omnetpp::cMessage* retryTimer_ = nullptr;
    int joinReqRetries_ = 0;

    int inPositionSampleCount_ = 0;

    static constexpr double kApproachGapBufferMeters_ = 2.0;
    static constexpr double kApproachDeltaV_ = 30.0 / 3.6; // 30 km/h
    static constexpr double kJoinGapToleranceMeters_ = 1.0;
    static constexpr int kInPositionRequiredSamples_ = 3;

    static constexpr double kJoinReqRetrySeconds_ = 0.5;
    static constexpr int kJoinReqMaxTries_ = 20;
};

} // namespace carla