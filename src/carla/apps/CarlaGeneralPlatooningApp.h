#pragma once

#include <map>
#include <string>
#include <vector>

#include <omnetpp.h>

#include "veins/base/utils/Coord.h"
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"

#include "carla/carlanetpp/CarlaInetMobility.h"
#include "carla/platooning/adapters/CarlaControllerAdapter.h"
#include "carla/platooning/adapters/CarlaPositionHelper.h"
#include "carla/platooning/adapters/ICarlaPlatooningApp.h"
#include "carla/platooning/controller/ControllerDispatcher.h"
#include "carla/platooning/controller/ControllerInputs.h"
#include "carla/platooning/maneuver/CarlaJoinAtBack.h"
#include "carla/platooning/state/ControlOutput.h"
#include "carla/platooning/state/PlatoonNeighborState.h"

#include "carla/platooning/messages/JoinFormation_m.h"
#include "carla/platooning/messages/JoinPlatoonRequest_m.h"
#include "carla/platooning/messages/JoinPlatoonResponse_m.h"
#include "carla/platooning/messages/ManeuverMessage_m.h"
#include "carla/platooning/messages/MoveToPosition_m.h"
#include "carla/platooning/messages/PlatooningBeacon_m.h"
#include "carla/platooning/messages/UpdatePlatoonData_m.h"
#include "carla/platooning/messages/UpdatePlatoonFormation_m.h"

namespace carla {

class CarlaGeneralPlatooningApp : public veins::DemoBaseApplLayer, public ICarlaPlatooningApp {
public:
    CarlaGeneralPlatooningApp();
    ~CarlaGeneralPlatooningApp() override;

    // ICarlaPlatooningApp
    const PlatoonRole& getPlatoonRole() const override { return role_; }
    void setPlatoonRole(PlatoonRole r) override { role_ = r; }

    bool isInManeuver() const override { return inManeuver_; }
    void setInManeuver(bool b, CarlaManeuver* maneuver) override;

    CarlaPositionHelper* getPositionHelper() override { return &positionHelper_; }
    CarlaControllerAdapter* getControllerAdapter() override { return &controllerAdapter_; }

    double getStandstillDistance(ActiveController controller) const override;
    double getHeadway(ActiveController controller) const override;
    double getTargetDistance(double speed) const override;
    double getTargetDistance(ActiveController controller, double speed) const override;
    ActiveController getTargetController() const override;

    veins::Coord getCurrentPosition() const override;
    int getCurrentLaneIndex() const override;

    void sendUnicast(omnetpp::cPacket* msg, int destination) override;

    void fillManeuverMessage(
        ManeuverMessage* msg,
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId) override;

    UpdatePlatoonFormation* createUpdatePlatoonFormation(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& platoonFormation) override;

    UpdatePlatoonData* createUpdatePlatoonData(
        int vehicleId,
        const std::string& externalId,
        int platoonId,
        int destinationId,
        double platoonSpeed,
        int platoonLane,
        const std::vector<int>& platoonFormation,
        int newPlatoonId) override;

    // maneuver helpers used by adapted JoinAtBack layer
    void maneuverSetControlMode(ControlMode mode);
    void maneuverSetJoinFrontVehicleId(int frontId);
    void maneuverClearJoinFrontVehicleId();
    void maneuverSetPlatoonSpeed(double platoonSpeed);
    void maneuverSetFormation(const std::vector<int>& formation);
    void maneuverCompleteJoinAsFollower(const std::vector<int>& formation, int platoonId, int leaderId);

protected:
    void initialize(int stage) override;
    void handleSelfMsg(omnetpp::cMessage* msg) override;
    void handleLowerMsg(omnetpp::cMessage* msg) override;

    void startJoinManeuverIfConfigured();

    void refreshNeighborFromBeacon(const PlatooningBeacon* pb);
    bool getNeighbor(int id, PlatoonNeighborState& out, omnetpp::simtime_t& age) const;

    void onPlatoonBeacon(const PlatooningBeacon* pb);
    void onManeuverMessage(const ManeuverMessage* mm);
    void handleUpdatePlatoonFormation(const UpdatePlatoonFormation* msg);
    void handleUpdatePlatoonData(const UpdatePlatoonData* msg);

    void sendPlatooningBeacon();
    void onControlTick();

    double scalarSpeedFromVelocity(const veins::Coord& v) const;
    std::vector<int> parseFormation(const std::string& csv) const;
    double computeGapToNeighbor(const PlatoonNeighborState& n) const;
    double computeSpeedBiasFromGapError(double gapError, double gain, double limit) const;
    int findFrontVehicleIdInFormation(const std::vector<int>& formation) const;

    ControllerInputs buildControllerInputs() const;
    ControlOutput computeControllerOutput() const;

protected:
    CarlaInetMobility* mobility_ = nullptr;

    std::string actorId_;
    int nodeId_ = -1;
    int platoonId_ = -1;
    int leaderId_ = -1;

    double nominalPlatoonSpeed_ = 0.0;
    double leaderTargetSpeed_ = 0.0;
    double vehicleLength_ = 4.5;

    double headway_ = 0.5;
    double gapMin_ = 5.0;
    double kGap_ = 0.2;
    double kDv_ = 0.7;
    double kSpeedP_ = 0.7;
    double aMin_ = -4.0;
    double aMax_ = 2.5;

    double kAccFF_ = 1.0;
    double kLeaderDv_ = 0.05;
    double kGapSpeed_ = 0.03;
    double maxClosureSpeed_ = 1.0;

    double caccXi_ = 1.0;
    double caccOmegaN_ = 0.2;

    simtime_t beaconInterval_ = SIMTIME_ZERO;
    simtime_t controlInterval_ = SIMTIME_ZERO;
    simtime_t maxAge_ = SIMTIME_ZERO;
    simtime_t startManeuverAt_ = SIMTIME_ZERO;

    int joinPosition_ = -1;
    double approachDeltaV_ = 0.0;
    double inPosSlack_ = 0.5;

    CarlaPositionHelper positionHelper_;
    CarlaControllerAdapter controllerAdapter_;
    ControllerDispatcher controllerDispatcher_;

    PlatoonRole role_ = PlatoonRole::NONE;
    bool inManeuver_ = false;
    CarlaManeuver* activeManeuver_ = nullptr;
    CarlaJoinAtBack* joinManeuver_ = nullptr;

    omnetpp::cMessage* controlTimer_ = nullptr;
    omnetpp::cMessage* startTimer_ = nullptr;

    std::map<int, PlatoonNeighborState> neighborsByVehicleId_;

    double desiredAcceleration_ = 0.0;
    double desiredSpeed_ = 0.0;
    bool hasControl_ = false;
    ControlMode controlMode_ = ControlMode::HOLD;

    double actualAcceleration_ = 0.0;
    double lastExportSpeed_ = 0.0;
    simtime_t lastExportTime_ = SIMTIME_ZERO;

    int joinFrontVehicleId_ = -1;
    long beaconSequence_ = 0;

    static omnetpp::simsignal_t desiredAccelerationSignal_;
    static omnetpp::simsignal_t desiredSpeedSignal_;
    static omnetpp::simsignal_t controlModeSignal_;
    static omnetpp::simsignal_t activeControllerSignal_;
    static omnetpp::simsignal_t hasControlSignal_;

    static omnetpp::simsignal_t speedSignal_;
    static omnetpp::simsignal_t accelerationSignal_;
    static omnetpp::simsignal_t controllerAccelerationExportSignal_;
    static omnetpp::simsignal_t distanceSignal_;
    static omnetpp::simsignal_t relativeSpeedSignal_;
};

} // namespace carla
