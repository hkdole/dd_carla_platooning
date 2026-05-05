#pragma once

#include <string>
#include <list>
#include <map>
#include <unordered_map>
#include <vector>

#include <zmq.hpp>
#include <omnetpp.h>

#include "carlanet/CarlaInetMobility.h"
#include "inet/common/INETDefs.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/common/geometry/common/Quaternion.h"

#include "carlanet/carlaApi.h"

class CarlanetManager : public omnetpp::cSimpleModule {
  public:
    CarlanetManager();
    ~CarlanetManager() override;

    bool isConnected() const { return connected; }

    virtual int  numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    virtual void finish() override;

    omnetpp::simtime_t getCarlaInitialTimestamp() const { return initialTimestamp; }

    void registerMobilityModule(CarlaInetMobility *mobilityModule);

    nlohmann::json sendToAndGetFromCarla(nlohmann::json requestMessage)
    {
        carla_api::generic_message toCarlaMessage;
        toCarlaMessage.user_defined = requestMessage;
        toCarlaMessage.timestamp = omnetpp::simTime().dbl();

        sendToCarla(nlohmann::json(toCarlaMessage));

        auto resp = receiveFromCarla<carla_api::generic_response>(10.0);
        return resp.user_defined;
    }

    template <typename S, typename T>
    T sendToAndGetFromCarla(const S& requestMessage)
    {
        return sendToAndGetFromCarla(nlohmann::json(requestMessage)).get<T>();
    }

  protected:
    std::string protocol = "tcp";
    std::string host = "127.0.0.1";
    int port = 5555;
    int timeout_ms = 1000;
    double simulationTimeStep = 0.05;

    std::string networkActiveModuleType;
    std::string networkPassiveModuleType;

    bool zmqReady = false;
    zmq::context_t context;
    zmq::socket_t socket;

    omnetpp::cMessage *simulationTimeStepEvent = nullptr;
    omnetpp::simtime_t initialTimestamp = 0;

    bool connected = false;

    std::map<std::string, CarlaInetMobility*> modulesToTrack;

    std::unordered_map<std::string, int> gateIndexByActor;
    int nextGateIndex = 0;

    std::vector<std::string> pendingUserMsgs;

  protected:
    void connect();
    void initializeCarla();
    void doSimulationTimeStep();
    void updateNodesPosition(std::list<carla_api_base::actor_position> actorList);

    void handleLateSpawn();
    void createAndInitializeActor(const carla_api_base::actor_position& newActor);
    void destroyActor(std::string actorId);

    const std::map<std::string, omnetpp::cValue>& getExtraInitParams();

    int getOrAssignGateIndex(const std::string& actorId);
    void ensureControlInSize(int minSize);
    void connectNodeToControlIn(omnetpp::cModule* node, int idx);
    void disconnectNodeFromControlIn(omnetpp::cModule* node, int idx);

    void sendToCarla(const nlohmann::json& j);

    nlohmann::json receiveFromCarla(double timeoutFactor = 1.0);

    template <typename T>
    T receiveFromCarla(double timeoutFactor = 1.0)
    {
        return receiveFromCarla(timeoutFactor).get<T>();
    }
};
