// file: ~/omnet6_ws/carlanetpp/src/carlanet/CarlanetManager.h  (DROP-IN)
#pragma once

#include <string>
#include <list>
#include <map>
#include <unordered_map>
#include <vector>

#include <zmq.hpp>
#include <omnetpp.h>

#include "../carlanetpp/CarlaInetMobility.h"
#include "inet/common/INETDefs.h"
#include "inet/common/geometry/common/Coord.h"
#include "inet/common/geometry/common/Quaternion.h"

#include "carla/carlanetpp/carlaApi.h"              // defines message structs + nlohmann::json to/from

class CarlanetManager : public omnetpp::cSimpleModule {
  public:
    CarlanetManager();
    ~CarlanetManager() override;

    bool isConnected() const { return connected; }

    // OMNeT lifecycle
    virtual int  numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessage(omnetpp::cMessage *msg) override;
    virtual void finish() override;
    omnetpp::simtime_t getCarlaInitialTimestamp() const { return initialTimestamp; }

    // Mobility registration
    void registerMobilityModule(CarlaInetMobility *mobilityModule);

    // Convenience JSON roundtrip used by apps
    nlohmann::json sendToAndGetFromCarla(nlohmann::json requestMessage) {
        carla_api::generic_message toCarlaMessage;
        toCarlaMessage.user_defined = requestMessage;
        toCarlaMessage.timestamp    = omnetpp::simTime().dbl();
        sendToCarla(nlohmann::json(toCarlaMessage));
        auto resp = receiveFromCarla<carla_api::generic_response>(10.0);
        return resp.user_defined;
    }
    template<typename S, typename T>
    T sendToAndGetFromCarla(const S& requestMessage) {
        return sendToAndGetFromCarla(nlohmann::json(requestMessage)).get<T>();
    }

  protected:
    // Params
    std::string protocol = "tcp";
    std::string host = "127.0.0.1";
    int         port = 5555;
    int         timeout_ms = 1000;
    double      simulationTimeStep = 0.05;

    // Dynamic types for created actors
    std::string networkActiveModuleType;
    std::string networkPassiveModuleType;

    // ZMQ
    bool zmqReady = false;
    zmq::context_t context;
    zmq::socket_t  socket;

    // Timer
    omnetpp::cMessage *simulationTimeStepEvent = nullptr;
    omnetpp::simtime_t initialTimestamp = 0;
    
    // State
    bool connected = false;

    // Mobility lookup
    std::map<std::string, CarlaInetMobility*> modulesToTrack;

    // Manager controlIn[] gate indexing (stable per actor_id)
    std::unordered_map<std::string, int> gateIndexByActor;
    int nextGateIndex = 0;

    // Buffered app→python messages
    std::vector<std::string> pendingUserMsgs;

        // Late spawn
    void handleLateSpawn();

  protected:
    // internal
    void connect();
    void initializeCarla();
    void doSimulationTimeStep();
    void updateNodesPosition(std::list<carla_api_base::actor_position> actorList);

    void createAndInitializeActor(const carla_api_base::actor_position& newActor);
    void destroyActor(std::string actorId);

    const std::map<std::string,omnetpp::cValue>& getExtraInitParams();

    // Gate helpers
    int  getOrAssignGateIndex(const std::string& actorId);
    void ensureControlInSize(int minSize);
    void connectNodeToControlIn(omnetpp::cModule* node, int idx);
    void disconnectNodeFromControlIn(omnetpp::cModule* node, int idx);

    // ZMQ helpers
    inline void sendToCarla(const nlohmann::json& j) {
        socket.send(zmq::buffer(j.dump()), zmq::send_flags::none);
    }
    nlohmann::json receiveFromCarla(double timeoutFactor);
    template <typename T> T receiveFromCarla(double timeoutFactor = 1) {
        return receiveFromCarla(timeoutFactor).get<T>();
    }
};
