#pragma once

#include <omnetpp.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "inet/common/geometry/common/Coord.h"
#include "inet/common/INETDefs.h"

#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "veins/modules/messages/DemoSafetyMessage_m.h"

#include "carla/carlanetpp/CarlaInetMobility.h"

namespace carla {

class CarlaPlexeApp : public veins::DemoBaseApplLayer {
public:
    ~CarlaPlexeApp() override;

protected:
    void initialize(int stage) override;
    void handleSelfMsg(omnetpp::cMessage* msg) override;
    void onBSM(veins::DemoSafetyMessage* bsm) override;

private:
    // Bridge-consumed signals
    static omnetpp::simsignal_t desiredAccelerationSignal;
    static omnetpp::simsignal_t desiredSpeedSignal;
    static omnetpp::simsignal_t joinStateSignal;

    enum class State {
        IDLE = 0,

        // Joiner side
        J_WAIT_REPLY,
        J_WAIT_INFORMATION,
        J_MOVE_IN_POSITION,
        J_WAIT_JOIN,
        J_FOLLOWER,   // join complete: role flipped to follower, steady follower control

        // Leader side
        L_WAIT_JOINER_IN_POSITION,
        L_WAIT_JOINER_TO_JOIN
    };

    // Parameters
    std::string actor_id;      // "veh0" ... (logging)
    std::string role;         // "leader" | "follower" | "joiner"
    int node_id = -1;

    int platoon_id = 0;
    int leader_id  = 0;

    std::string initialFormationStr;

    omnetpp::simtime_t beaconInterval;
    omnetpp::simtime_t control_interval;

    // Steady-state spacing / control (post-join)
    double headway = 0.5;     // s (PLEXE ploegH analogue)
    double gap_min  = 5.0;     // m (standstill distance analogue)
    double k_gap    = 0.2;     // gap gain (ploegKp analogue)
    double k_dv     = 0.7;     // relative speed gain (ploegKd analogue)

    double platoon_speed = 27.777777;  // m/s
    double k_speed_p      = 0.7;
    double a_min         = -4.0;
    double a_max         =  2.5;

    // Join maneuver timing/behavior
    omnetpp::simtime_t start_maneuver_at;
    omnetpp::simtime_t join_req_retry;      // used as timeout/retry interval (optional)
    int join_req_max_tries = 1;              // default 1 for "PLEXE-like" behavior (one request)

    double approach_delta_v   = 8.333333;   // m/s (30 km/h)
    double approach_spacing  = 15.0;       // m (PLEXE setCACCConstantSpacing(15))
    double in_pos_slack       = 11.0;       // m (PLEXE "+ 11")
    double vehicle_length    = 4.5;        // m

    omnetpp::simtime_t max_age;
    bool debug = true;

    // Optional: leader permission gate (PLEXE app->isJoinAllowed()).
    bool joinAllowed = true;

    // Modules
    CarlaInetMobility* mobility = nullptr;

    // Timers (ours)
    omnetpp::cMessage* controlTimer = nullptr;
    omnetpp::cMessage* startTimer   = nullptr;
    omnetpp::cMessage* retryTimer   = nullptr;

    // Neighbor table
    struct Neighbor {
        inet::Coord pos;
        inet::Coord vel;
        omnetpp::simtime_t last = -1;
        std::string role;
        double desired_acceleration = 0.0;
    };
    std::unordered_map<int, Neighbor> neighborsByVehicleId;

    // Platoon Formation (leader-beaconed)
    std::vector<int> platoonFormation;

    // Maneuver bookkeeping
    State joinState = State::IDLE;

    int joinerId   = -1;
    int joinIndex  = -1;
    std::vector<int> pendingFormation;

    int joinReqTries = 0;

    // --- protocol tracing / correlation (Step 2 parity) ---
    std::unordered_map<std::string, uint32_t> txSeqByMessageType;  // per-msgType TX sequenceuence
    uint32_t externalId = 0;                           // correlates one join attempt across messages

    // Control outputs
    double desired_acceleration = 0.0;
    double desired_speed        = 0.0;
    inet::Coord lastForward = inet::Coord(1, 0, 0);

    // For braking scenario only
    // --- leader brake profile ---
    simtime_t leader_brake_at = -1;
    simtime_t leader_brake_hold_for = 0;
    double leader_brake_speed = 0.0;       // m/s
    double leader_post_brake_speed = -1.0; // m/s (<0 => use platoon_speed)

private:
    // Helpers
    int deriveMyId() const;

    bool isLeader() const { return role == "leader"; }
    bool isJoiner() const { return role == "joiner"; }
    bool isFollower() const { return role == "follower"; }

    // Join-phase helper (used to relax platoon_id filtering and enforce nonce correlation)
    bool joinerInManeuver() const;

    // Monotonic sequence generator per message msgType
    uint32_t nextSequence(const std::string& msgType);

    void setState(State s);

    // Safe parameter getters (avoid runtime error if param missing)
    bool hasParam(const char* n) const;
    int parIntOr(const char* n, int def);
    double parDoubleOr(const char* n, double def);
    double parSpeedMpsOr(const char* n, double defMps);
    bool parBoolOr(const char* n, bool def);
    std::string parStringOr(const char* n, const std::string& def);
    omnetpp::simtime_t parTimeOr(const char* n, omnetpp::simtime_t def);

    // Spacing utility (mirror PLEXE targetDistance idea)
    double targetDistance(double speedMps) const { return gap_min + headway * speedMps; }

    // V2V
    void sendPlatoonBeacon();
    void sendManeuverMsg(const std::string& msgType, int destination_id, bool permitted = true);

    // Param pack/unpack on DemoSafetyMessage
    void setParI(veins::DemoSafetyMessage* m, const char* n, int v);
    void setParD(veins::DemoSafetyMessage* m, const char* n, double v);
    void setParB(veins::DemoSafetyMessage* m, const char* n, bool v);
    void setParS(veins::DemoSafetyMessage* m, const char* n, const std::string& v);

    bool getParI(veins::DemoSafetyMessage* m, const char* n, int& v) const;
    bool getParD(veins::DemoSafetyMessage* m, const char* n, double& v) const;
    bool getParB(veins::DemoSafetyMessage* m, const char* n, bool& v) const;
    bool getParS(veins::DemoSafetyMessage* m, const char* n, std::string& v) const;

    static std::string encForm(const std::vector<int>& f);
    static std::vector<int> decForm(const std::string& s);

    // Maneuver handlers
    void startJoinManeuver();
    void handleJoinReq(int fromId, int reqPlatoonId, int reqLeaderId);
    void handleJoinRsp(bool permitted, int fromLeader);
    void handleMoveToPos(double pSpeed, int idx, const std::vector<int>& f);
    void handleMoveToPosAck(int fromJoiner);
    void handleJoinFormation(double pSpeed, int idx, const std::vector<int>& f);
    void handleJoinFormationAck(int fromJoiner);
    void handleUpdateFormation(const std::vector<int>& f);

    // Control
    void onControlTick();

    bool getNeighbor(int id, Neighbor& out, omnetpp::simtime_t& age) const;
    inet::Coord forwardUnit(const inet::Coord& v);

    double gapToIdLong(int predId,
                       const inet::Coord& myPos,
                       const inet::Coord& myVel,
                       double& vPredLong,
                       omnetpp::simtime_t& ageOut);

    bool findMyIndexInFormation(int& idxOut) const;

    // PLEXE-like "in position" check triggered by PlatooningBeacon from front vehicle
    void joinerCheckInPositionOnFrontPlatooningBeacon(int frontId);

    const char* stateStr(State s) const;
};

} // namespace carla
