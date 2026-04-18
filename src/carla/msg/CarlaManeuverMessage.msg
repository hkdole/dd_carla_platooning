cplusplus {{
#include <vector>
}}

enum CarlaManeuverType
{
    JOIN_REQ = 1;
    JOIN_RSP = 2;
    MOVE_TO_POS = 3;
    MOVE_TO_POS_ACK = 4;
    JOIN_FORMATION = 5;
    JOIN_FORMATION_ACK = 6;
    UPDATE_FORMATION = 7;
}

class CarlaManeuverMessage extends omnetpp::cPacket
{
    int msgType;            // CarlaManeuverType
    int senderId;           // OMNeT++ vehicle id (myId)
    int platoonId;          // logical platoon id
    int leaderId;           // leader id (typically 0)
    int joinerId;           // joiner id (for join-related messages)
    bool permitted = false; // JOIN_RSP
    double platoonSpeed = -1;   // m/s (MOVE_TO_POS / JOIN_FORMATION / UPDATE_FORMATION)
    int joinIndex = -1;         // insertion index (JoinAtBack uses end)
    int newFormation[];         // ordered ids: [leader, ..., tail]
}
