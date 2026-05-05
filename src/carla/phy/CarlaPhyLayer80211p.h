#ifndef __PLEXE_CARLAPHYLAYER80211P_H_
#define __PLEXE_CARLAPHYLAYER80211P_H_

#include "veins/modules/phy/PhyLayer80211p.h"
#include "inet/mobility/base/MobilityBase.h"
#include "inet/mobility/contract/IMobility.h"

namespace plexe {

// Veins 802.11p PHY adapter for CARLA-driven mobility.
//
// Standard Veins PHY expects mobility updates compatible with Veins' own mobility stack.
// This class bridges INET mobility, updated from CARLA through CarlaInetMobility, into
// Veins antenna position/heading updates so radio propagation uses current CARLA positions.
//
// This file does not implement platoon protocol logic or vehicle control. It only keeps
// the wireless model synchronized with CARLA/INET vehicle movement.
class CarlaPhyLayer80211p : public veins::PhyLayer80211p
{
  protected:
    // One-shot self-message used to register the NIC after initialization.
    omnetpp::cMessage* regMsg = nullptr;

    // Last known mobility snapshot.
    // Used for registration and to preserve heading when velocity is near zero.
    veins::Coord lastPos = veins::Coord(0, 0, 0);
    veins::Coord lastVel = veins::Coord(0, 0, 0);
    veins::Heading lastHeading = veins::Heading(0);

  protected:
    // OMNeT++ lifecycle hook. Subscribes to mobility updates and schedules NIC registration.
    virtual void initialize(int stage) override;

    // Handles the registration self-message, then delegates normal PHY messages to Veins.
    virtual void handleMessage(omnetpp::cMessage* msg) override;

    // Receives mobilityStateChangedSignal and updates the Veins antenna/NIC position.
    virtual void receiveSignal(omnetpp::cComponent* source,
                               omnetpp::simsignal_t signalID,
                               omnetpp::cObject* obj,
                               omnetpp::cObject* details) override;

    // Registers the NIC with the Veins ConnectionManager once host/nic/cc are available.
    void doRegisterIfNeeded();

    // Converts INET mobility position/velocity into Veins antenna position/heading state.
    void updateFromMobility(inet::IMobility* mob);

  public:
    // Cancels/deletes the registration timer if needed.
    virtual ~CarlaPhyLayer80211p() override;
};

} // namespace plexe

#endif