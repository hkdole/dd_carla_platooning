#ifndef __PLEXE_CARLAPHYLAYER80211P_H_
#define __PLEXE_CARLAPHYLAYER80211P_H_

#include "veins/modules/phy/PhyLayer80211p.h"
#include "inet/mobility/base/MobilityBase.h"
#include "inet/mobility/contract/IMobility.h"

namespace plexe {

class CarlaPhyLayer80211p : public veins::PhyLayer80211p
{
  protected:
    omnetpp::cMessage* regMsg = nullptr;

    // cache last known mobility (so registration uses something reasonable)
    veins::Coord lastPos = veins::Coord(0,0,0);
    veins::Coord lastVel = veins::Coord(0,0,0);
    veins::Heading lastHeading = veins::Heading(0);

  protected:
    virtual void initialize(int stage) override;
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void receiveSignal(omnetpp::cComponent* source,
                               omnetpp::simsignal_t signalID,
                               omnetpp::cObject* obj,
                               omnetpp::cObject* details) override;

    void doRegisterIfNeeded();
    void updateFromMobility(inet::IMobility* mob);

  public:
    virtual ~CarlaPhyLayer80211p() override;
};

} // namespace plexe

#endif
