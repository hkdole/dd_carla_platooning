#include "carla/phy/CarlaPhyLayer80211p.h"

#include <cmath>

#include "inet/mobility/base/MobilityBase.h"
#include "inet/mobility/contract/IMobility.h"

using namespace omnetpp;

// Registers this C++ class as the implementation of the NED module.
// The namespace is plexe because the class is declared as plexe::CarlaPhyLayer80211p
// in the header and referenced that way by the NED @class property.
Define_Module(plexe::CarlaPhyLayer80211p);

// Cancels and deletes the one-shot NIC registration timer.
// OMNeT++ self-messages must be canceled before deletion if still scheduled.
plexe::CarlaPhyLayer80211p::~CarlaPhyLayer80211p()
{
    if (regMsg) { cancelAndDelete(regMsg); regMsg = nullptr; }
}

// Computes a heading from the current velocity vector.
// If the vehicle is nearly stopped, keep the previous heading instead of snapping to 0.
static inline veins::Heading headingFromVel(const veins::Coord& v, const veins::Heading& fallback)
{
    if (std::abs(v.x) < 1e-6 && std::abs(v.y) < 1e-6) return fallback;
    return veins::Heading(std::atan2(v.y, v.x));
}

// Initializes the PHY extension.
// This module connects INET/CARLA mobility updates to the Veins 802.11p PHY so the
// ConnectionManager knows where the NIC is as the CARLA vehicle moves.
void plexe::CarlaPhyLayer80211p::initialize(int stage)
{
    veins::PhyLayer80211p::initialize(stage);

    if (stage == 0) {
        cModule* host = findHost();
        if (!host) throw cRuntimeError("CarlaPhyLayer80211p: findHost() returned nullptr");

        // Subscribe to mobility changes emitted by CarlaInetMobility.
        // Each update refreshes antennaPosition and notifies the Veins ConnectionManager.
        cModule* mobMod = host->getSubmodule("mobility");
        if (mobMod) {
            mobMod->subscribe(inet::MobilityBase::mobilityStateChangedSignal, this);
        }
        else {
            EV_WARN << "[CarlaPhyLayer80211p] host has no mobility submodule\n";
        }

        // Register the NIC after initialization so cc/nic/host are available.
        // NOTE: CarlaPhyLayer80211p.ned defines registrationDelay, but it's
        // currently ignored and registers at simTime(). To use the parameter:
        //     scheduleAt(simTime() + par("registrationDelay"), regMsg);
        regMsg = new cMessage("carlaRegisterNic");
        scheduleAt(simTime(), regMsg);
    }
}

// Receives subscribed OMNeT++ signals.
// The important signal here is mobilityStateChangedSignal from CarlaInetMobility.
void plexe::CarlaPhyLayer80211p::receiveSignal(cComponent* source,
                                              simsignal_t signalID,
                                              cObject* obj,
                                              cObject* details)
{
    if (signalID == inet::MobilityBase::mobilityStateChangedSignal) {

        // Prefer source, the emitting mobility module, over obj.
        // Some INET versions pass nullptr or a non-mobility object as obj.
        auto* mob = dynamic_cast<inet::IMobility*>(source);
        if (!mob) {
            EV_WARN << "[CarlaPhyLayer80211p] mobilityStateChangedSignal but source is not IMobility\n";
            return;
        }

        updateFromMobility(mob);

        // Tell Veins' ConnectionManager that the NIC moved.
        // Without this, radio range/interference calculations may use stale positions.
}