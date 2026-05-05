#include "carla/phy/CarlaPhyLayer80211p.h"

#include <cmath>

#include "inet/mobility/base/MobilityBase.h"
#include "inet/mobility/contract/IMobility.h"

using namespace omnetpp;

// Custom Veins 802.11p PHY layer for CARLA-backed mobility.
//
// Veins' normal 802.11p PHY expects mobility updates from the Veins/TraCI stack.
// This platform gets vehicle position/velocity from CARLA through CarlaInetMobility,
// which is an INET mobility module. This class bridges that mismatch by listening
// to INET mobilityStateChangedSignal and updating Veins' antenna position manually.
Define_Module(plexe::CarlaPhyLayer80211p);

// Cancels and deletes the delayed NIC-registration self-message.
// OMNeT++ self-messages must be canceled before deletion if still scheduled.
plexe::CarlaPhyLayer80211p::~CarlaPhyLayer80211p()
{
    if (regMsg) { cancelAndDelete(regMsg); regMsg = nullptr; }
}

// Computes a heading angle from horizontal velocity.
// If the vehicle is nearly stopped, keep the previous heading to avoid noisy atan2(0, 0) behavior.
static inline veins::Heading headingFromVel(const veins::Coord& v, const veins::Heading& fallback)
{
    if (std::abs(v.x) < 1e-6 && std::abs(v.y) < 1e-6) return fallback;
    return veins::Heading(std::atan2(v.y, v.x));
}

// OMNeT++ initialization hook.
// Stage 0 subscribes to the host mobility module and schedules registration with Veins' ConnectionManager.
void plexe::CarlaPhyLayer80211p::initialize(int stage)
{
    veins::PhyLayer80211p::initialize(stage);

    if (stage == 0) {
        cModule* host = findHost();
        if (!host) throw cRuntimeError("CarlaPhyLayer80211p: findHost() returned nullptr");

        // The vehicle host should contain a mobility submodule, usually CarlaInetMobility.
        // It emits INET mobilityStateChangedSignal after CarlanetManager applies a CARLA snapshot.
        cModule* mobMod = host->getSubmodule("mobility");
        if (mobMod) {
            mobMod->subscribe(inet::MobilityBase::mobilityStateChangedSignal, this);
        }
        else {
            EV_WARN << "[CarlaPhyLayer80211p] host has no mobility submodule\n";
        }

        // Registration is delayed until simTime() so the base PHY and ConnectionManager pointers
        // have finished their own initialization path.
        regMsg = new cMessage("carlaRegisterNic");
        scheduleAt(simTime(), regMsg);
    }
}

// Receives INET mobility updates and pushes the new antenna position into Veins.
// This is the key CARLA/INET -> Veins radio-location bridge.
void plexe::CarlaPhyLayer80211p::receiveSignal(cComponent* source,
                                              simsignal_t signalID,
                                              cObject* obj,
                                              cObject* details)
{
    if (signalID == inet::MobilityBase::mobilityStateChangedSignal) {

        // Prefer source, the signal emitter. In some INET versions obj may be null
        // or may not carry the mobility module itself.
        auto* mob = dynamic_cast<inet::IMobility*>(source);
        if (!mob) {
            EV_WARN << "[CarlaPhyLayer80211p] mobilityStateChangedSignal but source is not IMobility\n";
            return;
        }

        // Convert INET position/velocity into Veins antenna position/heading.
        updateFromMobility(mob);

        // If the NIC is already known to the ConnectionManager, update its radio position.
        // Without this, wireless range/interference calculations may use stale coordinates.
        if (cc && isRegistered) {
            cc->updateNicPos(getParentModule()->getId(),
                             antennaPosition.getPositionAt(),
                             antennaHeading);
        }
        return;
    }

    // Keep all normal Veins PHY signal handling for non-mobility signals.
    veins::PhyLayer80211p::receiveSignal(source, signalID, obj, details);
}

// Handles this PHY layer's self-message and all normal PHY messages.
// regMsg performs one-time registration with the Veins ConnectionManager.
void plexe::CarlaPhyLayer80211p::handleMessage(cMessage* msg)
{
    if (msg == regMsg) {
        doRegisterIfNeeded();
        delete regMsg;
        regMsg = nullptr;
        return;
    }

    // Other messages are normal PHY events such as AirFrame processing.
    veins::PhyLayer80211p::handleMessage(msg);
}

// Copies the latest INET mobility state into Veins antenna state.
// AntennaPosition is what Veins uses for path loss, radio range, and direct-send decisions.
void plexe::CarlaPhyLayer80211p::updateFromMobility(inet::IMobility* mob)
{
    inet::Coord ipos = mob->getCurrentPosition();
    inet::Coord ivel = mob->getCurrentVelocity();

    // Convert INET coordinates to Veins coordinates.
    lastPos = veins::Coord(ipos.x, ipos.y, ipos.z);
    lastVel = veins::Coord(ivel.x, ivel.y, ivel.z);

    // Estimate heading from velocity. If stopped, retain the last heading.
    lastHeading = headingFromVel(lastVel, lastHeading);

    // Antenna offset is defined in the vehicle frame. Rotate it into world frame
    // using the current heading before adding it to the vehicle position.
    antennaPosition = veins::AntennaPosition(
        getId(),
        lastPos + antennaOffset.rotatedYaw(-lastHeading.getRad()),
        lastVel,
        simTime()
    );

    // Antenna yaw may differ from vehicle heading if configured in the NED parameters.
    antennaHeading = veins::Heading(lastHeading.getRad() + antennaOffsetYaw);
}

// Registers this NIC with Veins' ConnectionManager once.
// The ConnectionManager needs each NIC's initial position before it can deliver 802.11p frames.
void plexe::CarlaPhyLayer80211p::doRegisterIfNeeded()
{
    if (isRegistered) return;

    cModule* nic = getParentModule();
    cModule* host = findHost();

    if (!cc || !nic || !host) {
        EV_WARN << "[CarlaPhyLayer80211p] cannot register (missing cc/nic/host)\n";
        return;
    }

    // Try to register at the current CARLA/INET mobility position.
    // If mobility is unavailable, register at origin and rely on later mobility updates.
    if (auto* mob = dynamic_cast<inet::IMobility*>(host->getSubmodule("mobility"))) {
        updateFromMobility(mob);
    }
    else {
        lastPos = veins::Coord(0, 0, 0);
        lastVel = veins::Coord(0, 0, 0);
        lastHeading = veins::Heading(0);
        antennaPosition = veins::AntennaPosition(getId(), lastPos, lastVel, simTime());
        antennaHeading = veins::Heading(0);
        EV_WARN << "[CarlaPhyLayer80211p] host.mobility not IMobility; registering at origin\n";
    }

    EV_INFO << "[CarlaPhyLayer80211p] registerNic nicID=" << nic->getId()
            << " pos=(" << antennaPosition.getPositionAt().x
            << "," << antennaPosition.getPositionAt().y
            << "," << antennaPosition.getPositionAt().z << ")\n";

    // registerNic() returns whether this NIC should use sendDirect optimization.
    // This is part of normal Veins PHY/ConnectionManager behavior.
    useSendDirect = cc->registerNic(nic, this,
                                    antennaPosition.getPositionAt(),
                                    antennaHeading);
    isRegistered = true;
}
