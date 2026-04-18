#include "carla/phy/CarlaPhyLayer80211p.h"

#include <cmath>

#include "inet/mobility/base/MobilityBase.h"
#include "inet/mobility/contract/IMobility.h"

using namespace omnetpp;

Define_Module(plexe::CarlaPhyLayer80211p);

plexe::CarlaPhyLayer80211p::~CarlaPhyLayer80211p()
{
    if (regMsg) { cancelAndDelete(regMsg); regMsg = nullptr; }
}

static inline veins::Heading headingFromVel(const veins::Coord& v, const veins::Heading& fallback)
{
    if (std::abs(v.x) < 1e-6 && std::abs(v.y) < 1e-6) return fallback;
    return veins::Heading(std::atan2(v.y, v.x));
}

void plexe::CarlaPhyLayer80211p::initialize(int stage)
{
    veins::PhyLayer80211p::initialize(stage);

    if (stage == 0) {
        cModule* host = findHost();
        if (!host) throw cRuntimeError("CarlaPhyLayer80211p: findHost() returned nullptr");

        cModule* mobMod = host->getSubmodule("mobility");
        if (mobMod) {
            mobMod->subscribe(inet::MobilityBase::mobilityStateChangedSignal, this);
        } else {
            EV_WARN << "[CarlaPhyLayer80211p] host has no mobility submodule\n";
        }

        regMsg = new cMessage("carlaRegisterNic");
        scheduleAt(simTime(), regMsg);
    }
}

void plexe::CarlaPhyLayer80211p::receiveSignal(cComponent* source,
                                              simsignal_t signalID,
                                              cObject* obj,
                                              cObject* details)
{
    if (signalID == inet::MobilityBase::mobilityStateChangedSignal) {

        // Prefer source (the emitter) over obj (may be nullptr depending on INET version)
        auto* mob = dynamic_cast<inet::IMobility*>(source);
        if (!mob) {
            EV_WARN << "[CarlaPhyLayer80211p] mobilityStateChangedSignal but source is not IMobility\n";
            return;
        }

        updateFromMobility(mob);

        if (cc && isRegistered) {
            cc->updateNicPos(getParentModule()->getId(),
                             antennaPosition.getPositionAt(),
                             antennaHeading);
        }
        return;
    }

    veins::PhyLayer80211p::receiveSignal(source, signalID, obj, details);
}


void plexe::CarlaPhyLayer80211p::handleMessage(cMessage* msg)
{
    if (msg == regMsg) {
        doRegisterIfNeeded();
        delete regMsg;
        regMsg = nullptr;
        return;
    }
    // otherwise normal PHY behavior
    veins::PhyLayer80211p::handleMessage(msg);
}

void plexe::CarlaPhyLayer80211p::updateFromMobility(inet::IMobility* mob)
{
    inet::Coord ipos = mob->getCurrentPosition();
    inet::Coord ivel = mob->getCurrentVelocity();

    lastPos = veins::Coord(ipos.x, ipos.y, ipos.z);
    lastVel = veins::Coord(ivel.x, ivel.y, ivel.z);
    lastHeading = headingFromVel(lastVel, lastHeading);

    antennaPosition = veins::AntennaPosition(
        getId(),
        lastPos + antennaOffset.rotatedYaw(-lastHeading.getRad()),
        lastVel,
        simTime()
    );
    antennaHeading = veins::Heading(lastHeading.getRad() + antennaOffsetYaw);
}

void plexe::CarlaPhyLayer80211p::doRegisterIfNeeded()
{
    if (isRegistered) return;

    cModule* nic = getParentModule();
    cModule* host = findHost();

    if (!cc || !nic || !host) {
        EV_WARN << "[CarlaPhyLayer80211p] cannot register (missing cc/nic/host)\n";
        return;
    }

    // Try to pull a mobility snapshot; if not available yet, register at origin and update later.
    if (auto* mob = dynamic_cast<inet::IMobility*>(host->getSubmodule("mobility"))) {
        updateFromMobility(mob);
    } else {
        lastPos = veins::Coord(0,0,0);
        lastVel = veins::Coord(0,0,0);
        lastHeading = veins::Heading(0);
        antennaPosition = veins::AntennaPosition(getId(), lastPos, lastVel, simTime());
        antennaHeading = veins::Heading(0);
        EV_WARN << "[CarlaPhyLayer80211p] host.mobility not IMobility; registering at origin\n";
    }

    EV_INFO << "[CarlaPhyLayer80211p] registerNic nicID=" << nic->getId()
            << " pos=(" << antennaPosition.getPositionAt().x
            << "," << antennaPosition.getPositionAt().y
            << "," << antennaPosition.getPositionAt().z << ")\n";

    useSendDirect = cc->registerNic(nic, this,
                                    antennaPosition.getPositionAt(),
                                    antennaHeading);
    isRegistered = true;
}
