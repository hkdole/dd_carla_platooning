// BridgeApp.cc
#include <cstring>     // strcmp
#include <string>
#include <stdexcept>
#include <sstream>
#include <iomanip>

#include <omnetpp.h>
#include "BridgeApp.h"

using namespace omnetpp;

// -----------------------------------------------------------------------------
// Module registration
// -----------------------------------------------------------------------------
Define_Module(BridgeApp);

// -----------------------------------------------------------------------------
// Initialize
// -----------------------------------------------------------------------------
void BridgeApp::initialize()
{
    actor_id = par("actor_id").stdstringValue();
    send_every_ms = par("send_every_ms").intValue();
    if (send_every_ms <= 0) send_every_ms = 50;

    tickInterval = SimTime(send_every_ms, SIMTIME_MS);

    desired_acceleration = 0.0;
    desired_speed = 0.0;
    controllerHasControl = false;

    controlUpdated = false;
    seenAccel = false;
    seenSpeed = false;
    seenHasControl = false;

    nextAllowedSend = simTime();

    cModule* car = getParentModule();
    cModule* appl = car ? car->getSubmodule("appl") : nullptr;

    {
        std::ostringstream oss;
        oss << "[BridgeApp][initialize]"
            << " hop=BRIDGE_LOCAL"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " send_every_ms=" << send_every_ms
            << " parent=" << (car ? car->getFullPath() : "<null>")
            << " appl=" << (appl ? appl->getFullPath() : "<null>")
            << "\n";
        EV_INFO << oss.str();
    }

    if (appl) {
        for (const char* sig : {"desired_acceleration", "desired_speed", "has_control"}) {
            try {
                appl->subscribe(sig, this);
                EV_INFO << "[BridgeApp][initialize] Subscribed to signal '" << sig
                        << "' on component " << appl->getFullPath() << "\n";
            }
            catch (std::exception& e) {
                EV_WARN << "[BridgeApp][initialize] Failed to subscribe to signal '" << sig
                        << "' on " << appl->getFullPath()
                        << " : " << e.what() << "\n";
            }
            catch (...) {
                EV_WARN << "[BridgeApp][initialize] Failed to subscribe to signal '" << sig
                        << "' on " << appl->getFullPath()
                        << " : unknown exception\n";
            }
        }
    }

    tick = new cMessage("bridge.send");
    tick->setSchedulingPriority(-1);
}

// -----------------------------------------------------------------------------
// Signal receive – control law values arrive here
// -----------------------------------------------------------------------------
void BridgeApp::receiveSignal(cComponent *src,
                              simsignal_t signalID,
                              double value,
                              cObject *details)
{
    (void)src;
    (void)details;
    Enter_Method_Silent();

    const char* sigName = getSignalName(signalID);
    if (!sigName) return;

    if (std::strcmp(sigName, "desired_acceleration") == 0) {
        desired_acceleration = value;
        seenAccel = true;
    }
    else if (std::strcmp(sigName, "desired_speed") == 0) {
        desired_speed = value;
        seenSpeed = true;
    }
    else if (std::strcmp(sigName, "has_control") == 0) {
        controllerHasControl = (value != 0.0);
        seenHasControl = true;
    }
    else {
        return;
    }

    // Wait until we have a complete controller snapshot at least once.
    if (!(seenAccel && seenSpeed && seenHasControl))
        return;

    controlUpdated = true;

    SimTime when = std::max(simTime(), nextAllowedSend);

    if (!tick->isScheduled()) {
        scheduleAt(when, tick);
    }
    else if (tick->getArrivalTime() > when) {
        cancelEvent(tick);
        scheduleAt(when, tick);
    }
}

// -----------------------------------------------------------------------------
// Low-level JSON send to CarlanetManager
// -----------------------------------------------------------------------------
void BridgeApp::sendJson(const std::string& userDefinedJson)
{
    auto* msg = new cMessage("bridge.json");
    // Manager must read msg->par("json")
    msg->addPar("json") = userDefinedJson.c_str();
    send(msg, "toManager");
}

// -----------------------------------------------------------------------------
// Main handler – tick drives GENERIC_MESSAGE out to manager
// -----------------------------------------------------------------------------
void BridgeApp::handleMessage(omnetpp::cMessage *msg)
{
    if (msg == tick) {
        if (!(seenAccel && seenSpeed && seenHasControl) || !controlUpdated) {
            controlUpdated = false;
            return;
        }

        std::ostringstream js;
        js.setf(std::ios::fixed);
        js << std::setprecision(6);

        js << "{"
           << "\"msg_type\":\"CONTROL\","
           << "\"actor_id\":\"" << actor_id << "\","
           << "\"ctrl\":{"
           << "\"has_control\":" << (controllerHasControl ? "true" : "false") << ","
           << "\"desired_acceleration\":" << desired_acceleration << ","
           << "\"desired_speed\":" << desired_speed
           << "}"
           << "}";

        {
            std::ostringstream oss;
            oss.setf(std::ios::fixed);
            oss << std::setprecision(6);

            oss << "[BridgeApp][handleMessage]"
                << " hop=BRIDGE_TO_MANAGER"
                << " simulation_time=" << SIMTIME_DBL(simTime())
                << " actor=" << actor_id
                << " desired_speed=" << desired_speed
                << " desired_acceleration=" << desired_acceleration
                << " has_control=" << (controllerHasControl ? 1 : 0)
                << "\n";
            EV_INFO << oss.str();
        }

        sendJson(js.str());

        controlUpdated = false;
        nextAllowedSend = simTime() + tickInterval;
        return;
    }

    EV_WARN << "[BridgeApp][handleMessage]"
            << " hop=BRIDGE_LOCAL"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " warning=unexpected_message"
            << " msg=" << msg->getName()
            << "\n";
    delete msg;
}

// -----------------------------------------------------------------------------
// Finish
// -----------------------------------------------------------------------------
void BridgeApp::finish()
{
    if (tick) {
        cancelAndDelete(tick);
        tick = nullptr;
    }
}