// BridgeApp.cc
#include "carla/apps/BridgeApp.h"

#include <algorithm>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdexcept>

using namespace omnetpp;

// Registers this class as an OMNeT++ simple module.
// The corresponding NED file instantiates BridgeApp inside each vehicle node.
Define_Module(BridgeApp);

namespace {

// Clamps a normalized CARLA actuator command into [0, 1].
// CARLA throttle and brake fields are normalized, not physical acceleration values.
static inline double clamp01(double v)
{
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
}

} // namespace

// Escapes a C++ string so it can be safely embedded in a JSON string literal.
// Used for actor_id because it is copied directly into the CONTROL payload.
std::string BridgeApp::jsonEscape(const std::string& s)
{
    std::ostringstream out;
    for (char c : s) {
        switch (c) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << c; break;
        }
    }
    return out.str();
}

// Returns true once the bridge has enough signal values to send one valid control command.
// desired_acceleration and desired_speed are exported as metadata and can safely default to zero.
bool BridgeApp::haveCompleteControlSnapshot() const
{
    // Longitudinal actuation requires at least has_control + throttle + brake.
    // Other CARLA control fields default to false until their first signal arrives.
    return seenHasControl && seenThrottle && seenBrake;
}

// Initializes the bridge for one CARLA actor.
// Reads NED/ini parameters, subscribes to the application module's control signals,
// and creates the self-message used to rate-limit JSON CONTROL transmissions.
void BridgeApp::initialize()
{
    actor_id = par("actor_id").stdstringValue();      // CARLA actor identifier expected by pyCARLANeT.
    send_every_ms = par("send_every_ms").intValue();  // ms; minimum interval between CONTROL JSON messages.
    if (send_every_ms <= 0) send_every_ms = 50;       // 50 ms matches a common 20 Hz CARLA/OMNeT++ step.

    tickInterval = SimTime(send_every_ms, SIMTIME_MS);
    nextAllowedSend = simTime();

    desired_acceleration = 0.0; // m/s^2; controller output metadata/logging field.
    desired_speed = 0.0;        // m/s; controller target metadata/logging field.
    controllerHasControl = false;

    // Final low-level CARLA longitudinal command produced by CarlaGeneralPlatooningApp.
    controlThrottle = 0.0;        // normalized [0, 1].
    controlBrake = 0.0;           // normalized [0, 1].
    controlHandBrake = false;
    controlReverse = false;
    controlManualGearShift = false;

    // Signal bookkeeping. A snapshot is not sent until required signals have been observed.
    controlUpdated = false;
    seenAccel = false;
    seenSpeed = false;
    seenHasControl = false;
    seenThrottle = false;
    seenBrake = false;
    seenHandBrake = false;
    seenReverse = false;
    seenManualGearShift = false;

    cModule* car = getParentModule();
    cModule* appl = car ? car->getSubmodule("appl") : nullptr;

    EV_INFO << "[BridgeApp][initialize]"
            << " hop=BRIDGE_LOCAL"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << actor_id
            << " send_every_ms=" << send_every_ms
            << " parent=" << (car ? car->getFullPath() : "<null>")
            << " appl=" << (appl ? appl->getFullPath() : "<null>")
            << "\n";

    if (appl) {
        // These signals are emitted by CarlaGeneralPlatooningApp::onControlTick().
        // BridgeApp does not compute platoon control; it only forwards the latest values.
        const char* signals[] = {
            "desired_acceleration",       // m/s^2; controller acceleration command for logs/metadata.
            "desired_speed",              // m/s; controller target speed for logs/metadata.
            "has_control",                // 1 if this actor should receive active throttle/brake.
            "control_throttle",           // normalized CARLA throttle command.
            "control_brake",              // normalized CARLA brake command.
            "control_hand_brake",         // boolean CARLA hand-brake flag.
            "control_reverse",            // boolean CARLA reverse flag.
            "control_manual_gear_shift"   // boolean CARLA manual gearbox flag.
        };

        for (const char* sig : signals) {
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

    // Self-message used to coalesce several signal updates into one JSON payload.
    // Negative priority lets same-time control emissions arrive before the bridge sends.
    tick = new cMessage("bridge.send");
    tick->setSchedulingPriority(-1);
}

// Receives numeric OMNeT++ signal updates from the vehicle application module.
// Each signal updates one field in the pending control snapshot; sending is delayed/coalesced by tick.
void BridgeApp::receiveSignal(cComponent* src,
                              simsignal_t signalID,
                              double value,
                              cObject* details)
{
    (void)src;
    (void)details;
    Enter_Method_Silent(); // Required when a signal callback enters module context.

    const char* sigName = getSignalName(signalID);
    if (!sigName) return;

    if (std::strcmp(sigName, "desired_acceleration") == 0) {
        desired_acceleration = value; // m/s^2; kept in JSON for logging and debugging.
        seenAccel = true;
    }
    else if (std::strcmp(sigName, "desired_speed") == 0) {
        desired_speed = value; // m/s; metadata only, not a hidden speed controller here.
        seenSpeed = true;
    }
    else if (std::strcmp(sigName, "has_control") == 0) {
        controllerHasControl = (value != 0.0); // false usually means hold/stop/no active controller.
        seenHasControl = true;
    }
    else if (std::strcmp(sigName, "control_throttle") == 0) {
        controlThrottle = clamp01(value); // normalized CARLA actuator value.
        seenThrottle = true;
    }
    else if (std::strcmp(sigName, "control_brake") == 0) {
        controlBrake = clamp01(value); // normalized CARLA actuator value.
        seenBrake = true;
    }
    else if (std::strcmp(sigName, "control_hand_brake") == 0) {
        controlHandBrake = (value != 0.0);
        seenHandBrake = true;
    }
    else if (std::strcmp(sigName, "control_reverse") == 0) {
        controlReverse = (value != 0.0);
        seenReverse = true;
    }
    else if (std::strcmp(sigName, "control_manual_gear_shift") == 0) {
        controlManualGearShift = (value != 0.0);
        seenManualGearShift = true;
    }
    else {
        return;
    }

    if (!haveCompleteControlSnapshot()) return;

    controlUpdated = true;

    // Rate-limit JSON sending while preserving the most recent signal values.
    // Multiple emissions in one control tick should produce one CONTROL message.
    SimTime when = std::max(simTime(), nextAllowedSend);
    if (!tick->isScheduled()) {
        scheduleAt(when, tick);
    }
    else if (tick->getArrivalTime() > when) {
        cancelEvent(tick);
        scheduleAt(when, tick);
    }
}

// Sends a JSON string to CarlanetManager through the module gate.
// CarlanetManager forwards this to the Python bridge as a GENERIC_MESSAGE payload.
void BridgeApp::sendJson(const std::string& userDefinedJson)
{
    auto* msg = new cMessage("bridge.json");
    msg->addPar("json") = userDefinedJson.c_str();
    send(msg, "toManager");
}

// Handles the bridge send timer.
// Builds one CONTROL JSON object from the latest control snapshot and sends it to CarlanetManager.
void BridgeApp::handleMessage(cMessage* msg)
{
    if (msg == tick) {
        if (!haveCompleteControlSnapshot() || !controlUpdated) {
            controlUpdated = false;
            return;
        }

        std::ostringstream js;
        js.setf(std::ios::fixed);
        js << std::setprecision(6);

        // CONTROL payload consumed by pyCARLANeT.
        // pyCARLANeT should apply these actuator fields to CARLA; it should not decide platoon behavior.
        js << "{"
           << "\"msg_type\":\"CONTROL\","
           << "\"actor_id\":\"" << jsonEscape(actor_id) << "\","
           << "\"ctrl\":{"
           << "\"has_control\":" << (controllerHasControl ? "true" : "false") << ","
           << "\"throttle\":" << controlThrottle << ","
           << "\"brake\":" << controlBrake << ","
           << "\"hand_brake\":" << (controlHandBrake ? "true" : "false") << ","
           << "\"reverse\":" << (controlReverse ? "true" : "false") << ","
           << "\"manual_gear_shift\":" << (controlManualGearShift ? "true" : "false") << ","
           << "\"autopilot_steering\":true," // CARLA/Python keeps lateral steering on autopilot.
           << "\"desired_acceleration\":" << desired_acceleration << ","
           << "\"controller_acceleration\":" << desired_acceleration << ","
           << "\"desired_speed\":" << desired_speed
           << "}"
           << "}";

        EV_INFO << "[BridgeApp][handleMessage]"
                << " hop=BRIDGE_TO_MANAGER"
                << " simulation_time=" << SIMTIME_DBL(simTime())
                << " actor=" << actor_id
                << " has_control=" << (controllerHasControl ? 1 : 0)
                << " throttle=" << controlThrottle
                << " brake=" << controlBrake
                << " hand_brake=" << (controlHandBrake ? 1 : 0)
                << " desired_speed=" << desired_speed
                << " desired_acceleration=" << desired_acceleration
                << " autopilot_steering=1"
                << "\n";

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

// Releases the bridge send timer at the end of the simulation.
void BridgeApp::finish()
{
    if (tick) {
        cancelAndDelete(tick);
        tick = nullptr;
    }
}
