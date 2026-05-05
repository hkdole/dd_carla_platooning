#pragma once

#include <omnetpp.h>
#include <string>

/**
 * BridgeApp forwards OMNeT++ control outputs to CarlanetManager.
 *
 * This module is the boundary between the OMNeT++ platooning logic and the
 * CARLA runtime bridge. It listens for control signals emitted by
 * CarlaGeneralPlatooningApp, batches the most recent values, and sends one
 * CONTROL JSON payload to CarlanetManager at a fixed rate.
 *
 * This module does not implement V2V logic, platoon membership decisions,
 * join/exit maneuvers, or longitudinal controllers. Those belong in the
 * platooning application, maneuver, and controller layers. pyCARLANeT should
 * apply the final actuator fields exactly as received.
 */
class BridgeApp : public omnetpp::cSimpleModule, public omnetpp::cListener {
  protected:
    std::string actor_id;              // CARLA actor identifier controlled by this OMNeT++ node.
    int send_every_ms = 50;            // ms; minimum period between CONTROL messages to CarlanetManager.
    omnetpp::SimTime tickInterval;     // OMNeT++ time equivalent of send_every_ms.
    omnetpp::SimTime nextAllowedSend;  // Earliest simulation time when another CONTROL payload may be sent.
    omnetpp::cMessage* tick = nullptr; // Self-message timer used to flush pending control updates.

    // Latest semantic controller outputs from CarlaGeneralPlatooningApp.
    // desired_acceleration is the controller's longitudinal acceleration command.
    // desired_speed is retained for logging/debugging and should not become a hidden Python controller.
    double desired_acceleration = 0.0; // m/s^2; clipped controller acceleration command.
    double desired_speed = 0.0;        // m/s; reference/debug speed target from OMNeT++.
    bool controllerHasControl = false; // True when OMNeT++ currently owns longitudinal actuation.

    // Latest final CARLA actuator commands from CarlaGeneralPlatooningApp.
    // These are already converted from desired acceleration by the OMNeT++ actuator tracker.
    double controlThrottle = 0.0;      // CARLA throttle command in [0, 1].
    double controlBrake = 0.0;         // CARLA brake command in [0, 1].
    bool controlHandBrake = false;     // CARLA hand-brake flag.
    bool controlReverse = false;       // CARLA reverse-gear flag.
    bool controlManualGearShift = false; // CARLA manual gear-shift flag.

    bool controlUpdated = false;       // True after any subscribed control signal changes.

    // Signal-completeness flags.
    // A CONTROL message is only safe to send after all required fields have been observed at least once.
    bool seenAccel = false;
    bool seenSpeed = false;
    bool seenHasControl = false;
    bool seenThrottle = false;
    bool seenBrake = false;
    bool seenHandBrake = false;
    bool seenReverse = false;
    bool seenManualGearShift = false;

  protected:
    /**
     * Initializes signal subscriptions, timing state, and actor metadata.
     * Called once by OMNeT++ when the module is created.
     */
    void initialize() override;

    /**
     * Handles the bridge self-message timer.
     * Used to periodically flush the most recent complete control snapshot.
     */
    void handleMessage(omnetpp::cMessage* msg) override;

    /**
     * Releases owned timer messages and performs final cleanup.
     * Called by OMNeT++ at simulation shutdown.
     */
    void finish() override;

    /**
     * Receives subscribed OMNeT++ signals from CarlaGeneralPlatooningApp.
     * Each signal updates one field of the pending CONTROL snapshot.
     */
    void receiveSignal(omnetpp::cComponent* src,
                       omnetpp::simsignal_t signalID,
                       double value,
                       omnetpp::cObject* details) override;

    /**
     * Sends a raw JSON string to CarlanetManager.
     * CarlanetManager forwards it through the ZMQ bridge to pyCARLANeT.
     */
    void sendJson(const std::string& userDefinedJson);

    /**
     * Returns true once all required control fields have been seen.
     * Prevents sending partially initialized throttle/brake/control snapshots.
     */
    bool haveCompleteControlSnapshot() const;

    /**
     * Escapes a string so it can be safely embedded in JSON.
     * Used for actor ids and other string fields in CONTROL payloads.
     */
    static std::string jsonEscape(const std::string& s);
};
