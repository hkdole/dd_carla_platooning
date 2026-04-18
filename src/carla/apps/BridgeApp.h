// BridgeApp.h
#ifndef __CARLA_BRIDGE_APP_H_
#define __CARLA_BRIDGE_APP_H_

#include <string>
#include <omnetpp.h>

class BridgeApp : public omnetpp::cSimpleModule, public omnetpp::cListener
{
  protected:
    // Parameters
    std::string      actor_id;
    int              send_every_ms = 50;
    omnetpp::SimTime tickInterval;

    // Self-message (coalesced send)
    omnetpp::cMessage* tick = nullptr;

    // Latest commands coming from the controller app (CarlaJoinAtBackApp, etc.)
    double desired_acceleration = 0.0;
    double desired_speed = 0.0;

    // Rate limit & state
    omnetpp::SimTime nextAllowedSend;
    bool controlUpdated = false;

    // Gate "has_control" until both signals have been seen at least once
    bool seenAccel = false;
    bool seenSpeed = false;
    bool controllerHasControl = false;
    bool seenHasControl = false;

  protected:
    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage* msg) override;
    virtual void finish() override;

    // Listener overload used by emit(signal,double)
    virtual void receiveSignal(omnetpp::cComponent* src,
                               omnetpp::simsignal_t signalID,
                               double value,
                               omnetpp::cObject* details) override;

    // Low-level helpers
    void sendJson(const std::string& userDefinedJson);

    // Utility
    bool hasControl() const { return controllerHasControl; }
    bool haveFullSnapshot() const { return seenAccel && seenSpeed && seenHasControl; }
};

#endif
