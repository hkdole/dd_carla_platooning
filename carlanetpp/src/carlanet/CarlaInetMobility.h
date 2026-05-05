#pragma once

#include <omnetpp.h>
#include <cmath>
#include <string>

#include "inet/mobility/base/MobilityBase.h"
#include "inet/common/geometry/common/Quaternion.h"
#include "inet/common/geometry/common/Coord.h"

class CarlaInetMobility : public inet::MobilityBase {
  protected:
    // STATIC canvas transform (no overlay)
    double scale = 1.0;
    double rotDeg = 0.0, rotRad = 0.0;
    double offx = 0.0, offy = 0.0;
    bool   flipX = false, flipY = false;

    // Optional frame alignment
    double netOffX = 0.0, netOffY = 0.0;
    int    netOffSign = 0;      // 0 disables offset application
    bool   swapXY = false;
    double preRotDeg = 0.0, preRotRad = 0.0;

    // Unit & UI nudge
    double carlaToSumoScale = 1.0;
    double deltaCanvasX = 0.0, deltaCanvasY = 0.0;

    bool preInitialized = false;
    inet::Coord       lastPosition{0, 0, 0};
    inet::Coord       lastVelocity{0, 0, 0};
    inet::Coord       lastAcceleration{0, 0, 0};
    inet::Quaternion  lastOrientation;
    inet::Quaternion  lastAngularVelocity;
    inet::Quaternion  lastAngularAcceleration;
    double curvatureRadius = 1000000.0; // meters

    std::string actorId;
    std::string carlaActorType;
    omnetpp::cValueMap* carlaActorConfiguration = nullptr;

  protected:
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void setInitialPosition() override;
    virtual void handleSelfMessage(omnetpp::cMessage *msg) override;

  public:
    // Backward-compatible overloads for any call sites not yet updated.
    void preInitialize(const inet::Coord& p, const inet::Coord& v, const inet::Quaternion& q);
    void nextPosition(const inet::Coord& p, const inet::Coord& v, const inet::Quaternion& q);

    // Raw CARLA state path.
    void preInitialize(const inet::Coord& p,
                       const inet::Coord& v,
                       const inet::Coord& a,
                       const inet::Quaternion& q,
                       const inet::Quaternion& w = inet::Quaternion(),
                       const inet::Quaternion& alpha = inet::Quaternion());

    void nextPosition(const inet::Coord& p,
                      const inet::Coord& v,
                      const inet::Coord& a,
                      const inet::Quaternion& q,
                      const inet::Quaternion& w = inet::Quaternion(),
                      const inet::Quaternion& alpha = inet::Quaternion());

    virtual const inet::Coord& getCurrentPosition() override { return lastPosition; }
    virtual const inet::Coord& getCurrentVelocity() override { return lastVelocity; }
    virtual const inet::Coord& getCurrentAcceleration() override { return lastAcceleration; }
    virtual const inet::Quaternion& getCurrentAngularPosition() override { return lastOrientation; }
    virtual const inet::Quaternion& getCurrentAngularVelocity() override { return lastAngularVelocity; }
    virtual const inet::Quaternion& getCurrentAngularAcceleration() override { return lastAngularAcceleration; }

    std::string getCarlaActorType() const { return carlaActorType; }
    const omnetpp::cValueMap* getCarlaActorConfiguration() const { return carlaActorConfiguration; }

    void setCurvatureRadius(double r) { if (std::isfinite(r) && r > 0.0) curvatureRadius = r; }
    double getCurvatureRadius() const { return curvatureRadius; }
};