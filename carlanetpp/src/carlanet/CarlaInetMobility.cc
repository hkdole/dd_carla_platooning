#include "CarlaInetMobility.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>

#include "CarlanetManager.h"
#include "inet/common/Units.h"

using namespace omnetpp;
using namespace inet;

Define_Module(CarlaInetMobility);

static inline double deg2rad(double deg) { return deg * M_PI / 180.0; }

static inline void rot2d(double& x, double& y, double a) {
    const double c = std::cos(a), s = std::sin(a);
    const double X = x * c - y * s;
    const double Y = x * s + y * c;
    x = X;
    y = Y;
}

static inline void applyCanvasTransform(double& x, double& y,
                                        double scale, double rotRad,
                                        double offsetX, double offsetY,
                                        bool flipX, bool flipY)
{
    if (flipX) x = -x;
    if (flipY) y = -y;
    if (rotRad != 0.0) rot2d(x, y, rotRad);
    x = offsetX + scale * x;
    y = offsetY + scale * y;
}

static inline bool isBad(double v) {
    return std::isnan(v) || std::isinf(v) || std::fabs(v) > 1e9;
}

void CarlaInetMobility::initialize(int stage)
{
    MobilityBase::initialize(stage);

    if (stage == INITSTAGE_LOCAL) {
        if (auto* parent = getParentModule()) {
            if (parent->hasPar("actor_id")) actorId = parent->par("actor_id").stdstringValue();
        }

        carlaActorType = par("carlaActorType").stdstringValue();
        carlaActorConfiguration = check_and_cast<cValueMap*>(par("carlaActorConfiguration").objectValue());

        netOffX = par("sumoNetOffsetX").doubleValue();
        netOffY = par("sumoNetOffsetY").doubleValue();
        netOffSign = par("netOffsetSign").intValue();

        swapXY    = par("swapXY").boolValue();
        preRotDeg = par("preRotDeg").doubleValue();
        preRotRad = deg2rad(preRotDeg);

        scale  = par("scale").doubleValue();
        rotDeg = par("rotDeg").doubleValue();
        rotRad = deg2rad(rotDeg);
        offx   = par("offsetX").doubleValue();
        offy   = par("offsetY").doubleValue();
        flipX  = par("flipX").boolValue();
        flipY  = par("flipY").boolValue();

        carlaToSumoScale = par("carlaToSumoScale").doubleValue();
        deltaCanvasX     = par("deltaCanvasX").doubleValue();
        deltaCanvasY     = par("deltaCanvasY").doubleValue();

        // Register with manager
        if (auto* root = getSimulation()->getSystemModule()) {
            if (auto* mgrMod = root->getSubmodule("manager")) {
                if (auto* mgr = dynamic_cast<CarlanetManager*>(mgrMod))
                    mgr->registerMobilityModule(this);
            }
        }

        {
            std::ostringstream oss;
            oss << "[CarlaInetMobility][initialize]"
                << " hop=MOBILITY_LOCAL"
                << " simulation_time=" << SIMTIME_DBL(simTime())
                << " actor=" << (actorId.empty() ? "?" : actorId)
                << " flipX=" << (flipX ? 1 : 0)
                << " flipY=" << (flipY ? 1 : 0)
                << " rotDeg=" << rotDeg
                << " scale=" << scale
                << " offsetX=" << offx
                << " offsetY=" << offy
                << " netOffsetSign=" << netOffSign
                << " sumoNetOffsetX=" << netOffX
                << " sumoNetOffsetY=" << netOffY
                << " swapXY=" << (swapXY ? 1 : 0)
                << " preRotDeg=" << preRotDeg
                << " carlaToSumoScale=" << carlaToSumoScale
                << " deltaCanvasX=" << deltaCanvasX
                << " deltaCanvasY=" << deltaCanvasY
                << "\n";
            EV_INFO << oss.str();
        }
    }
}

void CarlaInetMobility::setInitialPosition()
{
    if (preInitialized) return;

    double px = 0.0, py = 0.0;
    if (auto* parent = getParentModule()) {
        cDisplayString ds = parent->getDisplayString();
        const char* sx = ds.getTagArg("p", 0);
        const char* sy = ds.getTagArg("p", 1);
        if (sx && *sx) px = std::atof(sx);
        if (sy && *sy) py = std::atof(sy);
    }

    lastPosition     = Coord(px, py, 0.0);
    lastVelocity     = Coord(0.0, 0.0, 0.0);
    lastAcceleration = Coord(0.0, 0.0, 0.0);
    lastOrientation  = Quaternion(EulerAngles(
        inet::units::values::rad(0.0),
        inet::units::values::rad(0.0),
        inet::units::values::rad(0.0)
    ));
    lastAngularVelocity = Quaternion(EulerAngles(
        inet::units::values::rad(0.0),
        inet::units::values::rad(0.0),
        inet::units::values::rad(0.0)
    ));
    lastAngularAcceleration = Quaternion(EulerAngles(
        inet::units::values::rad(0.0),
        inet::units::values::rad(0.0),
        inet::units::values::rad(0.0)
    ));
    emitMobilityStateChangedSignal();
}

void CarlaInetMobility::preInitialize(const Coord& p, const Coord& v, const Quaternion& q)
{
    preInitialize(p, v, Coord(0.0, 0.0, 0.0), q);
}

void CarlaInetMobility::preInitialize(const Coord& p,
                                      const Coord& v,
                                      const Coord& a,
                                      const Quaternion& q,
                                      const Quaternion& w,
                                      const Quaternion& alpha)
{
    preInitialized = true;
    lastPosition = p;
    lastVelocity = v;
    lastAcceleration = a;
    lastOrientation = q;
    lastAngularVelocity = w;
    lastAngularAcceleration = alpha;
}

void CarlaInetMobility::nextPosition(const Coord& carlaPosition,
                                     const Coord& carlaVelocity,
                                     const Quaternion& carlaOrientation)
{
    nextPosition(carlaPosition,
                 carlaVelocity,
                 Coord(0.0, 0.0, 0.0),
                 carlaOrientation);
}

void CarlaInetMobility::nextPosition(const Coord& carlaPosition,
                                     const Coord& carlaVelocity,
                                     const Coord& carlaAcceleration,
                                     const Quaternion& carlaOrientation,
                                     const Quaternion& carlaAngularVelocity,
                                     const Quaternion& carlaAngularAcceleration)
{
    if (!preInitialized) {
        preInitialize(carlaPosition,
                      carlaVelocity,
                      carlaAcceleration,
                      carlaOrientation,
                      carlaAngularVelocity,
                      carlaAngularAcceleration);
    }

    double x = carlaPosition.x;
    double y = carlaPosition.y;

    if (netOffSign != 0) {
        x = x - netOffSign * netOffX;
        y = y - netOffSign * netOffY;
    }

    if (swapXY) std::swap(x, y);
    if (preRotRad != 0.0) rot2d(x, y, preRotRad);

    x *= carlaToSumoScale;
    y *= carlaToSumoScale;

    applyCanvasTransform(x, y, scale, rotRad, offx, offy, flipX, flipY);
    x += deltaCanvasX;
    y += deltaCanvasY;

    if (isBad(x) || isBad(y)) {
        std::ostringstream oss;
        oss << "[CarlaInetMobility][nextPosition]"
            << " hop=PY_TO_MOBILITY"
            << " simulation_time=" << SIMTIME_DBL(simTime())
            << " actor=" << (actorId.empty() ? "?" : actorId)
            << " warning=bad_coordinate"
            << " x=" << x
            << " y=" << y
            << "\n";
        EV_WARN << oss.str();
        return;
    }

    lastPosition = Coord(x, y, carlaPosition.z);

    // Raw state from CARLA. No derived gap/headway/controller logic belongs here.
    lastVelocity = carlaVelocity;
    lastAcceleration = carlaAcceleration;
    lastOrientation = carlaOrientation;
    lastAngularVelocity = carlaAngularVelocity;
    lastAngularAcceleration = carlaAngularAcceleration;

    emitMobilityStateChangedSignal();

    if (auto* parent = getParentModule()) {
        cDisplayString ds = parent->getDisplayString();
        ds.setTagArg("p", 0, (long)std::lround(lastPosition.x));
        ds.setTagArg("p", 1, (long)std::lround(lastPosition.y));
        parent->setDisplayString(ds);
    }
}

void CarlaInetMobility::handleSelfMessage(cMessage*) { }