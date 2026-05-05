#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "carla/platooning/state/ControlMode.h"
#include "carla/platooning/state/PlatoonTypes.h"

namespace carla {

// Explicit NaN helper used for fields that are unknown rather than truly zero.
inline double controllerNaN()
{
    return std::numeric_limits<double>::quiet_NaN();
}

// One neighbor input used by the controller layer.
// These values should already be longitudinal/scalar values, not raw world-frame x/y components.
struct ControllerNeighborInput {
    bool valid = false;

    // Longitudinal/scalar speed in m/s.
    // This must use the same convention as ControllerInputs::egoSpeed.
    double speed = 0.0;

    // Signed longitudinal acceleration in m/s^2.
    // NaN means unknown; 0.0 means known zero acceleration.
    double actualAcceleration = std::numeric_limits<double>::quiet_NaN();

    // Advertised controller acceleration from V2V beacons in m/s^2.
    // Used for Plexe-parity CACC feed-forward when useControllerAcceleration is true.
    double controllerAcceleration = std::numeric_limits<double>::quiet_NaN();

    // Bumper-to-bumper longitudinal gap in meters.
    // This should not be raw center-to-center distance.
    double distance = -1.0;
};

// Complete input bundle consumed by the longitudinal controller dispatcher.
// CarlaGeneralPlatooningApp builds this from role, control mode, formation, and V2V beacon cache.
struct ControllerInputs {
    ControlMode controlMode = ControlMode::HOLD;
    ActiveController activeController = ActiveController::UNKNOWN;

    // Ego longitudinal speed in m/s.
    double egoSpeed = 0.0;

    // Reference/cruise speed in m/s.
    // Leaders consume this as a real cruise target. For followers, this is mostly fallback/reference.
    double targetSpeed = 0.0;

    // Desired bumper gap in meters.
    //
    // CDS / Plexe join-at-back parity:
    //   targetGap = standstillDistance
    //
    // CTS / headway experiments:
    //   targetGap = standstillDistance + headway * relevantSpeed
    double targetGap = 5.0;
    double standstillDistance = 5.0;
    double headway = 0.5;

    // Joiner approach and exported-speed limiting parameters.
    double approachDeltaV = 0.0;
    double maxClosureSpeed = 1.0;

    // Controller gains.
    double kGap = 0.2;       // spacing-error gain
    double kDv = 0.7;        // relative-speed gain
    double kSpeedP = 0.7;    // cruise-control speed gain
    double kAccFF = 1.0;     // optional acceleration feed-forward gain, reserved for non-Plexe variants
    double kLeaderDv = 0.05; // optional leader-relative speed gain, reserved for non-Plexe variants
    double kGapSpeed = 0.03; // gap-error to exported/reference-speed bias

    // Controller acceleration limits in m/s^2.
    double aMin = -4.0;
    double aMax = 2.5;

    // Plexe/SUMO MSCFModel_CC CACC parameters.
    //
    // Upstream-style defaults:
    //   caccC1     = 0.5
    //   caccXi     = 1.0
    //   caccOmegaN = 0.2
    double caccC1 = 0.5;
    double caccXi = 1.0;
    double caccOmegaN = 0.2;

    // Plexe-parity feed-forward switch.
    //
    // true:
    //   use predecessor/leader controllerAcceleration from beacons when available.
    //
    // false:
    //   prefer measured physical actualAcceleration from CARLA state.
    //
    // For Plexe parity, true is usually correct. For more physical CARLA-only
    // experiments, false may be more appropriate.
    bool useControllerAcceleration = true;

    ControllerNeighborInput predecessor;
    ControllerNeighborInput leader;

    // Used only during joiner approach phases before the joiner becomes a normal follower.
    ControllerNeighborInput fakeFront;
    ControllerNeighborInput fakeLeader;
};

// Plexe/SUMO CACC alpha gains.
struct PlexeCaccGains {
    double alpha1 = 0.5;
    double alpha2 = 0.5;
    double alpha3 = -0.3;
    double alpha4 = -0.1;
    double alpha5 = -0.04;
};

// Full trace of the CACC calculation.
// This is useful for diagnosing string instability and sudden acceleration spikes.
struct PlexeCaccTrace {
    double egoSpeed = 0.0;
    double predSpeed = 0.0;
    double leaderSpeed = 0.0;

    double bumperGap = 0.0;
    double spacing = 0.0;

    double epsilon = 0.0;
    double epsilonDot = 0.0;
    double leaderClosingRate = 0.0;

    double predAccelerationFF = 0.0;
    double leaderAccelerationFF = 0.0;

    double alpha1 = 0.0;
    double alpha2 = 0.0;
    double alpha3 = 0.0;
    double alpha4 = 0.0;
    double alpha5 = 0.0;

    double termPredAccel = 0.0;
    double termLeaderAccel = 0.0;
    double termPredSpeed = 0.0;
    double termLeaderSpeed = 0.0;
    double termGap = 0.0;

    double rawAcceleration = 0.0;
    double clippedAcceleration = 0.0;
};

// Safe clamp used throughout controller code.
// Non-finite values are converted to 0.0 before clamping.
inline double clampControllerValue(double value, double lo, double hi)
{
    if (!std::isfinite(value)) value = 0.0;
    if (!std::isfinite(lo)) lo = -std::numeric_limits<double>::infinity();
    if (!std::isfinite(hi)) hi = std::numeric_limits<double>::infinity();
    if (lo > hi) std::swap(lo, hi);

    return std::max(lo, std::min(hi, value));
}

// Returns fallback when value is NaN or infinite.
inline double finiteOr(double value, double fallback)
{
    return std::isfinite(value) ? value : fallback;
}

// Positive means ego is slower than neighbor.
inline double speedErrorToNeighbor(double egoSpeed, double neighborSpeed)
{
    return neighborSpeed - egoSpeed;
}

// Positive means ego is closing in on neighbor.
inline double closingRateToNeighbor(double egoSpeed, double neighborSpeed)
{
    return egoSpeed - neighborSpeed;
}

// Selects feed-forward acceleration from V2V data.
// The controllerAcceleration path gives Plexe-style ideal feed-forward.
// The actualAcceleration path gives CARLA-measured feed-forward.
inline double chooseFeedforwardAcceleration(
    const ControllerNeighborInput& n,
    bool useControllerAcceleration)
{
    if (!n.valid) return 0.0;

    if (useControllerAcceleration && std::isfinite(n.controllerAcceleration)) {
        return n.controllerAcceleration;
    }

    if (std::isfinite(n.actualAcceleration)) {
        return n.actualAcceleration;
    }

    if (std::isfinite(n.controllerAcceleration)) {
        return n.controllerAcceleration;
    }

    return 0.0;
}

// Computes Plexe/SUMO PATH-CACC alpha gains.
inline PlexeCaccGains computePlexeCaccGains(const ControllerInputs& in)
{
    const double c1 = clampControllerValue(finiteOr(in.caccC1, 0.5), 0.0, 1.0);

    // xi < 1 produces an underdamped form that this real-valued transcription does not model.
    // Clamp to 1.0 for stable Plexe-parity defaults.
    const double xi = std::max(1.0, finiteOr(in.caccXi, 1.0));

    const double omegaN = std::max(0.0, finiteOr(in.caccOmegaN, 0.2));
    const double root = std::sqrt(std::max(0.0, xi * xi - 1.0));

    PlexeCaccGains g;
    g.alpha1 = 1.0 - c1;
    g.alpha2 = c1;
    g.alpha3 = -(2.0 * xi - c1 * (xi + root)) * omegaN;
    g.alpha4 = -c1 * (xi + root) * omegaN;
    g.alpha5 = -omegaN * omegaN;
    return g;
}

// Computes a Plexe-style cruise-control acceleration command.
inline double computePlexeCcAcceleration(const ControllerInputs& in, double desiredSpeed)
{
    const double ego = std::max(0.0, finiteOr(in.egoSpeed, 0.0));
    const double target = std::max(0.0, finiteOr(desiredSpeed, ego));
    const double raw = finiteOr(in.kSpeedP, 0.7) * (target - ego);

    return clampControllerValue(raw, in.aMin, in.aMax);
}

// Optional ACC fallback law.
// This is useful when predecessor data exists but no leader-equivalent data is available.
inline double computePlexeAccAcceleration(
    const ControllerInputs& in,
    const ControllerNeighborInput& front,
    double bumperGap,
    double desiredGap)
{
    if (!front.valid || bumperGap < 0.0) {
        return computePlexeCcAcceleration(in, in.targetSpeed);
    }

    const double T = std::max(0.1, finiteOr(in.headway, 0.5));
    const double lambda = std::max(0.0, finiteOr(in.kGap, 0.2));

    const double egoSpeed = std::max(0.0, finiteOr(in.egoSpeed, 0.0));
    const double frontSpeed = std::max(0.0, finiteOr(front.speed, egoSpeed));

    const double error = std::max(0.0, finiteOr(bumperGap, 0.0)) -
                         std::max(0.0, finiteOr(desiredGap, in.targetGap));

    const double closingRate = egoSpeed - frontSpeed;

    // Positive error means too far back. Positive closingRate means approaching too fast.
    const double raw = (lambda * error - closingRate) / T;

    return clampControllerValue(raw, in.aMin, in.aMax);
}

// Computes the full CACC trace using Plexe/SUMO sign conventions.
//
// epsilon = spacing - gap
//   gap too large  -> epsilon < 0 -> alpha5 * epsilon > 0 -> accelerate
//   gap too small  -> epsilon > 0 -> alpha5 * epsilon < 0 -> brake
//
// epsilonDot = egoSpeed - predecessorSpeed
//   ego slower     -> epsilonDot < 0 -> alpha3 * epsilonDot > 0 -> accelerate
//   ego faster     -> epsilonDot > 0 -> alpha3 * epsilonDot < 0 -> brake
inline PlexeCaccTrace computePlexeCaccTrace(
    const ControllerInputs& in,
    const ControllerNeighborInput& front,
    const ControllerNeighborInput& leader,
    double bumperGap,
    double spacing)
{
    PlexeCaccTrace t;

    const PlexeCaccGains g = computePlexeCaccGains(in);

    const double egoSpeed = std::max(0.0, finiteOr(in.egoSpeed, 0.0));
    const double predSpeed = front.valid
        ? std::max(0.0, finiteOr(front.speed, egoSpeed))
        : egoSpeed;
    const double leaderSpeed = leader.valid
        ? std::max(0.0, finiteOr(leader.speed, predSpeed))
        : predSpeed;

    const double safeGap = std::max(0.0, finiteOr(bumperGap, 0.0));
    const double safeSpacing = std::max(0.0, finiteOr(spacing, in.targetGap));

    const double predAcceleration =
        chooseFeedforwardAcceleration(front, in.useControllerAcceleration);

    const double leaderAcceleration =
        chooseFeedforwardAcceleration(leader, in.useControllerAcceleration);

    const double epsilon = safeSpacing - safeGap;
    const double epsilonDot = closingRateToNeighbor(egoSpeed, predSpeed);
    const double leaderClosingRate = closingRateToNeighbor(egoSpeed, leaderSpeed);

    t.egoSpeed = egoSpeed;
    t.predSpeed = predSpeed;
    t.leaderSpeed = leaderSpeed;

    t.bumperGap = safeGap;
    t.spacing = safeSpacing;

    t.epsilon = epsilon;
    t.epsilonDot = epsilonDot;
    t.leaderClosingRate = leaderClosingRate;

    t.predAccelerationFF = predAcceleration;
    t.leaderAccelerationFF = leaderAcceleration;

    t.alpha1 = g.alpha1;
    t.alpha2 = g.alpha2;
    t.alpha3 = g.alpha3;
    t.alpha4 = g.alpha4;
    t.alpha5 = g.alpha5;

    t.termPredAccel = g.alpha1 * predAcceleration;
    t.termLeaderAccel = g.alpha2 * leaderAcceleration;
    t.termPredSpeed = g.alpha3 * epsilonDot;
    t.termLeaderSpeed = g.alpha4 * leaderClosingRate;
    t.termGap = g.alpha5 * epsilon;

    t.rawAcceleration =
        t.termPredAccel +
        t.termLeaderAccel +
        t.termPredSpeed +
        t.termLeaderSpeed +
        t.termGap;

    t.clippedAcceleration =
        clampControllerValue(t.rawAcceleration, in.aMin, in.aMax);

    return t;
}

// Computes only the clipped CACC acceleration.
inline double computePlexeCaccAcceleration(
    const ControllerInputs& in,
    const ControllerNeighborInput& front,
    const ControllerNeighborInput& leader,
    double bumperGap,
    double spacing)
{
    return computePlexeCaccTrace(
        in,
        front,
        leader,
        bumperGap,
        spacing
    ).clippedAcceleration;
}

// Computes a reference/export speed from front-vehicle speed and gap error.
// This is not the authoritative controller command; desiredAcceleration is.
inline double computeExportSpeedFromFrontGap(
    const ControllerInputs& in,
    const ControllerNeighborInput& front,
    double bumperGap,
    double spacing)
{
    const double egoSpeed = std::max(0.0, finiteOr(in.egoSpeed, 0.0));
    const double frontSpeed = front.valid
        ? std::max(0.0, finiteOr(front.speed, egoSpeed))
        : egoSpeed;

    const double safeGap = std::max(0.0, finiteOr(bumperGap, spacing));
    const double safeSpacing = std::max(0.0, finiteOr(spacing, in.targetGap));
    const double gapError = safeGap - safeSpacing;

    const double closureLimit =
        std::max(0.0, finiteOr(in.maxClosureSpeed, 1.0));

    const double bias = clampControllerValue(
        finiteOr(in.kGapSpeed, 0.03) * gapError,
        -closureLimit,
        closureLimit);

    return std::max(0.0, frontSpeed + bias);
}

} // namespace carla