#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "carla/platooning/state/ObservedVehicleState.h"

namespace carla {

enum class LateralSteeringStatus {
    Valid,
    ExpectedPreviewUnavailable,
    StructuralFailure,
};

struct LateralSteeringResult {
    LateralSteeringStatus status = LateralSteeringStatus::StructuralFailure;
    double steeringRad = 0.0;
    std::string reason;
};

/** Resolve only an expected preview loss; structural failures are not valid input. */
double resolveExpectedPreviewSteering(
    const LateralSteeringResult& result,
    bool lastValidSteeringAvailable,
    double lastValidSteeringRad);

/**
 * OMNeT++-owned route-following controller producing a physical front-wheel
 * steering angle in radians.
 *
 * The shared-route progress carried in ObservedVehicleState identifies the
 * correct occurrence of a potentially self-near or looping immutable route.
 * Projection geometry and steering remain computed locally in this controller.
 */
class LateralController {
public:
    void initialize(
        const std::vector<LateralRoutePoint>& route,
        double wheelbaseM,
        double maximumFrontWheelSteeringRad);

    bool isInitialized() const
    {
        return initialized_;
    }

    LateralSteeringResult computeSteering(
        const ObservedVehicleState& state,
        double referenceLateralOffsetM = 0.0);

    void setLookaheadM(double lookaheadM) { lookaheadM_ = lookaheadM; }

private:
    struct RouteSample {
        double xM = 0.0;
        double yM = 0.0;
        double yawRad = 0.0;
    };

    struct RouteProjection {
        double progressM = 0.0;
        double lateralOffsetM = 0.0;
        double headingErrorRad = 0.0;
        std::size_t segmentIndex = 0;
        double distanceM = 0.0;
    };

    RouteSample sampleAt(double routeSM) const;

    RouteProjection projectPose(
        double xM,
        double yM,
        double yawRad,
        double observedRouteProgressM) const;

    double computeSteeringOrThrow(
        const ObservedVehicleState& state,
        double referenceLateralOffsetM) const;

    static double wrapToPi(double angleRad);

    std::vector<LateralRoutePoint> route_;
    std::vector<double> cumulativeDistanceM_;
    double wheelbaseM_ = 0.0;
    double maximumFrontWheelSteeringRad_ = 0.0;
    bool initialized_ = false;
    double lookaheadM_ = 6.0;
};

} // namespace carla
