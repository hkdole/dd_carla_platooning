#include "carla/platooning/controller/LateralController.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace carla {
namespace {

class ExpectedPreviewUnavailable : public std::runtime_error {
public:
    explicit ExpectedPreviewUnavailable(const char* reason)
        : std::runtime_error(reason)
    {
    }
};

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kLookaheadM = 6.0;
constexpr double kMinimumControlSpeedMps = 0.5;
constexpr double kMinimumTargetForwardM = 0.05;
constexpr double kMinimumSegmentLengthM = 1e-6;

// pyCARLANeT reports progress on the same immutable shared-route polyline.
// This window identifies the correct local occurrence without delegating the
// actual geometric projection or steering calculation to the bridge.
constexpr double kObservedProgressSearchWindowM = 12.0;
constexpr double kMaximumProjectionDistanceM = 8.0;
constexpr double kMaximumProgressDisagreementM = 4.0;

bool isFiniteRoutePoint(const LateralRoutePoint& point)
{
    return std::isfinite(point.xM) &&
        std::isfinite(point.yM) &&
        std::isfinite(point.yawRad);
}

double clampSymmetric(double value, double limit)
{
    if (!std::isfinite(value) || !std::isfinite(limit) || limit <= 0.0) {
        throw std::runtime_error(
            "non-finite lateral steering result or steering limit");
    }

    return std::max(-limit, std::min(limit, value));
}

double segmentYawRad(
    const LateralRoutePoint& first,
    const LateralRoutePoint& second)
{
    const double dx = second.xM - first.xM;
    const double dy = second.yM - first.yM;
    const double length = std::hypot(dx, dy);

    if (!std::isfinite(length) || length <= kMinimumSegmentLengthM) {
        throw std::runtime_error(
            "immutable route contains an unusable tangent segment");
    }

    return std::atan2(dy, dx);
}

} // namespace

double resolveExpectedPreviewSteering(
    const LateralSteeringResult& result,
    bool lastValidSteeringAvailable,
    double lastValidSteeringRad)
{
    if (result.status !=
        LateralSteeringStatus::ExpectedPreviewUnavailable) {
        throw std::invalid_argument(
            "lateral preview fallback requires expected unavailability");
    }
    if (lastValidSteeringAvailable &&
        !std::isfinite(lastValidSteeringRad)) {
        throw std::invalid_argument(
            "last valid lateral steering must be finite");
    }
    return lastValidSteeringAvailable ? lastValidSteeringRad : 0.0;
}

double LateralController::wrapToPi(double angleRad)
{
    return std::atan2(std::sin(angleRad), std::cos(angleRad));
}

void LateralController::initialize(
    const std::vector<LateralRoutePoint>& route,
    double wheelbaseM,
    double maximumFrontWheelSteeringRad)
{
    if (route.size() < 2 ||
        !std::isfinite(wheelbaseM) || wheelbaseM <= 0.0 ||
        !std::isfinite(maximumFrontWheelSteeringRad) ||
        maximumFrontWheelSteeringRad <= 0.0) {
        throw std::invalid_argument(
            "invalid immutable route or CARLA steering metadata");
    }

    // Validate and build replacement state locally. A failed reinitialization
    // must not partially overwrite a previously valid controller.
    std::vector<LateralRoutePoint> validatedRoute = route;
    std::vector<double> cumulativeDistanceM(validatedRoute.size(), 0.0);

    if (!isFiniteRoutePoint(validatedRoute.front())) {
        throw std::invalid_argument(
            "immutable route contains a non-finite point");
    }

    for (std::size_t i = 1; i < validatedRoute.size(); ++i) {
        if (!isFiniteRoutePoint(validatedRoute[i])) {
            throw std::invalid_argument(
                "immutable route contains a non-finite point");
        }

        const double dx =
            validatedRoute[i].xM - validatedRoute[i - 1].xM;
        const double dy =
            validatedRoute[i].yM - validatedRoute[i - 1].yM;
        const double segmentLength = std::hypot(dx, dy);

        if (!std::isfinite(segmentLength) ||
            segmentLength <= kMinimumSegmentLengthM) {
            throw std::invalid_argument(
                "immutable route contains a degenerate segment");
        }

        cumulativeDistanceM[i] =
            cumulativeDistanceM[i - 1] + segmentLength;

        if (!std::isfinite(cumulativeDistanceM[i])) {
            throw std::invalid_argument(
                "immutable route length is non-finite");
        }
    }

    route_ = std::move(validatedRoute);
    cumulativeDistanceM_ = std::move(cumulativeDistanceM);
    wheelbaseM_ = wheelbaseM;
    maximumFrontWheelSteeringRad_ =
        maximumFrontWheelSteeringRad;
    initialized_ = true;
}

LateralController::RouteSample LateralController::sampleAt(
    double routeSM) const
{
    if (!std::isfinite(routeSM)) {
        throw std::runtime_error(
            "non-finite route sample distance");
    }

    const double boundedS = std::max(
        0.0,
        std::min(cumulativeDistanceM_.back(), routeSM));

    const auto upper = std::upper_bound(
        cumulativeDistanceM_.begin(),
        cumulativeDistanceM_.end(),
        boundedS);

    std::size_t lo = 0;
    std::size_t hi = 1;

    if (upper == cumulativeDistanceM_.begin()) {
        lo = 0;
        hi = 1;
    }
    else if (upper == cumulativeDistanceM_.end()) {
        hi = route_.size() - 1;
        lo = hi - 1;
    }
    else {
        hi = static_cast<std::size_t>(
            upper - cumulativeDistanceM_.begin());
        lo = hi - 1;
    }

    const double segmentLength =
        cumulativeDistanceM_[hi] - cumulativeDistanceM_[lo];
    const double ratio = std::max(
        0.0,
        std::min(
            1.0,
            (boundedS - cumulativeDistanceM_[lo]) /
                segmentLength));

    // Ordered X/Y geometry is authoritative for travel direction.
    const double tangentYaw =
        segmentYawRad(route_[lo], route_[hi]);

    return {
        route_[lo].xM +
            ratio * (route_[hi].xM - route_[lo].xM),
        route_[lo].yM +
            ratio * (route_[hi].yM - route_[lo].yM),
        tangentYaw,
    };
}

LateralController::RouteProjection LateralController::projectPose(
    double xM,
    double yM,
    double yawRad,
    double observedRouteProgressM) const
{
    if (!std::isfinite(xM) ||
        !std::isfinite(yM) ||
        !std::isfinite(yawRad) ||
        !std::isfinite(observedRouteProgressM)) {
        throw std::runtime_error(
            "non-finite CARLA pose or shared-route progress");
    }

    if (observedRouteProgressM < 0.0 ||
        observedRouteProgressM > cumulativeDistanceM_.back()) {
        throw std::runtime_error(
            "shared-route progress is outside immutable route bounds");
    }

    const double searchStartM = std::max(
        0.0,
        observedRouteProgressM -
            kObservedProgressSearchWindowM);
    const double searchEndM = std::min(
        cumulativeDistanceM_.back(),
        observedRouteProgressM +
            kObservedProgressSearchWindowM);

    const auto firstPointIterator = std::lower_bound(
        cumulativeDistanceM_.begin(),
        cumulativeDistanceM_.end(),
        searchStartM);
    const auto lastPointIterator = std::upper_bound(
        cumulativeDistanceM_.begin(),
        cumulativeDistanceM_.end(),
        searchEndM);

    std::size_t firstSegment = 0;
    if (firstPointIterator != cumulativeDistanceM_.begin()) {
        firstSegment = static_cast<std::size_t>(
            firstPointIterator -
            cumulativeDistanceM_.begin() - 1);
    }

    std::size_t lastSegment = route_.size() - 2;
    if (lastPointIterator != cumulativeDistanceM_.end()) {
        lastSegment = static_cast<std::size_t>(
            lastPointIterator -
            cumulativeDistanceM_.begin());
        lastSegment = std::min(
            lastSegment,
            route_.size() - 2);
    }

    if (firstSegment > lastSegment) {
        throw std::runtime_error(
            "shared-route progress produced an empty projection window");
    }

    double bestScore =
        std::numeric_limits<double>::infinity();
    RouteProjection best;
    bool found = false;

    for (std::size_t i = firstSegment;
         i <= lastSegment;
         ++i) {
        const double ax = route_[i].xM;
        const double ay = route_[i].yM;
        const double vx = route_[i + 1].xM - ax;
        const double vy = route_[i + 1].yM - ay;
        const double segmentLengthSquared =
            vx * vx + vy * vy;

        if (segmentLengthSquared <=
            kMinimumSegmentLengthM *
                kMinimumSegmentLengthM) {
            continue;
        }

        const double segmentLength =
            std::sqrt(segmentLengthSquared);
        const double unboundedRatio =
            ((xM - ax) * vx + (yM - ay) * vy) /
            segmentLengthSquared;
        const double ratio = std::max(
            0.0,
            std::min(1.0, unboundedRatio));

        const double projectedX = ax + ratio * vx;
        const double projectedY = ay + ratio * vy;
        const double offsetX = xM - projectedX;
        const double offsetY = yM - projectedY;
        const double distance =
            std::hypot(offsetX, offsetY);
        const double tangentYaw = std::atan2(vy, vx);
        const double headingError =
            wrapToPi(tangentYaw - yawRad);
        const double progress =
            cumulativeDistanceM_[i] +
            ratio * segmentLength;
        const double progressDisagreement =
            std::fabs(
                progress - observedRouteProgressM);

        // Distance and heading select the exact local projection.
        // Progress identifies the correct occurrence of looping or
        // self-near route geometry.
        const double score =
            distance +
            0.02 * std::fabs(headingError) *
                kRadiansToDegrees +
            0.50 * progressDisagreement;

        if (score < bestScore) {
            bestScore = score;
            best.progressM = progress;
            best.lateralOffsetM =
                offsetX * (-vy / segmentLength) +
                offsetY * (vx / segmentLength);
            best.headingErrorRad = headingError;
            best.segmentIndex = i;
            best.distanceM = distance;
            found = true;
        }
    }

    if (!found) {
        throw std::runtime_error(
            "immutable route has no projectable segment "
            "inside shared-progress window");
    }

    if (best.distanceM > kMaximumProjectionDistanceM) {
        throw std::runtime_error(
            "CARLA pose is too far from immutable route");
    }

    if (std::fabs(
            best.progressM -
            observedRouteProgressM) >
        kMaximumProgressDisagreementM) {
        throw std::runtime_error(
            "C++ route projection disagrees with "
            "observed shared-route progress");
    }

    return best;
}

LateralSteeringResult LateralController::computeSteering(
    const ObservedVehicleState& state,
    double referenceLateralOffsetM)
{
    if (std::isfinite(state.speedMps) &&
        state.speedMps < kMinimumControlSpeedMps) {
        return {
            LateralSteeringStatus::ExpectedPreviewUnavailable,
            0.0,
            "low_speed",
        };
    }

    try {
        return {
            LateralSteeringStatus::Valid,
            computeSteeringOrThrow(state, referenceLateralOffsetM),
            "valid",
        };
    }
    catch (const ExpectedPreviewUnavailable& error) {
        return {
            LateralSteeringStatus::ExpectedPreviewUnavailable,
            0.0,
            error.what(),
        };
    }
    catch (const std::invalid_argument& error) {
        return {
            LateralSteeringStatus::StructuralFailure,
            0.0,
            error.what(),
        };
    }
    catch (const std::logic_error& error) {
        return {
            LateralSteeringStatus::StructuralFailure,
            0.0,
            error.what(),
        };
    }
    catch (const std::runtime_error& error) {
        return {
            LateralSteeringStatus::StructuralFailure,
            0.0,
            error.what(),
        };
    }
}

double LateralController::computeSteeringOrThrow(
    const ObservedVehicleState& state,
    double referenceLateralOffsetM) const
{
    if (!initialized_) {
        throw std::logic_error(
            "lateral controller used before immutable route initialization");
    }

    if (!std::isfinite(state.positionM.x) ||
        !std::isfinite(state.positionM.y) ||
        !std::isfinite(state.yawRad) ||
        !std::isfinite(state.speedMps)) {
        throw std::runtime_error(
            "non-finite observed vehicle state for lateral control");
    }

    if (!state.routeProjectionValid ||
        !std::isfinite(state.routeProgressM)) {
        throw std::runtime_error(
            "valid shared-route progress is required for lateral control");
    }

    if (!std::isfinite(referenceLateralOffsetM)) {
        throw std::invalid_argument(
            "non-finite lateral reference offset");
    }

    const RouteProjection projection = projectPose(
        state.positionM.x,
        state.positionM.y,
        state.yawRad,
        state.routeProgressM);

    const RouteSample centerlineTarget =
        sampleAt(projection.progressM + kLookaheadM);

    const RouteSample target = {
        centerlineTarget.xM -
            std::sin(centerlineTarget.yawRad) *
                referenceLateralOffsetM,
        centerlineTarget.yM +
            std::cos(centerlineTarget.yawRad) *
                referenceLateralOffsetM,
        centerlineTarget.yawRad,
    };

    const double dx = target.xM - state.positionM.x;
    const double dy = target.yM - state.positionM.y;
    const double distance = std::hypot(dx, dy);

    const bool previewClampedAtRouteEnd =
        projection.progressM + kLookaheadM >=
            cumulativeDistanceM_.back() - kMinimumSegmentLengthM;

    if (!std::isfinite(distance) ||
        distance <= kMinimumSegmentLengthM) {
        if (previewClampedAtRouteEnd && std::isfinite(distance)) {
            throw ExpectedPreviewUnavailable("no_forward_preview");
        }
        throw std::runtime_error(
            "lateral preview target has invalid distance");
    }

    const double forwardX = std::cos(state.yawRad);
    const double forwardY = std::sin(state.yawRad);
    const double rightX = -std::sin(state.yawRad);
    const double rightY = std::cos(state.yawRad);

    const double targetForward =
        dx * forwardX + dy * forwardY;
    const double targetRight =
        dx * rightX + dy * rightY;

    if (targetForward <= kMinimumTargetForwardM) {
        // Projection, route bounds, route distance, and progress agreement
        // have already been validated. A target that is not in the current
        // forward half-plane is therefore an unavailable preview (for example
        // at route end or after valid physical yaw evolution), not corrupted
        // route state. The app deterministically retains its last valid steer.
        throw ExpectedPreviewUnavailable("no_forward_preview");
    }

    // CARLA Ackermann positive steering is right. targetRight uses the same
    // vehicle-frame convention, so the pure-pursuit sign is direct.
    const double curvature =
        2.0 * targetRight / (distance * distance);
    const double steeringRad =
        std::atan(wheelbaseM_ * curvature);

    return clampSymmetric(
        steeringRad,
        maximumFrontWheelSteeringRad_);
}

} // namespace carla
