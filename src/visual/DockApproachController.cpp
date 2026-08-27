#include "robot/visual/DockApproachController.hpp"

#include <algorithm>
#include <cmath>

#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VisualMath.hpp"

namespace robot::visual
{

namespace
{
// Which of the four table edges `base.position` sits nearest to, and how
// far away it is - the one shared computation
// computeDockApproachPoint()/computeDockEntranceHeadingDegrees() both
// build on, so the two can never derive a different axis/direction from
// each other. Never guessed - this is the same "the dock always backs
// onto whichever edge it was placed nearest" fact VirtualWorld.cpp's own
// kDockHousingZ/kRobotStartZ placement already assumes, just derived here
// instead of hardcoded.
enum class Axis
{
    X,
    Z
};

struct NearestEdge
{
    Axis axis;
    float distance;
    // +1 if the NEAREST edge is the max-bound edge (entrance therefore
    // faces the min-bound side), -1 if the nearest edge is the min-bound
    // edge (entrance faces the max-bound side).
    float entranceSign;
};

NearestEdge nearestTableEdge(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    const float distToMinX = base.position.x - tableSurface.minX;
    const float distToMaxX = tableSurface.maxX - base.position.x;
    const float distToMinZ = base.position.z - tableSurface.minZ;
    const float distToMaxZ = tableSurface.maxZ - base.position.z;

    NearestEdge best{Axis::Z, distToMinZ, 1.0F};
    if (distToMaxZ < best.distance)
    {
        best = NearestEdge{Axis::Z, distToMaxZ, -1.0F};
    }
    if (distToMinX < best.distance)
    {
        best = NearestEdge{Axis::X, distToMinX, 1.0F};
    }
    if (distToMaxX < best.distance)
    {
        best = NearestEdge{Axis::X, distToMaxX, -1.0F};
    }
    return best;
}

// The world-space offset (dx, dz) from `base.position` to the approach
// point - entranceSign * kDockApproachOffsetWorldUnits along whichever
// axis the platform is nearest a table edge on, zero along the other.
struct ApproachOffset
{
    float dx;
    float dz;
};

ApproachOffset computeApproachOffset(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    const NearestEdge edge = nearestTableEdge(base, tableSurface);
    const float depthAlongAxis = (edge.axis == Axis::X) ? base.size.x : base.size.z;
    const float offsetMagnitude =
        (depthAlongAxis / 2.0F) + kRobotCollisionRadius + kDockApproachMarginWorldUnits;
    if (edge.axis == Axis::X)
    {
        return ApproachOffset{edge.entranceSign * offsetMagnitude, 0.0F};
    }
    return ApproachOffset{0.0F, edge.entranceSign * offsetMagnitude};
}
} // namespace

Vec3 computeDockApproachPoint(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    const ApproachOffset offset = computeApproachOffset(base, tableSurface);
    return Vec3{base.position.x + offset.dx, base.position.y, base.position.z + offset.dz};
}

float computeDockEntranceHeadingDegrees(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    // Heading FROM the approach point TOWARD base.position - the negation
    // of the approach offset itself.
    const ApproachOffset offset = computeApproachOffset(base, tableSurface);
    return normalizeHeadingDegrees(headingDegreesFromDirection(-offset.dx, -offset.dz));
}

bool isDockApproachGeometryValid(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    const Vec3 approachPoint = computeDockApproachPoint(base, tableSurface);
    const ApproachOffset offset = computeApproachOffset(base, tableSurface);
    const float offsetMagnitude = std::sqrt((offset.dx * offset.dx) + (offset.dz * offset.dz));
    if (!(offsetMagnitude > 0.0F) || !std::isfinite(offsetMagnitude))
    {
        return false;
    }
    // Comfortably inside the table boundary - at least one full robot
    // collision radius of margin past the approach point itself, so the
    // robot's own footprint (centered there) never hangs off the table
    // edge during Aligning.
    if (approachPoint.x - tableSurface.minX < kRobotCollisionRadius ||
        tableSurface.maxX - approachPoint.x < kRobotCollisionRadius ||
        approachPoint.z - tableSurface.minZ < kRobotCollisionRadius ||
        tableSurface.maxZ - approachPoint.z < kRobotCollisionRadius)
    {
        return false;
    }
    return true;
}

DockApproachOutput DockApproachController::update(const RobotPose& pose, const BasePlatform& base,
                                                    const TableSurface& tableSurface, bool enabled,
                                                    bool arrivedAtApproachPoint) noexcept
{
    if (!enabled)
    {
        state_ = DockApproachState::Inactive;
        return DockApproachOutput{WheelSpeeds{0.0F, 0.0F}, state_, false, false};
    }

    if (state_ == DockApproachState::Inactive)
    {
        state_ = DockApproachState::NavigatingToApproach;
    }

    if (state_ == DockApproachState::Failed)
    {
        return DockApproachOutput{WheelSpeeds{0.0F, 0.0F}, state_, false, false};
    }

    if (!isDockApproachGeometryValid(base, tableSurface))
    {
        state_ = DockApproachState::Failed;
        return DockApproachOutput{WheelSpeeds{0.0F, 0.0F}, state_, false, false};
    }

    const Vec3 approachPoint = computeDockApproachPoint(base, tableSurface);
    const float entranceHeading = computeDockEntranceHeadingDegrees(base, tableSurface);

    const float dxHome = base.position.x - pose.position.x;
    const float dzHome = base.position.z - pose.position.z;
    const float distanceToHome = std::sqrt((dxHome * dxHome) + (dzHome * dzHome));

    const float dxApproach = approachPoint.x - pose.position.x;
    const float dzApproach = approachPoint.z - pose.position.z;
    const float distanceToApproach = std::sqrt((dxApproach * dxApproach) + (dzApproach * dzApproach));

    const ApproachOffset offset = computeApproachOffset(base, tableSurface);
    const float offsetMagnitude = std::sqrt((offset.dx * offset.dx) + (offset.dz * offset.dz));
    const float reentryDistance = offsetMagnitude * kReentryDistanceMultiplier;

    if (state_ == DockApproachState::NavigatingToApproach)
    {
        if (arrivedAtApproachPoint)
        {
            state_ = (distanceToHome <= HomeNavigator::kHomeArrivalRadius) ? DockApproachState::Arrived
                                                                            : DockApproachState::Aligning;
        }
    }
    else if (state_ == DockApproachState::Aligning || state_ == DockApproachState::FinalApproach ||
             state_ == DockApproachState::Arrived)
    {
        // Phase 13X final-approach fix (real-GUI-traced defect): distance/
        // reentry are re-evaluated EVERY frame even while already Arrived
        // - never a one-way "arrived, done forever" latch. A parked robot
        // can legitimately be displaced again (e.g. ordinary obstacle-
        // triggered avoidance firing the instant the suppression above
        // lapses - see `dockApproachActive`'s own docs in this header, now
        // covering Arrived too - or a Safety recovery); without this,
        // DockApproachController would freeze permanently the moment
        // anything ever moved the robot away from an already-arrived
        // position, producing zero wheels forever and never re-docking.
        if (distanceToHome <= HomeNavigator::kHomeArrivalRadius)
        {
            state_ = DockApproachState::Arrived;
        }
        else if (distanceToApproach > reentryDistance)
        {
            // Displaced too far from the known-safe lane (e.g. a Safety
            // recovery mid-approach) - hand back to Stage 1 rather than
            // continuing to chase a fixed-heading lane that may no longer
            // pass through the robot's current position.
            state_ = DockApproachState::NavigatingToApproach;
        }
        else if (state_ == DockApproachState::Arrived)
        {
            // Just displaced out of arrival radius while still in the lane
            // - always re-verify heading before resuming, never straight
            // back into FinalApproach blindly.
            state_ = DockApproachState::Aligning;
        }
        else
        {
            const float headingError = shortestSignedHeadingErrorDegrees(pose.headingDegrees, entranceHeading);
            const float absHeadingError = std::fabs(headingError);
            if (state_ == DockApproachState::Aligning && absHeadingError <= kDockingHeadingToleranceDegrees)
            {
                state_ = DockApproachState::FinalApproach;
            }
            else if (state_ == DockApproachState::FinalApproach && absHeadingError >= kDockingStopToleranceDegrees)
            {
                state_ = DockApproachState::Aligning;
            }
        }
    }

    WheelSpeeds speeds{0.0F, 0.0F};
    bool driving = false;
    switch (state_)
    {
        case DockApproachState::Aligning:
        {
            const float headingError = shortestSignedHeadingErrorDegrees(pose.headingDegrees, entranceHeading);
            const float directionSign = (headingError >= 0.0F) ? 1.0F : -1.0F;
            speeds = WheelSpeeds{-kAligningTurnSpeed * directionSign, kAligningTurnSpeed * directionSign};
            driving = true;
            break;
        }
        case DockApproachState::FinalApproach:
            speeds = WheelSpeeds{kFinalApproachSpeed, kFinalApproachSpeed};
            driving = true;
            break;
        case DockApproachState::Inactive:
        case DockApproachState::NavigatingToApproach:
        case DockApproachState::Arrived:
        case DockApproachState::Failed:
            speeds = WheelSpeeds{0.0F, 0.0F};
            driving = false;
            break;
    }

    return DockApproachOutput{speeds, state_, state_ == DockApproachState::Arrived, driving};
}

void DockApproachController::reset() noexcept
{
    state_ = DockApproachState::Inactive;
}

DockApproachState DockApproachController::state() const noexcept
{
    return state_;
}

} // namespace robot::visual
