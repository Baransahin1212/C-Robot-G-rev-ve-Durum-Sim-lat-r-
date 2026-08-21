#include "robot/visual/ForwardClearanceProbe.hpp"

#include <algorithm>
#include <cmath>

#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VisualMath.hpp"

namespace robot::visual
{

namespace
{

// Same rationale as VirtualDistanceSensor.cpp's own kDirectionEpsilon:
// below this magnitude, a segment-direction component is treated as
// exactly zero (parallel to that axis' slab) rather than divided by -
// avoids NaN/Inf from a near-zero float division for headings that are
// conceptually axis-aligned (e.g. heading 90 => direction.z is a tiny
// non-zero float from the degrees->radians conversion, not exactly 0.0).
constexpr float kDirectionEpsilon = 1.0e-6F;

// Finite segment (origin -> origin + direction * length) vs. an
// axis-aligned X/Z box, via the standard slab method - the same technique
// VirtualDistanceSensor.cpp uses for its infinite forward ray, adapted to
// a bounded [0, length] parametric range instead of [0, +inf): tMin
// starts at 0 (the segment start, i.e. the robot's center - an obstacle
// entirely "behind" that start point can never intersect regardless of
// how far it extends behind) and tMax starts at `length` (the lookahead
// distance - an obstacle entirely beyond it can never intersect either).
// Returns true (intersects) inclusive of exact tangent/boundary contact
// (tMin == tMax), matching this project's existing sensor/collision
// convention of treating a boundary touch as a hit, not a miss. Never
// divides by zero - direction.x == 0 and direction.z == 0 are both
// handled as parallel-to-slab special cases.
bool segmentIntersectsExpandedAabb(const Vec3& origin, const Vec3& direction, float length, float minX, float maxX,
                                    float minZ, float maxZ)
{
    float tMin = 0.0F;
    float tMax = length;

    if (std::fabs(direction.x) < kDirectionEpsilon)
    {
        if (origin.x < minX || origin.x > maxX)
        {
            return false;
        }
    }
    else
    {
        float t1 = (minX - origin.x) / direction.x;
        float t2 = (maxX - origin.x) / direction.x;
        if (t1 > t2)
        {
            std::swap(t1, t2);
        }
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax)
        {
            return false;
        }
    }

    if (std::fabs(direction.z) < kDirectionEpsilon)
    {
        if (origin.z < minZ || origin.z > maxZ)
        {
            return false;
        }
    }
    else
    {
        float t1 = (minZ - origin.z) / direction.z;
        float t2 = (maxZ - origin.z) / direction.z;
        if (t1 > t2)
        {
            std::swap(t1, t2);
        }
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax)
        {
            return false;
        }
    }

    return tMin <= tMax;
}

} // namespace

ForwardClearanceProbe::ForwardClearanceProbe(const VirtualWorld& world)
    : world_(world)
{
}

bool ForwardClearanceProbe::isForwardCorridorClear() const
{
    const RobotPose& pose = world_.robotPose();
    const Vec3& origin = pose.position;
    const Vec3 direction = forwardDirection(pose);

    // Same collision-radius source of truth RobotCollision.hpp's physical
    // guard uses - never an independently-hand-duplicated body dimension
    // (Phase 13R brief, section 3).
    const float clearanceRadius = kRobotCollisionRadius + kSafetyMargin;

    for (const BoxObstacle& obstacle : world_.obstacles())
    {
        if (!obstacle.enabled)
        {
            continue;
        }

        const float minX = obstacle.position.x - (obstacle.size.x / 2.0F) - clearanceRadius;
        const float maxX = obstacle.position.x + (obstacle.size.x / 2.0F) + clearanceRadius;
        const float minZ = obstacle.position.z - (obstacle.size.z / 2.0F) - clearanceRadius;
        const float maxZ = obstacle.position.z + (obstacle.size.z / 2.0F) + clearanceRadius;

        if (segmentIntersectsExpandedAabb(origin, direction, kLookaheadDistance, minX, maxX, minZ, maxZ))
        {
            return false;
        }
    }

    return true;
}

} // namespace robot::visual
