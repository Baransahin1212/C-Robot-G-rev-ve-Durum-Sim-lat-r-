#include "robot/visual/VirtualDistanceSensor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot/visual/VisualMath.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

namespace
{

// Below this magnitude, a ray-direction component is treated as exactly
// zero (parallel to that axis' slab) rather than divided by - avoids NaN/Inf
// from a near-zero float division and matches the intent of headings that
// are conceptually axis-aligned (e.g. heading 90 => direction.z is a tiny
// non-zero float, not exactly 0.0, purely from the degrees->radians
// conversion).
constexpr float kDirectionEpsilon = 1.0e-6F;

// 2D (X/Z) ray/AABB slab intersection against one obstacle's footprint.
// Returns the entry distance along the ray when it is a valid forward hit
// within [0, maxRange]; std::nullopt otherwise (behind the ray, missed
// entirely, or beyond maxRange). Deliberately headless - no raylib
// collision helper (e.g. GetRayCollisionBox) is used, so this stays
// unit-testable without a window. Handles direction.x == 0, direction.z ==
// 0, and tangent/boundary hits deterministically; never divides by zero.
std::optional<float> intersectRayWithObstacleFootprint(const Vec3& origin, const Vec3& direction,
                                                         const BoxObstacle& obstacle, float maxRange)
{
    const float minX = obstacle.position.x - (obstacle.size.x / 2.0F);
    const float maxX = obstacle.position.x + (obstacle.size.x / 2.0F);
    const float minZ = obstacle.position.z - (obstacle.size.z / 2.0F);
    const float maxZ = obstacle.position.z + (obstacle.size.z / 2.0F);

    float tMin = -std::numeric_limits<float>::infinity();
    float tMax = maxRange;

    if (std::fabs(direction.x) < kDirectionEpsilon)
    {
        // Ray parallel to the X slab: only a hit if the origin already lies
        // within the obstacle's X range - otherwise the ray can never enter
        // it, no matter how far it travels along Z.
        if (origin.x < minX || origin.x > maxX)
        {
            return std::nullopt;
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
            return std::nullopt;
        }
    }

    if (std::fabs(direction.z) < kDirectionEpsilon)
    {
        if (origin.z < minZ || origin.z > maxZ)
        {
            return std::nullopt;
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
            return std::nullopt;
        }
    }

    if (tMax < 0.0F)
    {
        // The entire intersection interval is behind the ray origin.
        return std::nullopt;
    }

    // tMin < 0 with tMax >= 0 means the origin itself is inside the
    // obstacle's footprint - report a touching distance of 0 rather than a
    // negative one, instead of treating it as "behind".
    const float distance = std::max(tMin, 0.0F);
    if (distance > maxRange)
    {
        return std::nullopt;
    }

    return distance;
}

} // namespace

VirtualDistanceSensor::VirtualDistanceSensor(const VirtualWorld& world)
    : world_(world)
{
}

Vec3 VirtualDistanceSensor::sensorOrigin() const
{
    const RobotPose& pose = world_.robotPose();
    const Vec3 forward = forwardDirection(pose);
    const float halfBodyLength = RobotDimensions::kBodyLength / 2.0F;
    return Vec3{pose.position.x + (forward.x * halfBodyLength), pose.position.y,
                pose.position.z + (forward.z * halfBodyLength)};
}

Vec3 VirtualDistanceSensor::sensorDirection() const
{
    return forwardDirection(world_.robotPose());
}

std::optional<float> VirtualDistanceSensor::distanceToNearestObstacle() const
{
    const Vec3 origin = sensorOrigin();
    const Vec3 direction = sensorDirection();

    std::optional<float> nearest;
    for (const BoxObstacle& obstacle : world_.obstacles())
    {
        if (!obstacle.enabled)
        {
            continue;
        }

        const std::optional<float> hit =
            intersectRayWithObstacleFootprint(origin, direction, obstacle, kMaximumRange);
        if (hit.has_value() && (!nearest.has_value() || *hit < *nearest))
        {
            nearest = hit;
        }
    }

    return nearest;
}

bool VirtualDistanceSensor::obstacleDetected() const
{
    const std::optional<float> distance = distanceToNearestObstacle();
    return distance.has_value() && *distance <= kDetectionDistance;
}

} // namespace robot::visual
