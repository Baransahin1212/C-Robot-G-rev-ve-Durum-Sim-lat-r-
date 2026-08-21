#include "robot/visual/VirtualObstacleSensorArray.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot/visual/VisualMath.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

namespace
{

// Same rationale as VirtualDistanceSensor.cpp's own kDirectionEpsilon:
// below this magnitude, a ray-direction component is treated as exactly
// zero (parallel to that axis' slab) rather than divided by.
constexpr float kDirectionEpsilon = 1.0e-6F;

// Identical technique to VirtualDistanceSensor.cpp's own
// intersectRayWithObstacleFootprint() - deliberately a self-contained
// copy (see this class's header docs for why), not a shared call,
// matching this codebase's existing precedent of each geometry component
// owning its own ray/segment-vs-AABB math.
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
        return std::nullopt;
    }

    const float distance = std::max(tMin, 0.0F);
    if (distance > maxRange)
    {
        return std::nullopt;
    }

    return distance;
}

std::optional<float> distanceToNearestObstacleAlongRay(const Vec3& origin, const Vec3& direction,
                                                         const VirtualWorld& world)
{
    std::optional<float> nearest;
    for (const BoxObstacle& obstacle : world.obstacles())
    {
        if (!obstacle.enabled)
        {
            continue;
        }

        const std::optional<float> hit =
            intersectRayWithObstacleFootprint(origin, direction, obstacle, VirtualDistanceSensor::kMaximumRange);
        if (hit.has_value() && (!nearest.has_value() || *hit < *nearest))
        {
            nearest = hit;
        }
    }
    return nearest;
}

} // namespace

VirtualObstacleSensorArray::VirtualObstacleSensorArray(const VirtualWorld& world)
    : world_(world)
{
}

Vec3 VirtualObstacleSensorArray::rayDirection() const
{
    return forwardDirection(world_.robotPose());
}

Vec3 VirtualObstacleSensorArray::rayOrigin(ObstacleRayPosition position) const
{
    const RobotPose& pose = world_.robotPose();
    const Vec3 forward = forwardDirection(pose);
    const float halfBodyLength = RobotDimensions::kBodyLength / 2.0F;
    const Vec3 center{pose.position.x + (forward.x * halfBodyLength), pose.position.y,
                       pose.position.z + (forward.z * halfBodyLength)};

    if (position == ObstacleRayPosition::FrontCenter)
    {
        return center;
    }

    const Vec3 right = rightDirection(pose);
    const float lateralOffset = (RobotDimensions::kBodyWidth / 2.0F) - kLateralInset;
    const float sign = (position == ObstacleRayPosition::FrontLeft) ? -1.0F : 1.0F;

    return Vec3{center.x + (right.x * sign * lateralOffset), center.y, center.z + (right.z * sign * lateralOffset)};
}

ObstacleSensorArrayReadings VirtualObstacleSensorArray::readings() const
{
    const Vec3 direction = rayDirection();

    ObstacleSensorArrayReadings result;

    result.frontLeftDistance =
        distanceToNearestObstacleAlongRay(rayOrigin(ObstacleRayPosition::FrontLeft), direction, world_);
    result.frontCenterDistance =
        distanceToNearestObstacleAlongRay(rayOrigin(ObstacleRayPosition::FrontCenter), direction, world_);
    result.frontRightDistance =
        distanceToNearestObstacleAlongRay(rayOrigin(ObstacleRayPosition::FrontRight), direction, world_);

    result.frontLeftDetected =
        result.frontLeftDistance.has_value() && *result.frontLeftDistance <= VirtualDistanceSensor::kDetectionDistance;
    result.frontCenterDetected = result.frontCenterDistance.has_value() &&
                                  *result.frontCenterDistance <= VirtualDistanceSensor::kDetectionDistance;
    result.frontRightDetected = result.frontRightDistance.has_value() &&
                                 *result.frontRightDistance <= VirtualDistanceSensor::kDetectionDistance;

    return result;
}

} // namespace robot::visual
