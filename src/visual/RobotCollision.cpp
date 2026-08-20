#include "robot/visual/RobotCollision.hpp"

#include <algorithm>

namespace robot::visual
{

namespace
{

// Tiny margin added to the collision radius so a proposed pose is
// rejected slightly before exact floating-point tangency, rather than
// occasionally slipping through right at the boundary due to rounding -
// keeps "never enters the obstacle" deterministic instead of flip-
// flopping at the edge.
constexpr float kCollisionEpsilon = 1.0e-4F;

bool collidesWithObstacle(const Vec3& position, const BoxObstacle& obstacle) noexcept
{
    if (!obstacle.enabled)
    {
        return false;
    }

    const float minX = obstacle.position.x - (obstacle.size.x / 2.0F);
    const float maxX = obstacle.position.x + (obstacle.size.x / 2.0F);
    const float minZ = obstacle.position.z - (obstacle.size.z / 2.0F);
    const float maxZ = obstacle.position.z + (obstacle.size.z / 2.0F);

    const float closestX = std::clamp(position.x, minX, maxX);
    const float closestZ = std::clamp(position.z, minZ, maxZ);

    const float dx = position.x - closestX;
    const float dz = position.z - closestZ;
    const float distanceSquared = (dx * dx) + (dz * dz);

    const float effectiveRadius = kRobotCollisionRadius + kCollisionEpsilon;
    return distanceSquared < (effectiveRadius * effectiveRadius);
}

} // namespace

bool robotPositionCollidesWithObstacles(const Vec3& position, const std::vector<BoxObstacle>& obstacles) noexcept
{
    for (const BoxObstacle& obstacle : obstacles)
    {
        if (collidesWithObstacle(position, obstacle))
        {
            return true;
        }
    }
    return false;
}

} // namespace robot::visual
