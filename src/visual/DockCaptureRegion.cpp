#include "robot/visual/DockCaptureRegion.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

#include "robot/visual/RobotCollision.hpp"

namespace robot::visual
{

namespace
{
float distanceWorld(const Vec3& a, const Vec3& b) noexcept
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dz * dz));
}

// Index of the nearest ENABLED obstacle to `point`, or -1 if none. Ranked
// by distance to each obstacle's own center - a coarse but honest-enough
// proxy for "which obstacle is this," since it only ever needs to rank
// obstacles against each other, never measure an exact clearance (that is
// robotPositionCollidesWithObstacles()'s own job).
int nearestObstacleIndex(const Vec3& point, const std::vector<BoxObstacle>& obstacles) noexcept
{
    int best = -1;
    float bestDistance = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < obstacles.size(); ++i)
    {
        if (!obstacles[i].enabled)
        {
            continue;
        }
        const float distance = distanceWorld(point, obstacles[i].position);
        if (distance < bestDistance)
        {
            bestDistance = distance;
            best = static_cast<int>(i);
        }
    }
    return best;
}
} // namespace

bool isInsideDockStagingCaptureRegion(const RobotPose& pose, const BasePlatform& base,
                                       const TableSurface& tableSurface) noexcept
{
    const Vec3 staging = computeDockStagingPoint(base, tableSurface);
    return distanceWorld(pose.position, staging) <= DockApproachController::kDockStagingCaptureRadius;
}

bool isNearestHazardAttributableToDockGeometry(const RobotPose& pose, const BasePlatform& base,
                                                const std::vector<BoxObstacle>& obstacles) noexcept
{
    const int nearestToRobot = nearestObstacleIndex(pose.position, obstacles);
    const int nearestToDock = nearestObstacleIndex(base.position, obstacles);
    if (nearestToRobot < 0 || nearestToDock < 0)
    {
        return false;
    }
    return nearestToRobot == nearestToDock;
}

bool isDockCaptureEligible(const RobotPose& pose, const BasePlatform& base, const TableSurface& tableSurface,
                            const std::vector<BoxObstacle>& obstacles, bool physicallySupported) noexcept
{
    if (!physicallySupported)
    {
        return false;
    }
    if (!isInsideDockStagingCaptureRegion(pose, base, tableSurface))
    {
        return false;
    }
    if (robotPositionCollidesWithObstacles(pose.position, obstacles))
    {
        return false;
    }
    const Vec3 staging = computeDockStagingPoint(base, tableSurface);
    if (robotPositionCollidesWithObstacles(staging, obstacles))
    {
        return false;
    }
    return isNearestHazardAttributableToDockGeometry(pose, base, obstacles);
}

} // namespace robot::visual
