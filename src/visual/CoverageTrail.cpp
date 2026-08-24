#include "robot/visual/CoverageTrail.hpp"

#include <cmath>

namespace robot::visual
{

void CoverageTrail::update(const RobotPose& pose)
{
    if (points_.empty())
    {
        points_.push_back(pose.position);
        dirty_ = true;
        return;
    }

    const Vec3& last = points_.back();
    const float dx = pose.position.x - last.x;
    const float dz = pose.position.z - last.z;
    const float distance = std::sqrt((dx * dx) + (dz * dz));

    if (distance >= kTrailSampleDistanceWorldUnits)
    {
        points_.push_back(pose.position);
        dirty_ = true;
    }
}

void CoverageTrail::clear() noexcept
{
    points_.clear();
    dirty_ = true;
}

const std::vector<Vec3>& CoverageTrail::points() const noexcept
{
    return points_;
}

void CoverageTrail::loadPoints(std::vector<Vec3> points)
{
    points_ = std::move(points);
    dirty_ = false;
}

bool CoverageTrail::consumeDirty() noexcept
{
    const bool wasDirty = dirty_;
    dirty_ = false;
    return wasDirty;
}

} // namespace robot::visual
