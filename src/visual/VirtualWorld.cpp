#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

namespace
{

// Robot starts near center-left of the ~10x10 demo scene, above the
// ground by half its body height (see VisualRobot.hpp's kBodyHeight) so
// it visually rests on the ground plane rather than being embedded in it.
constexpr float kRobotStartX = -3.0F;
constexpr float kRobotStartY = 0.125F;
constexpr float kRobotStartZ = 1.0F;

// Base platform sits near one corner of the scene, away from the robot's
// start position and every obstacle below.
constexpr float kBaseX = 4.0F;
constexpr float kBaseZ = 4.0F;
constexpr float kBaseWidth = 1.5F;
constexpr float kBaseHeight = 0.05F;
constexpr float kBaseDepth = 1.5F;

// The one demo obstacle deliberately placed directly in the robot's initial
// forward path (Phase 13O) - X matches the robot's start X exactly, so the
// robot's heading-0 (+Z) forward ray runs straight through it, and Z is
// placed far enough ahead of the robot's start Z that the robot visibly
// travels for over a second before VirtualDistanceSensor reports it within
// range. VirtualWorld::kBlockingObstacleIndex identifies its index in
// obstacles_ below (must stay in sync with push_back order).
constexpr float kBlockingObstacleX = kRobotStartX;
constexpr float kBlockingObstacleZ = 4.3F;

} // namespace

VirtualWorld::VirtualWorld()
    : robotPose_{Vec3{kRobotStartX, kRobotStartY, kRobotStartZ}, 0.0F}
    , basePlatform_{Vec3{kBaseX, kBaseHeight / 2.0F, kBaseZ}, Vec3{kBaseWidth, kBaseHeight, kBaseDepth}}
{
    // Four scattered box obstacles, none overlapping the robot's start
    // position or the base platform.
    obstacles_.push_back(BoxObstacle{Vec3{-1.0F, 0.4F, -2.0F}, Vec3{0.8F, 0.8F, 0.8F}});
    obstacles_.push_back(BoxObstacle{Vec3{1.5F, 0.4F, 0.5F}, Vec3{0.8F, 0.8F, 0.8F}});
    obstacles_.push_back(BoxObstacle{Vec3{2.5F, 0.4F, -2.5F}, Vec3{1.0F, 0.8F, 0.6F}});
    obstacles_.push_back(BoxObstacle{Vec3{kBlockingObstacleX, 0.4F, kBlockingObstacleZ}, Vec3{0.7F, 0.8F, 1.2F}});
}

const RobotPose& VirtualWorld::robotPose() const noexcept
{
    return robotPose_;
}

const BasePlatform& VirtualWorld::basePlatform() const noexcept
{
    return basePlatform_;
}

const std::vector<BoxObstacle>& VirtualWorld::obstacles() const noexcept
{
    return obstacles_;
}

void VirtualWorld::setRobotPosition(const Vec3& position)
{
    robotPose_.position = position;
}

void VirtualWorld::setRobotHeading(float headingDegrees)
{
    robotPose_.headingDegrees = headingDegrees;
}

bool VirtualWorld::setObstaclePosition(std::size_t index, const Vec3& position)
{
    if (index >= obstacles_.size())
    {
        return false;
    }
    obstacles_[index].position = position;
    return true;
}

bool VirtualWorld::setObstacleEnabled(std::size_t index, bool enabled)
{
    if (index >= obstacles_.size())
    {
        return false;
    }
    obstacles_[index].enabled = enabled;
    return true;
}

bool VirtualWorld::obstacleEnabled(std::size_t index) const
{
    if (index >= obstacles_.size())
    {
        return false;
    }
    return obstacles_[index].enabled;
}

} // namespace robot::visual
