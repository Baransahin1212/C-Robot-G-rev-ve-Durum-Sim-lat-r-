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
    obstacles_.push_back(BoxObstacle{Vec3{-2.0F, 0.4F, 3.0F}, Vec3{0.7F, 0.8F, 1.2F}});
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

} // namespace robot::visual
