#include <cmath>

#include <gtest/gtest.h>

#include "robot/visual/VirtualDistanceSensor.hpp"
#include "robot/visual/VirtualObstacleSensorArray.hpp"
#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace
{

using robot::visual::ObstacleRayPosition;
using robot::visual::ObstacleSensorArrayReadings;
using robot::visual::Vec3;
using robot::visual::VirtualDistanceSensor;
using robot::visual::VirtualObstacleSensorArray;
using robot::visual::VirtualWorld;

// Same convention every other visual-simulator test file uses: disable
// every default demo obstacle so a test can place/enable exactly one (or
// two) controlled obstacles of its own, with expected results computed
// from known, simple numbers.
void disableAllObstacles(VirtualWorld& world)
{
    for (std::size_t i = 0; i < world.obstacles().size(); ++i)
    {
        world.setObstacleEnabled(i, false);
    }
}

constexpr float kEpsilon = 0.001F;

// Lateral offset of the left/right rays from center - RobotDimensions'
// own kBodyWidth is the source of truth, kLateralInset the only
// additional named constant (test 15 proves this directly against the
// class's own rayOrigin()).
const float kLateralOffset =
    (robot::visual::RobotDimensions::kBodyWidth / 2.0F) - VirtualObstacleSensorArray::kLateralInset;

} // namespace

// 1: EmptyWorldAllRaysClear
TEST(VirtualObstacleSensorArrayTest, EmptyWorldAllRaysClear)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    VirtualObstacleSensorArray array(world);

    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_FALSE(readings.frontLeftDetected);
    EXPECT_FALSE(readings.frontCenterDetected);
    EXPECT_FALSE(readings.frontRightDetected);
    EXPECT_FALSE(readings.anyDetected());
    EXPECT_FALSE(readings.frontLeftDistance.has_value());
    EXPECT_FALSE(readings.frontCenterDistance.has_value());
    EXPECT_FALSE(readings.frontRightDistance.has_value());
}

// 2: CenterObstacleDetectedByCenterRay
TEST(VirtualObstacleSensorArrayTest, CenterObstacleDetectedByCenterRay)
{
    // Near face 0.6, sensor origin Z 0.25 -> distance 0.35, within
    // kDetectionDistance (1.0).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 1.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualObstacleSensorArray array(world);

    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_TRUE(readings.frontCenterDetected);
    ASSERT_TRUE(readings.frontCenterDistance.has_value());
    EXPECT_NEAR(*readings.frontCenterDistance, 0.35F, kEpsilon);
}

// 3: LeftOffsetObstacleDetectedByLeftRay
//
// Explicitly resized to 0.7 wide (half-width 0.35) via
// VirtualWorld::setObstacleSize() (Phase 13W human-visual-redesign v2) -
// narrow enough that positioned at X -0.5, its X range is
// [-0.85, -0.15]: contains the left ray's origin X (-0.25) but neither
// center's (0.0) nor right's (+0.25).
TEST(VirtualObstacleSensorArrayTest, LeftOffsetObstacleDetectedByLeftRay)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(3, Vec3{-0.5F, 0.4F, 1.5F});
    world.setObstacleSize(3, Vec3{0.7F, 0.8F, 1.2F});
    world.setObstacleEnabled(3, true);
    VirtualObstacleSensorArray array(world);

    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_TRUE(readings.frontLeftDetected);
    EXPECT_FALSE(readings.frontCenterDetected);
    EXPECT_FALSE(readings.frontRightDetected);
}

// 4: RightOffsetObstacleDetectedByRightRay
//
// Mirror image of test 3: X range [0.15, 0.85].
TEST(VirtualObstacleSensorArrayTest, RightOffsetObstacleDetectedByRightRay)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(3, Vec3{0.5F, 0.4F, 1.5F});
    world.setObstacleSize(3, Vec3{0.7F, 0.8F, 1.2F});
    world.setObstacleEnabled(3, true);
    VirtualObstacleSensorArray array(world);

    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_FALSE(readings.frontLeftDetected);
    EXPECT_FALSE(readings.frontCenterDetected);
    EXPECT_TRUE(readings.frontRightDetected);
}

// 5: LeftOffsetObstacleMissesCenterRayButAggregateDetects
TEST(VirtualObstacleSensorArrayTest, LeftOffsetObstacleMissesCenterRayButAggregateDetects)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(3, Vec3{-0.5F, 0.4F, 1.5F});
    world.setObstacleSize(3, Vec3{0.7F, 0.8F, 1.2F});
    world.setObstacleEnabled(3, true);
    VirtualObstacleSensorArray array(world);

    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_FALSE(readings.frontCenterDetected);
    EXPECT_TRUE(readings.anyDetected());
}

// 6: RightOffsetObstacleMissesCenterRayButAggregateDetects
TEST(VirtualObstacleSensorArrayTest, RightOffsetObstacleMissesCenterRayButAggregateDetects)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(3, Vec3{0.5F, 0.4F, 1.5F});
    world.setObstacleSize(3, Vec3{0.7F, 0.8F, 1.2F});
    world.setObstacleEnabled(3, true);
    VirtualObstacleSensorArray array(world);

    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_FALSE(readings.frontCenterDetected);
    EXPECT_TRUE(readings.anyDetected());
}

// 7: DisabledObstacleIgnored
TEST(VirtualObstacleSensorArrayTest, DisabledObstacleIgnored)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 1.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    // Deliberately left disabled (disableAllObstacles() above).
    VirtualObstacleSensorArray array(world);

    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_FALSE(readings.anyDetected());
}

// 8: ObstacleBehindRobotIgnored
TEST(VirtualObstacleSensorArrayTest, ObstacleBehindRobotIgnored)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, -2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualObstacleSensorArray array(world);

    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_FALSE(readings.anyDetected());
}

// 9: HeadingZero
TEST(VirtualObstacleSensorArrayTest, HeadingZero)
{
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    VirtualObstacleSensorArray array(world);

    const Vec3 direction = array.rayDirection();

    EXPECT_NEAR(direction.x, 0.0F, kEpsilon);
    EXPECT_NEAR(direction.z, 1.0F, kEpsilon);
}

// 10: HeadingNinety
TEST(VirtualObstacleSensorArrayTest, HeadingNinety)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    world.setObstaclePosition(0, Vec3{1.0F, 0.4F, 0.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualObstacleSensorArray array(world);

    const Vec3 direction = array.rayDirection();
    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_NEAR(direction.x, 1.0F, kEpsilon);
    EXPECT_NEAR(direction.z, 0.0F, kEpsilon);
    EXPECT_TRUE(readings.frontCenterDetected);
}

// 11: HeadingMinusNinety
TEST(VirtualObstacleSensorArrayTest, HeadingMinusNinety)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(-90.0F);
    world.setObstaclePosition(0, Vec3{-1.0F, 0.4F, 0.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualObstacleSensorArray array(world);

    const Vec3 direction = array.rayDirection();
    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_NEAR(direction.x, -1.0F, kEpsilon);
    EXPECT_NEAR(direction.z, 0.0F, kEpsilon);
    EXPECT_TRUE(readings.frontCenterDetected);
}

// 12: HeadingOneEighty
TEST(VirtualObstacleSensorArrayTest, HeadingOneEighty)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(180.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, -1.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualObstacleSensorArray array(world);

    const Vec3 direction = array.rayDirection();
    const ObstacleSensorArrayReadings readings = array.readings();

    EXPECT_NEAR(direction.x, 0.0F, kEpsilon);
    EXPECT_NEAR(direction.z, -1.0F, kEpsilon);
    EXPECT_TRUE(readings.frontCenterDetected);
}

// 13: ClosestDistancePerRayIsCorrect
TEST(VirtualObstacleSensorArrayTest, ClosestDistancePerRayIsCorrect)
{
    // Center: near face 1.6, origin Z 0.4 -> distance 1.2 (matches
    // VirtualDistanceSensorTest's own ObstacleStraightAheadReturnsDistance
    // for the same geometry). Left/right rays: same Z geometry, offset in
    // X only - the obstacle (0.8 wide, half 0.4) still spans both
    // (|+-0.25| < 0.4), so all three see the same near face and therefore
    // the same distance.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualObstacleSensorArray array(world);

    const ObstacleSensorArrayReadings readings = array.readings();

    ASSERT_TRUE(readings.frontLeftDistance.has_value());
    ASSERT_TRUE(readings.frontCenterDistance.has_value());
    ASSERT_TRUE(readings.frontRightDistance.has_value());
    EXPECT_NEAR(*readings.frontLeftDistance, 1.35F, kEpsilon);
    EXPECT_NEAR(*readings.frontCenterDistance, 1.35F, kEpsilon);
    EXPECT_NEAR(*readings.frontRightDistance, 1.35F, kEpsilon);
}

// 14: MultipleObstaclesHandled
TEST(VirtualObstacleSensorArrayTest, MultipleObstaclesHandled)
{
    // Two obstacles on the center ray's path - the nearer one must win,
    // mirroring VirtualDistanceSensorTest's own
    // NearestOfMultipleObstaclesIsReturned.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F}); // near: distance 1.2
    world.setObstacleEnabled(0, true);
    world.setObstaclePosition(1, Vec3{0.0F, 0.4F, 3.2F});
    world.setObstacleSize(1, Vec3{0.8F, 0.8F, 0.8F}); // far: distance 2.4
    world.setObstacleEnabled(1, true);
    VirtualObstacleSensorArray array(world);

    const ObstacleSensorArrayReadings readings = array.readings();

    ASSERT_TRUE(readings.frontCenterDistance.has_value());
    EXPECT_NEAR(*readings.frontCenterDistance, 1.35F, kEpsilon);
}

// 15: BodyDimensionsAreReusedRatherThanDuplicated
TEST(VirtualObstacleSensorArrayTest, BodyDimensionsAreReusedRatherThanDuplicated)
{
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    VirtualObstacleSensorArray array(world);

    const Vec3 left = array.rayOrigin(ObstacleRayPosition::FrontLeft);
    const Vec3 right = array.rayOrigin(ObstacleRayPosition::FrontRight);
    const Vec3 centerOrigin = array.rayOrigin(ObstacleRayPosition::FrontCenter);

    // The lateral offset is derived purely from RobotDimensions::kBodyWidth
    // and VirtualObstacleSensorArray::kLateralInset (kLateralOffset,
    // computed once above from those same two sources) - never an
    // independently hand-duplicated magic number.
    EXPECT_NEAR(centerOrigin.x - left.x, kLateralOffset, kEpsilon);
    EXPECT_NEAR(right.x - centerOrigin.x, kLateralOffset, kEpsilon);
}
