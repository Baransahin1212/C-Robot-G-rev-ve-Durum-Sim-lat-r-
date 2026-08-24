#include <cmath>

#include <gtest/gtest.h>

#include "robot/visual/VirtualDistanceSensor.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::visual::Vec3;
using robot::visual::VirtualDistanceSensor;
using robot::visual::VirtualWorld;

// All 18 tests build their own controlled obstacle geometry rather than
// relying on VirtualWorld's fixed Phase 13M demo layout: every fixed
// obstacle is disabled first, then the one obstacle under test is
// repositioned and re-enabled via VirtualWorld's test-support mutators
// (setObstaclePosition()/setObstacleEnabled()) so each test's expected
// distance can be computed from known, simple numbers instead of the demo
// scene's arbitrary coordinates.
void disableAllObstacles(VirtualWorld& world)
{
    for (std::size_t i = 0; i < world.obstacles().size(); ++i)
    {
        world.setObstacleEnabled(i, false);
    }
}

constexpr float kEpsilon = 0.001F;

} // namespace

// 1: NoObstaclesReturnsNoHit
TEST(VirtualDistanceSensorTest, NoObstaclesReturnsNoHit)
{
    // Arrange
    VirtualWorld world;
    disableAllObstacles(world);
    VirtualDistanceSensor sensor(world);

    // Act / Assert
    EXPECT_FALSE(sensor.distanceToNearestObstacle().has_value());
    EXPECT_FALSE(sensor.obstacleDetected());
}

// 2: ObstacleStraightAheadReturnsDistance
TEST(VirtualDistanceSensorTest, ObstacleStraightAheadReturnsDistance)
{
    // Arrange: robot at origin, heading 0 (+Z); obstacle (0.8 cube) centered
    // 2.0 ahead - near face at Z 1.6, sensor origin at Z 0.25 (0.25 =
    // RobotDimensions::kBodyLength / 2), so expected distance is 1.35.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act
    const std::optional<float> distance = sensor.distanceToNearestObstacle();

    // Assert
    ASSERT_TRUE(distance.has_value());
    EXPECT_NEAR(*distance, 1.35F, kEpsilon);
}

// 3: ObstacleBehindRobotIsIgnored
TEST(VirtualDistanceSensorTest, ObstacleBehindRobotIsIgnored)
{
    // Arrange: heading 0 faces +Z; obstacle sits at Z -2.0, entirely behind.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, -2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act / Assert
    EXPECT_FALSE(sensor.distanceToNearestObstacle().has_value());
}

// 4: ObstacleToSideIsIgnored
TEST(VirtualDistanceSensorTest, ObstacleToSideIsIgnored)
{
    // Arrange: heading 0 keeps the ray at constant X 0.0; obstacle's X range
    // (4.6 to 5.4) never includes it.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{5.0F, 0.4F, 2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act / Assert
    EXPECT_FALSE(sensor.distanceToNearestObstacle().has_value());
}

// 5: NearestOfMultipleObstaclesIsReturned
TEST(VirtualDistanceSensorTest, NearestOfMultipleObstaclesIsReturned)
{
    // Arrange: two obstacles ahead on the same ray, both within
    // kMaximumRange (2.5) - the near one (expected distance 1.2) must win
    // over the far one (expected distance 2.4).
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
    VirtualDistanceSensor sensor(world);

    // Act
    const std::optional<float> distance = sensor.distanceToNearestObstacle();

    // Assert
    ASSERT_TRUE(distance.has_value());
    EXPECT_NEAR(*distance, 1.35F, kEpsilon);
}

// 6: ObstacleBeyondMaximumRangeIsIgnored
TEST(VirtualDistanceSensorTest, ObstacleBeyondMaximumRangeIsIgnored)
{
    // Arrange: near face at Z 9.6, sensor at Z 0.4 - distance 9.2, far
    // beyond kMaximumRange (2.5).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 10.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act / Assert
    EXPECT_FALSE(sensor.distanceToNearestObstacle().has_value());
}

// 7: HeadingZeroLooksTowardPositiveZ
TEST(VirtualDistanceSensorTest, HeadingZeroLooksTowardPositiveZ)
{
    // Arrange
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act
    const Vec3 direction = sensor.sensorDirection();
    const std::optional<float> distance = sensor.distanceToNearestObstacle();

    // Assert
    EXPECT_NEAR(direction.x, 0.0F, kEpsilon);
    EXPECT_NEAR(direction.z, 1.0F, kEpsilon);
    ASSERT_TRUE(distance.has_value());
    EXPECT_NEAR(*distance, 1.35F, kEpsilon);
}

// 8: HeadingNinetyLooksTowardPositiveX
TEST(VirtualDistanceSensorTest, HeadingNinetyLooksTowardPositiveX)
{
    // Arrange: forward = (sin 90, 0, cos 90) = (1, 0, 0); sensor origin
    // (0.4, y, 0.0); obstacle centered at X 2.0 - near face X 1.6, expected
    // distance 1.2.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    world.setObstaclePosition(0, Vec3{2.0F, 0.4F, 0.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act
    const Vec3 direction = sensor.sensorDirection();
    const std::optional<float> distance = sensor.distanceToNearestObstacle();

    // Assert
    EXPECT_NEAR(direction.x, 1.0F, kEpsilon);
    EXPECT_NEAR(direction.z, 0.0F, kEpsilon);
    ASSERT_TRUE(distance.has_value());
    EXPECT_NEAR(*distance, 1.35F, kEpsilon);
}

// 9: HeadingMinusNinetyLooksTowardNegativeX
TEST(VirtualDistanceSensorTest, HeadingMinusNinetyLooksTowardNegativeX)
{
    // Arrange: forward = (sin -90, 0, cos -90) = (-1, 0, 0); sensor origin
    // (-0.4, y, 0.0); obstacle centered at X -2.0 - near face X -1.6,
    // expected distance 1.2.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(-90.0F);
    world.setObstaclePosition(0, Vec3{-2.0F, 0.4F, 0.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act
    const Vec3 direction = sensor.sensorDirection();
    const std::optional<float> distance = sensor.distanceToNearestObstacle();

    // Assert
    EXPECT_NEAR(direction.x, -1.0F, kEpsilon);
    EXPECT_NEAR(direction.z, 0.0F, kEpsilon);
    ASSERT_TRUE(distance.has_value());
    EXPECT_NEAR(*distance, 1.35F, kEpsilon);
}

// 10: HeadingOneEightyLooksTowardNegativeZ
TEST(VirtualDistanceSensorTest, HeadingOneEightyLooksTowardNegativeZ)
{
    // Arrange: forward = (sin 180, 0, cos 180) = (0, 0, -1); sensor origin
    // (0, y, -0.4); obstacle centered at Z -2.0 - near face Z -1.6,
    // expected distance 1.2.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(180.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, -2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act
    const Vec3 direction = sensor.sensorDirection();
    const std::optional<float> distance = sensor.distanceToNearestObstacle();

    // Assert
    EXPECT_NEAR(direction.x, 0.0F, kEpsilon);
    EXPECT_NEAR(direction.z, -1.0F, kEpsilon);
    ASSERT_TRUE(distance.has_value());
    EXPECT_NEAR(*distance, 1.35F, kEpsilon);
}

// 11: SensorOriginStartsAtRobotFront
TEST(VirtualDistanceSensorTest, SensorOriginStartsAtRobotFront)
{
    // Arrange: heading 0 - sensorOrigin() must be the robot's center offset
    // by +Z half the robot's body length (0.25), not the center itself.
    VirtualWorld world;
    world.setRobotPosition(Vec3{2.0F, 0.125F, 3.0F});
    world.setRobotHeading(0.0F);
    VirtualDistanceSensor sensor(world);

    // Act
    const Vec3 origin = sensor.sensorOrigin();

    // Assert
    EXPECT_NEAR(origin.x, 2.0F, kEpsilon);
    EXPECT_NEAR(origin.y, 0.125F, kEpsilon);
    EXPECT_NEAR(origin.z, 3.25F, kEpsilon);
}

// 12: DisabledObstacleIsIgnored
TEST(VirtualDistanceSensorTest, DisabledObstacleIsIgnored)
{
    // Arrange: same geometry as ObstacleStraightAheadReturnsDistance, but
    // left disabled.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    // Deliberately left disabled (disableAllObstacles() above).
    VirtualDistanceSensor sensor(world);

    // Act / Assert
    EXPECT_FALSE(sensor.distanceToNearestObstacle().has_value());
    EXPECT_FALSE(sensor.obstacleDetected());
}

// 13: EnablingObstacleMakesItVisibleToSensor
TEST(VirtualDistanceSensorTest, EnablingObstacleMakesItVisibleToSensor)
{
    // Arrange
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    VirtualDistanceSensor sensor(world);
    ASSERT_FALSE(sensor.distanceToNearestObstacle().has_value());

    // Act
    world.setObstacleEnabled(0, true);

    // Assert
    ASSERT_TRUE(sensor.distanceToNearestObstacle().has_value());
    EXPECT_NEAR(*sensor.distanceToNearestObstacle(), 1.35F, kEpsilon);
}

// 14: ParallelRayDoesNotDivideByZero
TEST(VirtualDistanceSensorTest, ParallelRayDoesNotDivideByZero)
{
    // Arrange: heading 0 makes direction.x exactly 0.0 (parallel to the X
    // slab). One obstacle whose X range does NOT contain the ray's X (must
    // return cleanly, no crash/NaN); one whose X range DOES contain it (must
    // still compute a finite hit distance from the Z slab alone). Two
    // independent VirtualWorld instances - VirtualDistanceSensor holds a
    // live reference, not a snapshot, so reusing one mutable world for both
    // sensors would let a later mutation change what an earlier sensor
    // reports.
    VirtualWorld worldOutsideSlab;
    disableAllObstacles(worldOutsideSlab);
    worldOutsideSlab.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    worldOutsideSlab.setRobotHeading(0.0F);
    worldOutsideSlab.setObstaclePosition(0, Vec3{5.0F, 0.4F, 2.0F});
    worldOutsideSlab.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F}); // outside X slab
    worldOutsideSlab.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensorOutsideSlab(worldOutsideSlab);

    VirtualWorld worldInsideSlab;
    disableAllObstacles(worldInsideSlab);
    worldInsideSlab.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    worldInsideSlab.setRobotHeading(0.0F);
    worldInsideSlab.setObstaclePosition(0, Vec3{0.0F, 0.4F, 2.0F});
    worldInsideSlab.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F}); // contains X 0.0
    worldInsideSlab.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensorInsideSlab(worldInsideSlab);

    // Act / Assert
    EXPECT_FALSE(sensorOutsideSlab.distanceToNearestObstacle().has_value());

    const std::optional<float> insideDistance = sensorInsideSlab.distanceToNearestObstacle();
    ASSERT_TRUE(insideDistance.has_value());
    EXPECT_TRUE(std::isfinite(*insideDistance));
    EXPECT_NEAR(*insideDistance, 1.35F, kEpsilon);
}

// 15: TangentOrBoundaryHitIsHandledDeterministically
TEST(VirtualDistanceSensorTest, TangentOrBoundaryHitIsHandledDeterministically)
{
    // Arrange: near face placed exactly at kMaximumRange away from the
    // sensor - a boundary case that must resolve the same way every call.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    // Sensor origin Z = 0.25; near face must land at Z = 0.25 + 2.5 = 2.75,
    // so with a 0.8-deep obstacle (half 0.4), center Z = 3.15.
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 3.15F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act
    const std::optional<float> first = sensor.distanceToNearestObstacle();
    const std::optional<float> second = sensor.distanceToNearestObstacle();

    // Assert: deterministic (repeated calls agree) and lands exactly at the
    // boundary.
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_FLOAT_EQ(*first, *second);
    EXPECT_NEAR(*first, VirtualDistanceSensor::kMaximumRange, kEpsilon);
}

// 16: DetectionIsFalseAboveThreshold
TEST(VirtualDistanceSensorTest, DetectionIsFalseAboveThreshold)
{
    // Arrange: distance 1.2 > kDetectionDistance (1.0).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act / Assert
    EXPECT_FALSE(sensor.obstacleDetected());
}

// 17: DetectionIsTrueAtThreshold
TEST(VirtualDistanceSensorTest, DetectionIsTrueAtThreshold)
{
    // Arrange: near face at Z 1.25, sensor origin Z 0.25 - distance exactly
    // 1.0 == kDetectionDistance.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 1.65F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act
    const std::optional<float> distance = sensor.distanceToNearestObstacle();

    // Assert
    ASSERT_TRUE(distance.has_value());
    EXPECT_NEAR(*distance, VirtualDistanceSensor::kDetectionDistance, kEpsilon);
    EXPECT_TRUE(sensor.obstacleDetected());
}

// 18: DetectionIsTrueBelowThreshold
TEST(VirtualDistanceSensorTest, DetectionIsTrueBelowThreshold)
{
    // Arrange: near face at Z 0.6, sensor origin Z 0.4 - distance 0.2 <
    // kDetectionDistance.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 1.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualDistanceSensor sensor(world);

    // Act / Assert
    EXPECT_TRUE(sensor.obstacleDetected());
}
