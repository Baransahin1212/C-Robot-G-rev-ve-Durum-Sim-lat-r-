#include <gtest/gtest.h>

#include "robot/visual/VirtualCliffSensor.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::visual::CliffSensorPosition;
using robot::visual::CliffSensorReadings;
using robot::visual::cliffSensorWorldPosition;
using robot::visual::computeCliffSensorReadings;
using robot::visual::isPointOnTable;
using robot::visual::RobotPose;
using robot::visual::TableSurface;
using robot::visual::Vec3;
using robot::visual::VirtualCliffSensor;
using robot::visual::VirtualWorld;

constexpr float kEpsilon = 0.001F;

} // namespace

// 1: RobotCenteredAllSensorsSafe
TEST(VirtualCliffSensorTest, RobotCenteredAllSensorsSafe)
{
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    VirtualCliffSensor sensor(world);

    const CliffSensorReadings readings = sensor.readings();

    EXPECT_FALSE(readings.frontLeft);
    EXPECT_FALSE(readings.frontRight);
    EXPECT_FALSE(readings.rearLeft);
    EXPECT_FALSE(readings.rearRight);
    EXPECT_FALSE(readings.anyCliff());
}

// 2: FrontLeftDetectsEdge
//
// Heading 45 deliberately used here (and in tests 3-4-5 below): at any
// axis-aligned heading (0/90/180/270), the FrontLeft/RearLeft corners
// always share one world coordinate and FrontRight/RearRight the other,
// so a straight table edge can only ever trip a PAIR of corners at once,
// never exactly one - see MultipleSensorsCanDetectEdge below for that
// axis-aligned case. A 45-degree heading gives all four corners distinct
// (x, z) offsets, so a single corner can be isolated.
TEST(VirtualCliffSensorTest, FrontLeftDetectsEdge)
{
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 5.6F});
    world.setRobotHeading(45.0F);
    VirtualCliffSensor sensor(world);

    const CliffSensorReadings readings = sensor.readings();

    EXPECT_TRUE(readings.frontLeft);
    EXPECT_FALSE(readings.frontRight);
    EXPECT_FALSE(readings.rearLeft);
    EXPECT_FALSE(readings.rearRight);
}

// 3: FrontRightDetectsEdge
TEST(VirtualCliffSensorTest, FrontRightDetectsEdge)
{
    VirtualWorld world;
    world.setRobotPosition(Vec3{5.6F, 0.125F, 0.0F});
    world.setRobotHeading(45.0F);
    VirtualCliffSensor sensor(world);

    const CliffSensorReadings readings = sensor.readings();

    EXPECT_FALSE(readings.frontLeft);
    EXPECT_TRUE(readings.frontRight);
    EXPECT_FALSE(readings.rearLeft);
    EXPECT_FALSE(readings.rearRight);
}

// 4: RearLeftDetectsEdge
TEST(VirtualCliffSensorTest, RearLeftDetectsEdge)
{
    VirtualWorld world;
    world.setRobotPosition(Vec3{-5.6F, 0.125F, 0.0F});
    world.setRobotHeading(45.0F);
    VirtualCliffSensor sensor(world);

    const CliffSensorReadings readings = sensor.readings();

    EXPECT_FALSE(readings.frontLeft);
    EXPECT_FALSE(readings.frontRight);
    EXPECT_TRUE(readings.rearLeft);
    EXPECT_FALSE(readings.rearRight);
}

// 5: RearRightDetectsEdge
TEST(VirtualCliffSensorTest, RearRightDetectsEdge)
{
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, -5.6F});
    world.setRobotHeading(45.0F);
    VirtualCliffSensor sensor(world);

    const CliffSensorReadings readings = sensor.readings();

    EXPECT_FALSE(readings.frontLeft);
    EXPECT_FALSE(readings.frontRight);
    EXPECT_FALSE(readings.rearLeft);
    EXPECT_TRUE(readings.rearRight);
}

// 6: HeadingRotationMovesSensorPositionsCorrectly
TEST(VirtualCliffSensorTest, HeadingRotationMovesSensorPositionsCorrectly)
{
    const RobotPose poseHeading0{Vec3{0.0F, 0.125F, 0.0F}, 0.0F};
    const RobotPose poseHeading90{Vec3{0.0F, 0.125F, 0.0F}, 90.0F};

    const Vec3 frontLeftAt0 = cliffSensorWorldPosition(poseHeading0, CliffSensorPosition::FrontLeft);
    const Vec3 frontLeftAt90 = cliffSensorWorldPosition(poseHeading90, CliffSensorPosition::FrontLeft);

    // Heading 0: FrontLeft = center + (-0.3, 0, 0.4). Heading 90: FrontLeft
    // = center + (0.4, 0, 0.3) - genuinely different world positions.
    EXPECT_NEAR(frontLeftAt0.x, -0.3F, kEpsilon);
    EXPECT_NEAR(frontLeftAt0.z, 0.4F, kEpsilon);
    EXPECT_NEAR(frontLeftAt90.x, 0.4F, kEpsilon);
    EXPECT_NEAR(frontLeftAt90.z, 0.3F, kEpsilon);
}

// 7: Heading90Works
TEST(VirtualCliffSensorTest, Heading90Works)
{
    const RobotPose pose{Vec3{1.0F, 0.125F, 2.0F}, 90.0F};

    const Vec3 frontLeft = cliffSensorWorldPosition(pose, CliffSensorPosition::FrontLeft);

    EXPECT_NEAR(frontLeft.x, 1.4F, kEpsilon);
    EXPECT_NEAR(frontLeft.z, 2.3F, kEpsilon);
}

// 8: Heading180Works
TEST(VirtualCliffSensorTest, Heading180Works)
{
    const RobotPose pose{Vec3{1.0F, 0.125F, 2.0F}, 180.0F};

    const Vec3 frontLeft = cliffSensorWorldPosition(pose, CliffSensorPosition::FrontLeft);

    EXPECT_NEAR(frontLeft.x, 1.3F, kEpsilon);
    EXPECT_NEAR(frontLeft.z, 1.6F, kEpsilon);
}

// 9: Heading270Works
TEST(VirtualCliffSensorTest, Heading270Works)
{
    const RobotPose pose{Vec3{1.0F, 0.125F, 2.0F}, 270.0F};

    const Vec3 frontLeft = cliffSensorWorldPosition(pose, CliffSensorPosition::FrontLeft);

    EXPECT_NEAR(frontLeft.x, 0.6F, kEpsilon);
    EXPECT_NEAR(frontLeft.z, 1.7F, kEpsilon);
}

// 10: SensorExactlyOnBoundaryHandledDeterministically
TEST(VirtualCliffSensorTest, SensorExactlyOnBoundaryHandledDeterministically)
{
    const TableSurface table{-6.0F, 6.0F, -6.0F, 6.0F};
    const Vec3 pointOnBoundary{6.0F, 0.125F, 0.0F};

    const bool first = isPointOnTable(pointOnBoundary, table);
    const bool second = isPointOnTable(pointOnBoundary, table);

    // Exactly on the boundary counts as still-supported (inclusive), and
    // is deterministic across repeated calls.
    EXPECT_TRUE(first);
    EXPECT_EQ(first, second);
}

// 11: MultipleSensorsCanDetectEdge
TEST(VirtualCliffSensorTest, MultipleSensorsCanDetectEdge)
{
    // Heading 0: FrontLeft/FrontRight share Z 0.4 ahead of center - both
    // trip together when the front edge crosses the table's Z boundary.
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 5.7F});
    world.setRobotHeading(0.0F);
    VirtualCliffSensor sensor(world);

    const CliffSensorReadings readings = sensor.readings();

    EXPECT_TRUE(readings.frontLeft);
    EXPECT_TRUE(readings.frontRight);
    EXPECT_FALSE(readings.rearLeft);
    EXPECT_FALSE(readings.rearRight);
    EXPECT_TRUE(readings.anyFrontCliff());
    EXPECT_FALSE(readings.anyRearCliff());
    EXPECT_TRUE(readings.anyCliff());
    EXPECT_FALSE(readings.allCliff());
}

// 12: RobotNearButNotOverEdgeIsSafe
TEST(VirtualCliffSensorTest, RobotNearButNotOverEdgeIsSafe)
{
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 5.5F});
    world.setRobotHeading(0.0F);
    VirtualCliffSensor sensor(world);

    const CliffSensorReadings readings = sensor.readings();

    EXPECT_FALSE(readings.anyCliff());
}

// --- computeCliffSensorReadings() reused directly (not through
// VirtualCliffSensor/VirtualWorld) - proves it is a genuinely pure
// function, exactly as VirtualRobotHardware's table-support guard relies
// on for a not-yet-committed proposed pose. ---

TEST(VirtualCliffSensorTest, ComputeCliffSensorReadingsIsPureAndReusable)
{
    const TableSurface table{-6.0F, 6.0F, -6.0F, 6.0F};
    const RobotPose safePose{Vec3{0.0F, 0.125F, 0.0F}, 0.0F};
    const RobotPose unsafePose{Vec3{0.0F, 0.125F, 5.7F}, 0.0F};

    EXPECT_FALSE(computeCliffSensorReadings(safePose, table).anyCliff());
    EXPECT_TRUE(computeCliffSensorReadings(unsafePose, table).anyCliff());
}
