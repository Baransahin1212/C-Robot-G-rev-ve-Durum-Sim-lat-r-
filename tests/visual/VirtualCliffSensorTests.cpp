#include <gtest/gtest.h>

#include "robot/visual/VirtualCliffSensor.hpp"
#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualRobot.hpp"

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
namespace RobotDimensions = robot::visual::RobotDimensions;

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
    // Phase 13W final workspace redesign: with the rescaled robot's
    // half-width/half-length (0.20F/0.25F), FrontLeft's Z-offset at a
    // 45-degree heading is (halfLength+halfWidth)/sqrt(2) =~ 0.318F - this
    // position clears the table's 2.0F Z boundary by that corner alone
    // (1.75F + 0.318F > 2.0F) while every other corner stays inside.
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 1.75F});
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
    // Phase 13W final workspace redesign: same corner-offset reasoning as
    // FrontLeftDetectsEdge above, mirrored onto the X boundary (4.0F).
    VirtualWorld world;
    world.setRobotPosition(Vec3{3.75F, 0.125F, 0.0F});
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
    // Phase 13W final workspace redesign: same corner-offset reasoning as
    // FrontLeftDetectsEdge above, mirrored onto the -X boundary (-4.0F).
    VirtualWorld world;
    world.setRobotPosition(Vec3{-3.75F, 0.125F, 0.0F});
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
    // Phase 13W final workspace redesign: same corner-offset reasoning as
    // FrontLeftDetectsEdge above, mirrored onto the -Z boundary (-2.0F).
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, -1.75F});
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

    // Heading 0: FrontLeft = center + (-halfWidth, 0, halfLength) =
    // center + (-0.20, 0, 0.25). Heading 90: FrontLeft = center +
    // (halfLength, 0, halfWidth) = center + (0.25, 0, 0.20) - genuinely
    // different world positions.
    EXPECT_NEAR(frontLeftAt0.x, -0.20F, kEpsilon);
    EXPECT_NEAR(frontLeftAt0.z, 0.25F, kEpsilon);
    EXPECT_NEAR(frontLeftAt90.x, 0.25F, kEpsilon);
    EXPECT_NEAR(frontLeftAt90.z, 0.20F, kEpsilon);
}

// 7: Heading90Works
TEST(VirtualCliffSensorTest, Heading90Works)
{
    const RobotPose pose{Vec3{1.0F, 0.125F, 2.0F}, 90.0F};

    const Vec3 frontLeft = cliffSensorWorldPosition(pose, CliffSensorPosition::FrontLeft);

    EXPECT_NEAR(frontLeft.x, 1.25F, kEpsilon);
    EXPECT_NEAR(frontLeft.z, 2.20F, kEpsilon);
}

// 8: Heading180Works
TEST(VirtualCliffSensorTest, Heading180Works)
{
    const RobotPose pose{Vec3{1.0F, 0.125F, 2.0F}, 180.0F};

    const Vec3 frontLeft = cliffSensorWorldPosition(pose, CliffSensorPosition::FrontLeft);

    EXPECT_NEAR(frontLeft.x, 1.20F, kEpsilon);
    EXPECT_NEAR(frontLeft.z, 1.75F, kEpsilon);
}

// 9: Heading270Works
TEST(VirtualCliffSensorTest, Heading270Works)
{
    const RobotPose pose{Vec3{1.0F, 0.125F, 2.0F}, 270.0F};

    const Vec3 frontLeft = cliffSensorWorldPosition(pose, CliffSensorPosition::FrontLeft);

    EXPECT_NEAR(frontLeft.x, 0.75F, kEpsilon);
    EXPECT_NEAR(frontLeft.z, 1.80F, kEpsilon);
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
    // Heading 0: FrontLeft/FrontRight share Z halfLength (0.25F) ahead of
    // center - both trip together when the front edge crosses the
    // table's Z boundary (2.0F): 1.85F + 0.25F > 2.0F.
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 1.85F});
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
    // Phase 13W human-visual-redesign v2: same margin pattern against the
    // table's new Z half-extent (2.0F, was 6.0F) - safely below the
    // crossing threshold.
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 1.5F});
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
    // 5.85F + halfLength (0.25F) = 6.1F, strictly beyond the 6.0F boundary.
    const RobotPose unsafePose{Vec3{0.0F, 0.125F, 5.85F}, 0.0F};

    EXPECT_FALSE(computeCliffSensorReadings(safePose, table).anyCliff());
    EXPECT_TRUE(computeCliffSensorReadings(unsafePose, table).anyCliff());
}

// --- Phase 13W final workspace redesign: robot scale ---

// 13: CliffSensorsMatchRobotCorners
//
// Each sensor corner must sit exactly at the physical body's own corner -
// derived from RobotDimensions, never a hand-duplicated offset - so
// resizing the robot again later automatically keeps the sensors tied to
// the real footprint.
TEST(VirtualCliffSensorTest, CliffSensorsMatchRobotCorners)
{
    const RobotPose pose{Vec3{0.0F, 0.125F, 0.0F}, 0.0F};
    const Vec3 frontLeft = cliffSensorWorldPosition(pose, CliffSensorPosition::FrontLeft);

    const float halfWidth = RobotDimensions::kBodyWidth / 2.0F;
    const float halfLength = RobotDimensions::kBodyLength / 2.0F;
    EXPECT_NEAR(frontLeft.x, -halfWidth, kEpsilon);
    EXPECT_NEAR(frontLeft.z, halfLength, kEpsilon);
}

// 14: SmallerRobotGeometryUsesCorrectCliffCorners
//
// Table-edge recovery bugfix #3 regression: pins the POST-rescale
// (0.40 x 0.50, not the old 0.60 x 0.80) body dimensions explicitly, and
// checks all four corners at once, not just FrontLeft - a stale/half-
// updated constant (as happened to kDockHousingIndex during the same
// rescale) would otherwise silently keep using stale geometry for cliff
// sensing/recovery while everything else moved to the smaller footprint.
TEST(VirtualCliffSensorTest, SmallerRobotGeometryUsesCorrectCliffCorners)
{
    ASSERT_FLOAT_EQ(RobotDimensions::kBodyWidth, 0.40F);
    ASSERT_FLOAT_EQ(RobotDimensions::kBodyLength, 0.50F);

    const RobotPose pose{Vec3{0.0F, 0.125F, 0.0F}, 0.0F};
    const float halfWidth = RobotDimensions::kBodyWidth / 2.0F;
    const float halfLength = RobotDimensions::kBodyLength / 2.0F;

    const Vec3 frontLeft = cliffSensorWorldPosition(pose, CliffSensorPosition::FrontLeft);
    const Vec3 frontRight = cliffSensorWorldPosition(pose, CliffSensorPosition::FrontRight);
    const Vec3 rearLeft = cliffSensorWorldPosition(pose, CliffSensorPosition::RearLeft);
    const Vec3 rearRight = cliffSensorWorldPosition(pose, CliffSensorPosition::RearRight);

    EXPECT_NEAR(frontLeft.x, -halfWidth, kEpsilon);
    EXPECT_NEAR(frontLeft.z, halfLength, kEpsilon);
    EXPECT_NEAR(frontRight.x, halfWidth, kEpsilon);
    EXPECT_NEAR(frontRight.z, halfLength, kEpsilon);
    EXPECT_NEAR(rearLeft.x, -halfWidth, kEpsilon);
    EXPECT_NEAR(rearLeft.z, -halfLength, kEpsilon);
    EXPECT_NEAR(rearRight.x, halfWidth, kEpsilon);
    EXPECT_NEAR(rearRight.z, -halfLength, kEpsilon);
}
