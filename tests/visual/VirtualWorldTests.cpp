#include <gtest/gtest.h>

#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::visual::VirtualWorld;

} // namespace

TEST(VirtualWorldTest, RobotStartsAtDeterministicPosition)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert
    EXPECT_FLOAT_EQ(world.robotPose().position.x, -3.0F);
    EXPECT_FLOAT_EQ(world.robotPose().position.z, 1.0F);
}

TEST(VirtualWorldTest, RobotStartsAboveGroundByHalfBodyHeight)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert: resting on the ground plane, not embedded in or floating
    // above it.
    EXPECT_GT(world.robotPose().position.y, 0.0F);
    EXPECT_LT(world.robotPose().position.y, 1.0F);
}

TEST(VirtualWorldTest, RobotStartsWithZeroHeading)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert
    EXPECT_FLOAT_EQ(world.robotPose().headingDegrees, 0.0F);
}

TEST(VirtualWorldTest, WorldContainsExpectedObstacleCount)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert: "3 or 4 rectangular boxes" per Phase 13M's brief.
    EXPECT_EQ(world.obstacles().size(), 4u);
}

TEST(VirtualWorldTest, ObstaclesHaveNonZeroSize)
{
    // Arrange
    VirtualWorld world;

    // Act / Assert
    for (const auto& obstacle : world.obstacles())
    {
        EXPECT_GT(obstacle.size.x, 0.0F);
        EXPECT_GT(obstacle.size.y, 0.0F);
        EXPECT_GT(obstacle.size.z, 0.0F);
    }
}

TEST(VirtualWorldTest, BasePlatformIsPositionedAwayFromRobotStart)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert: base is "near one corner", robot is "near center-left" -
    // they must not coincide.
    EXPECT_NE(world.basePlatform().position.x, world.robotPose().position.x);
    EXPECT_NE(world.basePlatform().position.z, world.robotPose().position.z);
}

TEST(VirtualWorldTest, BasePlatformHasNonZeroFootprint)
{
    // Arrange / Act
    VirtualWorld world;

    // Assert
    EXPECT_GT(world.basePlatform().size.x, 0.0F);
    EXPECT_GT(world.basePlatform().size.z, 0.0F);
}

TEST(VirtualWorldTest, ConstructingMultipleWorldsIsDeterministic)
{
    // Arrange / Act
    VirtualWorld first;
    VirtualWorld second;

    // Assert: no randomness, no shared mutable state between instances.
    EXPECT_FLOAT_EQ(first.robotPose().position.x, second.robotPose().position.x);
    EXPECT_FLOAT_EQ(first.robotPose().position.y, second.robotPose().position.y);
    EXPECT_FLOAT_EQ(first.robotPose().position.z, second.robotPose().position.z);
    EXPECT_EQ(first.obstacles().size(), second.obstacles().size());
}
