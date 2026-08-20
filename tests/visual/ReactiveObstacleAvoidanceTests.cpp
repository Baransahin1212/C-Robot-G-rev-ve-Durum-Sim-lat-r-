#include <cmath>

#include <gtest/gtest.h>

#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"

namespace
{

using robot::visual::DifferentialDrive;
using robot::visual::ReactiveObstacleAvoidance;
using robot::visual::RobotPose;
using robot::visual::VirtualRobotHardware;
using robot::visual::WheelSpeeds;

} // namespace

// 1: ReturnsDeterministicTurnWheelSpeeds
TEST(ReactiveObstacleAvoidanceTest, ReturnsDeterministicTurnWheelSpeeds)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;

    // Act
    const WheelSpeeds first = avoidance.avoidanceWheelSpeeds();
    const WheelSpeeds second = avoidance.avoidanceWheelSpeeds();

    // Assert: stateless and deterministic - repeated calls agree.
    EXPECT_FLOAT_EQ(first.left, second.left);
    EXPECT_FLOAT_EQ(first.right, second.right);
    EXPECT_FLOAT_EQ(first.left, -ReactiveObstacleAvoidance::kTurnWheelSpeed);
    EXPECT_FLOAT_EQ(first.right, ReactiveObstacleAvoidance::kTurnWheelSpeed);
}

// 2: TurnSpeedsHaveOppositeSigns
TEST(ReactiveObstacleAvoidanceTest, TurnSpeedsHaveOppositeSigns)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;

    // Act
    const WheelSpeeds speeds = avoidance.avoidanceWheelSpeeds();

    // Assert
    EXPECT_LT(speeds.left, 0.0F);
    EXPECT_GT(speeds.right, 0.0F);
}

// 3: TurnProducesZeroLinearVelocityConceptually
TEST(ReactiveObstacleAvoidanceTest, TurnProducesZeroLinearVelocityConceptually)
{
    // Arrange: v = (vRight + vLeft) / 2, per DifferentialDrive's own
    // equations - opposite-magnitude wheel speeds must average to zero.
    ReactiveObstacleAvoidance avoidance;

    // Act
    const WheelSpeeds speeds = avoidance.avoidanceWheelSpeeds();
    const float linearVelocity = (speeds.left + speeds.right) / 2.0F;

    // Assert
    EXPECT_FLOAT_EQ(linearVelocity, 0.0F);
}

// 4: TurnDirectionMatchesDifferentialDriveConvention
TEST(ReactiveObstacleAvoidanceTest, TurnDirectionMatchesDifferentialDriveConvention)
{
    // Arrange: feed the avoidance wheel speeds through the real
    // DifferentialDrive (not an assumption about its convention) and
    // confirm heading actually increases, matching
    // DifferentialDriveTests.cpp's own TurningDirectionMatchesConvention.
    ReactiveObstacleAvoidance avoidance;
    const WheelSpeeds speeds = avoidance.avoidanceWheelSpeeds();
    DifferentialDrive drive;
    drive.setWheelSpeeds(speeds.left, speeds.right);
    RobotPose pose{};

    // Act
    drive.update(pose, 0.1F);

    // Assert: heading increased, position stayed put (pure in-place
    // rotation).
    EXPECT_GT(pose.headingDegrees, 0.0F);
    EXPECT_NEAR(pose.position.x, 0.0F, 0.001F);
    EXPECT_NEAR(pose.position.z, 0.0F, 0.001F);
}

// 5: TurnSpeedMagnitudeIsWithinForwardSpeedRange
TEST(ReactiveObstacleAvoidanceTest, TurnSpeedMagnitudeIsWithinForwardSpeedRange)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;

    // Act
    const WheelSpeeds speeds = avoidance.avoidanceWheelSpeeds();

    // Assert: a turn maneuver stays within the normal forward-speed
    // envelope - not an unreasonably fast spin.
    EXPECT_LE(std::fabs(speeds.left), VirtualRobotHardware::kForwardWheelSpeed);
    EXPECT_LE(std::fabs(speeds.right), VirtualRobotHardware::kForwardWheelSpeed);
}
