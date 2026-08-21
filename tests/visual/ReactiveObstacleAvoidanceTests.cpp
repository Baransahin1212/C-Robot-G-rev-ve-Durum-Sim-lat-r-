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

// --- Latch lifecycle (Phase 13R) ---
//
// ReactiveObstacleAvoidance::update() replaces the Phase 13Q "recompute
// shouldAvoidObstacle from scratch every frame" pattern with a small
// stateful latch: once triggered, it must stay active across calls even
// after the trigger condition itself goes false (e.g. the FSM's real
// return to Moving on the ObstacleCleared edge), until the caller reports
// the forward BODY corridor is actually clear (ForwardClearanceProbe).
// This is the direct fix for the Phase 13Q limitation documented in
// docs/technical-decisions.md.

// 6: DefaultsInactive
TEST(ReactiveObstacleAvoidanceTest, DefaultsInactive)
{
    // Arrange / Act
    ReactiveObstacleAvoidance avoidance;

    // Assert
    EXPECT_FALSE(avoidance.active());
}

// 7: TriggerActivatesAvoidance
TEST(ReactiveObstacleAvoidanceTest, TriggerActivatesAvoidance)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;

    // Act
    avoidance.update(/*enabled=*/true, /*triggerAvoidance=*/true, /*forwardCorridorClear=*/false);

    // Assert
    EXPECT_TRUE(avoidance.active());
}

// 8: ActiveReturnsDeterministicTurnSpeeds
TEST(ReactiveObstacleAvoidanceTest, ActiveReturnsDeterministicTurnSpeeds)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false);
    ASSERT_TRUE(avoidance.active());

    // Act
    const WheelSpeeds first = avoidance.avoidanceWheelSpeeds();
    const WheelSpeeds second = avoidance.avoidanceWheelSpeeds();

    // Assert
    EXPECT_FLOAT_EQ(first.left, second.left);
    EXPECT_FLOAT_EQ(first.right, second.right);
    EXPECT_FLOAT_EQ(first.left, -ReactiveObstacleAvoidance::kTurnWheelSpeed);
    EXPECT_FLOAT_EQ(first.right, ReactiveObstacleAvoidance::kTurnWheelSpeed);
}

// 9: RemainsActiveWhenTriggerDisappearsButClearanceStillBlocked
//
// This is the exact Phase 13R fix: the trigger condition going false (the
// FSM has already returned to Moving on the real ObstacleCleared edge)
// must NOT by itself deactivate the latch while the body corridor is
// still blocked.
TEST(ReactiveObstacleAvoidanceTest, RemainsActiveWhenTriggerDisappearsButClearanceStillBlocked)
{
    // Arrange: triggered while WaitingForObstacleClear + sensor detected.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false);
    ASSERT_TRUE(avoidance.active());

    // Act: the trigger condition is now false (FSM moved on to Moving),
    // but forwardCorridorClear is still false (body still blocked).
    avoidance.update(true, /*triggerAvoidance=*/false, /*forwardCorridorClear=*/false);

    // Assert: still latched active.
    EXPECT_TRUE(avoidance.active());
}

// 10: DeactivatesOnlyWhenClearanceBecomesSafe
TEST(ReactiveObstacleAvoidanceTest, DeactivatesOnlyWhenClearanceBecomesSafe)
{
    // Arrange: latched active, trigger already gone (matches test 9).
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false);
    avoidance.update(true, false, false);
    ASSERT_TRUE(avoidance.active());

    // Act: forwardCorridorClear finally reports true.
    avoidance.update(true, false, /*forwardCorridorClear=*/true);

    // Assert
    EXPECT_FALSE(avoidance.active());
}

// 11: DisabledForcesInactive
TEST(ReactiveObstacleAvoidanceTest, DisabledForcesInactive)
{
    // Arrange: latched active.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false);
    ASSERT_TRUE(avoidance.active());

    // Act: `A` toggled off - forces inactive immediately regardless of
    // clearance state.
    avoidance.update(/*enabled=*/false, /*triggerAvoidance=*/true, /*forwardCorridorClear=*/false);

    // Assert
    EXPECT_FALSE(avoidance.active());
}

// 12: ReEnableDoesNotActivateWithoutTrigger
TEST(ReactiveObstacleAvoidanceTest, ReEnableDoesNotActivateWithoutTrigger)
{
    // Arrange: was active, then disabled.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false);
    avoidance.update(false, false, false);
    ASSERT_FALSE(avoidance.active());

    // Act: re-enabled, but no trigger this frame.
    avoidance.update(/*enabled=*/true, /*triggerAvoidance=*/false, false);

    // Assert
    EXPECT_FALSE(avoidance.active());
}

// 13: ReEnableWithTriggerActivates
TEST(ReactiveObstacleAvoidanceTest, ReEnableWithTriggerActivates)
{
    // Arrange: was active, then disabled.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false);
    avoidance.update(false, false, false);
    ASSERT_FALSE(avoidance.active());

    // Act: re-enabled, with a fresh trigger this frame.
    avoidance.update(true, /*triggerAvoidance=*/true, false);

    // Assert
    EXPECT_TRUE(avoidance.active());
}

// 14: TurnDirectionConventionStillCorrect
TEST(ReactiveObstacleAvoidanceTest, TurnDirectionConventionStillCorrect)
{
    // Arrange: feed the avoidance wheel speeds through the real
    // DifferentialDrive while latched active - same convention proof as
    // TurnDirectionMatchesDifferentialDriveConvention above, now exercised
    // through the stateful lifecycle.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false);
    const WheelSpeeds speeds = avoidance.avoidanceWheelSpeeds();
    DifferentialDrive drive;
    drive.setWheelSpeeds(speeds.left, speeds.right);
    RobotPose pose{};

    // Act
    drive.update(pose, 0.1F);

    // Assert
    EXPECT_GT(pose.headingDegrees, 0.0F);
    EXPECT_NEAR(pose.position.x, 0.0F, 0.001F);
    EXPECT_NEAR(pose.position.z, 0.0F, 0.001F);
}

// 15: TurnSpeedStillWithinExpectedRange
TEST(ReactiveObstacleAvoidanceTest, TurnSpeedStillWithinExpectedRange)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false);

    // Act
    const WheelSpeeds speeds = avoidance.avoidanceWheelSpeeds();

    // Assert
    EXPECT_LE(std::fabs(speeds.left), VirtualRobotHardware::kForwardWheelSpeed);
    EXPECT_LE(std::fabs(speeds.right), VirtualRobotHardware::kForwardWheelSpeed);
    EXPECT_LT(speeds.left, 0.0F);
    EXPECT_GT(speeds.right, 0.0F);
}
