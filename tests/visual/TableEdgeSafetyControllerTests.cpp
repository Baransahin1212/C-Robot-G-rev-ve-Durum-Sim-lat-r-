#include <cmath>

#include <gtest/gtest.h>

#include "robot/visual/TableEdgeSafetyController.hpp"
#include "robot/visual/VirtualCliffSensor.hpp"

namespace
{

using robot::visual::CliffSensorReadings;
using robot::visual::computeCliffSensorReadings;
using robot::visual::RobotPose;
using robot::visual::TableEdgeSafetyController;
using robot::visual::TableSurface;
using robot::visual::Vec3;
using robot::visual::WheelSpeeds;

CliffSensorReadings allSafe()
{
    return CliffSensorReadings{false, false, false, false};
}

CliffSensorReadings frontCliff()
{
    return CliffSensorReadings{true, false, false, false};
}

CliffSensorReadings rearCliff()
{
    return CliffSensorReadings{false, false, true, false};
}

// A representative pose/table used by tests that only care about pure
// sensor-driven state transitions, not the specific recovery-target
// heading math (which the geometry-driven tests below cover directly).
// Any pose works for these - the target heading it produces is never
// asserted on by these tests.
const TableSurface kTable{-6.0F, 6.0F, -6.0F, 6.0F};
const RobotPose kArbitraryPose{Vec3{0.0F, 0.125F, 5.6F}, 0.0F};

} // namespace

// 1: DefaultsInactive
TEST(TableEdgeSafetyControllerTest, DefaultsInactive)
{
    TableEdgeSafetyController controller;

    EXPECT_FALSE(controller.active());
    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Inactive);
}

// 2: FrontCliffActivatesRecovery
TEST(TableEdgeSafetyControllerTest, FrontCliffActivatesRecovery)
{
    TableEdgeSafetyController controller;

    controller.update(frontCliff(), kArbitraryPose, kTable);

    EXPECT_TRUE(controller.active());
    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);
}

// 3: FrontCliffCommandsReverseInitially
TEST(TableEdgeSafetyControllerTest, FrontCliffCommandsReverseInitially)
{
    TableEdgeSafetyController controller;
    controller.update(frontCliff(), kArbitraryPose, kTable);

    const WheelSpeeds speeds = controller.recoveryWheelSpeeds();

    EXPECT_LT(speeds.left, 0.0F);
    EXPECT_LT(speeds.right, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
}

// 4: RearCliffCommandsForwardInitially
TEST(TableEdgeSafetyControllerTest, RearCliffCommandsForwardInitially)
{
    TableEdgeSafetyController controller;
    controller.update(rearCliff(), kArbitraryPose, kTable);

    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::MovingForwardFromRearEdge);
    const WheelSpeeds speeds = controller.recoveryWheelSpeeds();

    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_GT(speeds.right, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
}

// 5: RecoveryTransitionsToTurn
TEST(TableEdgeSafetyControllerTest, RecoveryTransitionsToTurn)
{
    TableEdgeSafetyController controller;
    controller.update(frontCliff(), kArbitraryPose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);

    // Front danger cleared, but the robot has not yet turned - this must
    // transition to Turning, not straight to Inactive.
    controller.update(allSafe(), kArbitraryPose, kTable);

    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);
    EXPECT_TRUE(controller.active());
}

// 6: TurnUsesOppositeWheelSigns
TEST(TableEdgeSafetyControllerTest, TurnUsesOppositeWheelSigns)
{
    TableEdgeSafetyController controller;
    controller.update(frontCliff(), kArbitraryPose, kTable);
    controller.update(allSafe(), kArbitraryPose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);

    const WheelSpeeds speeds = controller.recoveryWheelSpeeds();

    EXPECT_FLOAT_EQ(speeds.left, -speeds.right);
    EXPECT_NE(speeds.left, 0.0F);
}

// 7: RemainsActiveUntilSafe
TEST(TableEdgeSafetyControllerTest, RemainsActiveUntilSafe)
{
    TableEdgeSafetyController controller;
    controller.update(frontCliff(), kArbitraryPose, kTable);
    controller.update(allSafe(), kArbitraryPose, kTable); // -> Turning
    ASSERT_TRUE(controller.active());

    // Turning while a (possibly different) corner is still off the table
    // must not release - only a fully-safe reading may (heading
    // alignment aside).
    controller.update(CliffSensorReadings{false, false, false, true}, kArbitraryPose, kTable); // rearRight off
    EXPECT_TRUE(controller.active());
    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);

    controller.update(CliffSensorReadings{true, false, false, false}, kArbitraryPose, kTable); // frontLeft off now
    EXPECT_TRUE(controller.active());
    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);
}

// --- Bugfix regression tests (human manual validation) ---
//
// The original release condition was `!readings.anyCliff()` alone. On a
// straight edge, BackingAway typically clears the triggering corner with
// only a small margin, so a single simulation step's rotation was
// frequently already enough to satisfy "all sensors safe" - releasing
// with an almost-unchanged, still-edge-facing heading. These tests
// reproduce that exact defect class directly: readings ARE genuinely all
// safe, yet release must still be withheld because the heading has not
// meaningfully turned toward the table interior.

// 8: DoesNotReleaseOnSensorsAloneWhenHeadingStillFacesTheEdge
//
// This is the mandatory regression test for the human-observed defect
// (Phase 13S bugfix brief, section 8): constructs the exact scenario the
// old implementation got wrong - BackingAway stops with a modest margin
// (comfortably realistic for a coarser simulation step than the 0.05s
// used elsewhere in this codebase, or a couple of accumulated backing
// steps), Turning begins, and a SINGLE simulation step's worth of
// rotation (~5.73 degrees at kRecoveryTurnSpeed over 0.05s - the same
// value used throughout this codebase's other 0.05s-step tests) is
// applied. At this position/rotation the readings are ALREADY back to
// fully safe (verified directly below) - under the old
// `!readings.anyCliff()`-only condition this would have released
// immediately; the fixed controller must not, since heading error is
// still large (recovery target points roughly opposite the approach
// heading, ~180 degrees away).
TEST(TableEdgeSafetyControllerTest, DoesNotReleaseOnSensorsAloneWhenHeadingStillFacesTheEdge)
{
    TableEdgeSafetyController controller;

    // Trigger at the +Z edge, heading 0 (facing straight at the edge),
    // robot on the table's X centerline so the recovery target heading
    // works out to a clean 180 degrees (straight back toward the table
    // center).
    RobotPose pose{Vec3{0.0F, 0.125F, 5.65F}, 0.0F};
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);

    // BackingAway clears with a modest margin (front corner at Z 5.7,
    // comfortably inside the Z 6.0 boundary - not hair-trigger-exact).
    pose.position.z = 5.3F;
    ASSERT_FALSE(computeCliffSensorReadings(pose, kTable).anyCliff()); // confirm: genuinely all safe already
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);

    // Apply exactly one simulation step's worth of rotation.
    pose.headingDegrees = 5.73F;
    ASSERT_FALSE(computeCliffSensorReadings(pose, kTable).anyCliff()); // still all safe at this rotated pose
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);

    // Assert: must still be actively recovering - readings being safe is
    // not sufficient on its own.
    EXPECT_TRUE(controller.active());
    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);
    EXPECT_GT(std::fabs(controller.currentHeadingErrorDegrees()), TableEdgeSafetyController::kRecoveryHeadingToleranceDegrees);
}

// 9: ReleasesOnlyAfterSensorsSafeAndHeadingAligned
TEST(TableEdgeSafetyControllerTest, ReleasesOnlyAfterSensorsSafeAndHeadingAligned)
{
    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{0.0F, 0.125F, 5.65F}, 0.0F};
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);

    pose.position.z = 5.6F;
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);

    // Target heading for this on-axis (X 0) trigger position is exactly
    // 180 degrees (straight back toward the table center at (0, 0)).
    EXPECT_NEAR(controller.targetRecoveryHeadingDegrees(), 180.0F, 0.01F);

    // Partially turned - still not aligned - must remain active.
    pose.headingDegrees = 90.0F;
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    EXPECT_TRUE(controller.active());

    // Fully turned to face the table interior - readings are also safe
    // at this heading (raw sensors, no margin) - but RearLeft sits
    // EXACTLY at the table boundary (Z 6.0, since only the front corners
    // were ever pulled back during BackingAway) - not robustly inside by
    // kRecoverySupportMargin. Heading is safe, support is not: this must
    // hand off to AdvancingInward, not release directly (table-edge
    // recovery bugfix #2).
    pose.headingDegrees = 180.0F;
    ASSERT_FALSE(computeCliffSensorReadings(pose, kTable).anyCliff());
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);

    EXPECT_TRUE(controller.active());
    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::AdvancingInward);

    // Drive forward (heading 180 = -Z, toward the table center at Z 0)
    // until the whole footprint is robustly inside the margin.
    pose.position.z = 5.0F;
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);

    EXPECT_FALSE(controller.active());
    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Inactive);
}

// 10: TurnDirectionFollowsShortestPathToTarget
//
// Replaces the old fixed-direction DeterministicTurnDirection test:
// TableEdgeSafetyController's turn direction is now geometry-driven (the
// shorter path to the recovery target heading), unlike
// ReactiveObstacleAvoidance's own always-one-direction policy, which this
// change deliberately does not touch.
TEST(TableEdgeSafetyControllerTest, TurnDirectionFollowsShortestPathToTarget)
{
    // Scenario A: triggered at X -3 - the table center is roughly
    // "toward positive heading" from this position, so the shorter turn
    // is the positive-error direction (left negative, right positive -
    // ReactiveObstacleAvoidance's own convention).
    {
        TableEdgeSafetyController controller;
        RobotPose pose{Vec3{-3.0F, 0.125F, 5.65F}, 0.0F};
        controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
        ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);
        pose.position.z = 5.6F;
        controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
        ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);
        ASSERT_GT(controller.currentHeadingErrorDegrees(), 0.0F);

        const WheelSpeeds speeds = controller.recoveryWheelSpeeds();
        EXPECT_LT(speeds.left, 0.0F);
        EXPECT_GT(speeds.right, 0.0F);
    }

    // Scenario B: triggered at X +3 - the mirror image, so the shorter
    // turn is the opposite (negative-error) direction.
    {
        TableEdgeSafetyController controller;
        RobotPose pose{Vec3{3.0F, 0.125F, 5.65F}, 0.0F};
        controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
        ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);
        pose.position.z = 5.6F;
        controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
        ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);
        ASSERT_LT(controller.currentHeadingErrorDegrees(), 0.0F);

        const WheelSpeeds speeds = controller.recoveryWheelSpeeds();
        EXPECT_GT(speeds.left, 0.0F);
        EXPECT_LT(speeds.right, 0.0F);
    }
}

// 11: TargetHeadingPointsTowardTableCenter
TEST(TableEdgeSafetyControllerTest, TargetHeadingPointsTowardTableCenter)
{
    TableEdgeSafetyController controller;

    // Triggered at the +X edge, heading 90 (facing +X) - table center is
    // due -X from here, i.e. target heading -90 (== 270).
    RobotPose pose{Vec3{5.65F, 0.125F, 0.0F}, 90.0F};
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);

    ASSERT_TRUE(controller.active());
    EXPECT_NEAR(controller.targetRecoveryHeadingDegrees(), 270.0F, 0.01F);
}

// 12: NoTriggerProducesNoOverrideNeed
//
// There is no separate enable/disable toggle for table-edge safety (it
// is a physical safety layer, always active, unlike the `A`-toggled
// ReactiveObstacleAvoidance) - the equivalent "disabled" concept is
// simply "never triggered": readings stay all-safe, the controller stays
// Inactive, and recoveryWheelSpeeds() deterministically returns zero.
TEST(TableEdgeSafetyControllerTest, NoTriggerProducesNoOverrideNeed)
{
    TableEdgeSafetyController controller;

    for (int i = 0; i < 5; ++i)
    {
        controller.update(allSafe(), kArbitraryPose, kTable);
    }

    EXPECT_FALSE(controller.active());
    const WheelSpeeds speeds = controller.recoveryWheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// --- Simultaneous front+rear precedence (documented known limitation) ---

TEST(TableEdgeSafetyControllerTest, SimultaneousFrontAndRearPrefersBackingAway)
{
    TableEdgeSafetyController controller;

    controller.update(CliffSensorReadings{true, false, true, false}, kArbitraryPose, kTable); // front AND rear

    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);
}

// --- Screenshot-condition regression (table-edge recovery bugfix #2) ---
//
// Reproduces the exact human-observed stuck condition: heading has
// already reached the recovery target (error within tolerance), but one
// rear corner (RearLeft here, matching the screenshot) permanently
// reports a cliff. The OLD Turning exit condition
// (`!readings.anyCliff() && headingSafe`) can never be satisfied by pure
// rotation once the heading is already correct - in-place turning never
// translates the robot center, so a corner that is off-table due to
// POSITION (not heading) can never be fixed by continuing to spin. This
// test feeds the SAME "heading already correct, rearLeft still off"
// readings/pose on every call (deliberately never advancing physics) so
// the old implementation would loop in Turning forever - proving the
// defect class deterministically without needing to actually let a test
// hang.
TEST(TableEdgeSafetyControllerTest, TurningTransitionsToAdvancingInwardWhenHeadingSafeButCornerStillEdge)
{
    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{0.0F, 0.125F, 5.65F}, 0.0F};
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);

    pose.position.z = 5.6F;
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);

    // Heading already exactly at the target (180 degrees for this on-axis
    // trigger position - see ReleasesOnlyAfterSensorsSafeAndHeadingAligned
    // above) - error is 0, well within tolerance - but RearLeft
    // permanently reports a cliff, never clearing.
    pose.headingDegrees = 180.0F;
    const CliffSensorReadings stuckReadings{false, false, true, false}; // rearLeft only

    for (int i = 0; i < 50; ++i)
    {
        controller.update(stuckReadings, pose, kTable);
    }

    // REPRODUCTION CHECKPOINT (pre-fix): at this point in the codebase's
    // history, this asserted `controller.state() ==
    // TableEdgeSafetyController::RecoveryState::Turning` after all 50
    // iterations - confirmed to pass against the unfixed implementation,
    // proving the "stuck spinning forever with the heading already
    // correct" defect deterministically before any fix code was written.
    // Now (post-fix) it must NOT still be spinning in place - it must
    // have moved on to translate inward instead.
    // REPRODUCTION CHECKPOINT (pre-fix): confirmed by running this exact
    // test against the unfixed implementation before writing the fix -
    // `controller.state()` was still `Turning` here every time, proving
    // the "stuck spinning forever with the heading already correct"
    // defect deterministically. Post-fix, it must have moved on to
    // translate inward instead.
    EXPECT_NE(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);
    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::AdvancingInward);

    // AdvancingInward must command equal POSITIVE wheel speeds (forward),
    // not an in-place turn.
    const WheelSpeeds speeds = controller.recoveryWheelSpeeds();
    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
}
