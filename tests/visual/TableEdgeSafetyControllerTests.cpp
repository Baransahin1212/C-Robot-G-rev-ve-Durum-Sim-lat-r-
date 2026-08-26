#include <cmath>

#include <gtest/gtest.h>

#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/TableEdgeSafetyController.hpp"
#include "robot/visual/VirtualCliffSensor.hpp"

namespace
{

using robot::visual::aggregateTableOverhang;
using robot::visual::areAllCornersSafelyInsideTable;
using robot::visual::CliffSensorReadings;
using robot::visual::computeCliffSensorReadings;
using robot::visual::DifferentialDrive;
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

// Drives a real TableEdgeSafetyController through an actual recovery
// incident using a real DifferentialDrive (kDefaultWheelTrack, tied to
// RobotDimensions::kBodyWidth - never a hand-picked wheelbase) for
// kinematics - the SAME per-step wiring main3d.cpp itself uses each
// frame (readings -> controller.update() -> recoveryWheelSpeeds() ->
// drive.setWheelSpeeds() -> drive.update(pose, dt)), just without
// VirtualRobotHardware/VirtualWorld's own additional state. Table-edge
// recovery bugfix #3 (BackingAway/MovingForwardFromRearEdge/
// AdvancingInward blindly translating along a heading-derived direction
// that does not reduce the actual triggering overhang) can only be
// proven fixed by actually letting a full incident play out like this -
// hand-crafted single-step scenarios (as used elsewhere in this file)
// cannot exercise it. Returns true once the controller releases
// (state() == Inactive) within maxSteps 0.05s steps, false if it never
// does - a false return is itself the deadlock this bugfix eliminates.
bool driveRecoveryToCompletion(TableEdgeSafetyController& controller, RobotPose& pose, const TableSurface& table,
                                int maxSteps = 400)
{
    DifferentialDrive drive;
    constexpr float dt = 0.05F;
    for (int step = 0; step < maxSteps; ++step)
    {
        const CliffSensorReadings readings = computeCliffSensorReadings(pose, table);
        controller.update(readings, pose, table);
        if (!controller.active())
        {
            return true;
        }
        const WheelSpeeds speeds = controller.recoveryWheelSpeeds();
        drive.setWheelSpeeds(speeds.left, speeds.right);
        drive.update(pose, dt);
    }
    return false;
}

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
    RobotPose pose{Vec3{0.0F, 0.125F, 5.80F}, 0.0F};
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);

    // BackingAway clears with a modest margin (front corner at Z 5.7,
    // comfortably inside the Z 6.0 boundary - not hair-trigger-exact).
    pose.position.z = 5.45F;
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
    RobotPose pose{Vec3{0.0F, 0.125F, 5.80F}, 0.0F};
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);

    pose.position.z = 5.75F;
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
        RobotPose pose{Vec3{-3.0F, 0.125F, 5.80F}, 0.0F};
        controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
        ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);
        pose.position.z = 5.75F;
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
        RobotPose pose{Vec3{3.0F, 0.125F, 5.80F}, 0.0F};
        controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
        ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);
        pose.position.z = 5.75F;
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
    RobotPose pose{Vec3{5.80F, 0.125F, 0.0F}, 90.0F};
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
    RobotPose pose{Vec3{0.0F, 0.125F, 5.80F}, 0.0F};
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);

    pose.position.z = 5.75F;
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

// --- Table-edge recovery bugfix #3 regression (Phase 13W human
// validation blocker: robot stuck at a table edge after the robot/table
// rescale, never recovering) ---
//
// Root cause: BackingAway/MovingForwardFromRearEdge/AdvancingInward all
// translated purely along a direction derived from the robot's CURRENT
// heading (reverse/forward respectively), without ever checking that
// this motion actually reduces the overhang that triggered recovery in
// the first place. "Front"/"rear" are robot-relative, not table-
// relative: an edge encountered at a shallow/lateral angle (heading
// nearly parallel to the edge, not perpendicular to it) can trigger a
// cliff whose overhang is on an axis the current heading barely moves
// along - blindly continuing can drive the robot toward/off a DIFFERENT
// edge, eventually reaching VirtualRobotHardware's full-off-table hard
// guard, which then rejects every further translation forever. The
// tests below drive a REAL recovery incident to completion (via
// driveRecoveryToCompletion() above, not hand-picked single-step
// scenarios) at exactly this shallow-angle geometry, for all four table
// edges plus a corner, and confirm the incident always resolves.

// RightEdgeRecoversInward
TEST(TableEdgeSafetyControllerTest, RightEdgeRecoversInward)
{
    TableEdgeSafetyController controller;
    // Shallow angle relative to the +X edge (heading 15 is nearly
    // parallel to it, not perpendicular) - the geometry class bugfix #3
    // addresses. Center already 0.05 past the boundary guarantees at
    // least one corner is off-table regardless of heading (see this
    // file's driveRecoveryToCompletion() docs).
    RobotPose pose{Vec3{6.05F, 0.125F, 1.5F}, 15.0F};
    ASSERT_TRUE(computeCliffSensorReadings(pose, kTable).anyCliff());

    ASSERT_TRUE(driveRecoveryToCompletion(controller, pose, kTable))
        << "recovery never resolved - permanently stuck at final pose x=" << pose.position.x
        << " z=" << pose.position.z << " heading=" << pose.headingDegrees
        << " state=" << static_cast<int>(controller.state());

    EXPECT_LE(pose.position.x, kTable.maxX);
    EXPECT_TRUE(areAllCornersSafelyInsideTable(pose, kTable, 0.0F));
}

// LeftEdgeRecoversInward
TEST(TableEdgeSafetyControllerTest, LeftEdgeRecoversInward)
{
    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{-6.05F, 0.125F, -1.2F}, 165.0F}; // mirror of the right-edge case
    ASSERT_TRUE(computeCliffSensorReadings(pose, kTable).anyCliff());

    ASSERT_TRUE(driveRecoveryToCompletion(controller, pose, kTable))
        << "recovery never resolved - permanently stuck at final pose x=" << pose.position.x
        << " z=" << pose.position.z << " heading=" << pose.headingDegrees
        << " state=" << static_cast<int>(controller.state());

    EXPECT_GE(pose.position.x, kTable.minX);
    EXPECT_TRUE(areAllCornersSafelyInsideTable(pose, kTable, 0.0F));
}

// FrontEdgeRecoversInward
TEST(TableEdgeSafetyControllerTest, FrontEdgeRecoversInward)
{
    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{1.3F, 0.125F, 6.05F}, 100.0F}; // shallow relative to the +Z edge
    ASSERT_TRUE(computeCliffSensorReadings(pose, kTable).anyCliff());

    ASSERT_TRUE(driveRecoveryToCompletion(controller, pose, kTable))
        << "recovery never resolved - permanently stuck at final pose x=" << pose.position.x
        << " z=" << pose.position.z << " heading=" << pose.headingDegrees
        << " state=" << static_cast<int>(controller.state());

    EXPECT_LE(pose.position.z, kTable.maxZ);
    EXPECT_TRUE(areAllCornersSafelyInsideTable(pose, kTable, 0.0F));
}

// RearEdgeRecoversInward
TEST(TableEdgeSafetyControllerTest, RearEdgeRecoversInward)
{
    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{-1.3F, 0.125F, -6.05F}, 280.0F}; // mirror of the front-edge case
    ASSERT_TRUE(computeCliffSensorReadings(pose, kTable).anyCliff());

    ASSERT_TRUE(driveRecoveryToCompletion(controller, pose, kTable))
        << "recovery never resolved - permanently stuck at final pose x=" << pose.position.x
        << " z=" << pose.position.z << " heading=" << pose.headingDegrees
        << " state=" << static_cast<int>(controller.state());

    EXPECT_GE(pose.position.z, kTable.minZ);
    EXPECT_TRUE(areAllCornersSafelyInsideTable(pose, kTable, 0.0F));
}

// CornerRecoversInwardWithSmallerRobotGeometry
//
// Corner regression required alongside the per-edge tests above,
// retested with the current (post-rescale, smaller) RobotDimensions -
// there is only one geometry in the codebase now (no separate old/new
// to switch between), so this exercises the same final footprint every
// other test in this file already uses.
TEST(TableEdgeSafetyControllerTest, CornerRecoversInwardWithSmallerRobotGeometry)
{
    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{6.05F, 0.125F, 6.05F}, 45.0F}; // +X/+Z corner, facing straight out
    ASSERT_TRUE(computeCliffSensorReadings(pose, kTable).anyCliff());

    ASSERT_TRUE(driveRecoveryToCompletion(controller, pose, kTable))
        << "recovery never resolved - permanently stuck at final pose x=" << pose.position.x
        << " z=" << pose.position.z << " heading=" << pose.headingDegrees
        << " state=" << static_cast<int>(controller.state());

    EXPECT_LE(pose.position.x, kTable.maxX);
    EXPECT_LE(pose.position.z, kTable.maxZ);
    EXPECT_TRUE(areAllCornersSafelyInsideTable(pose, kTable, 0.0F));
}

// HeadingOutwardCannotEnterAdvanceInward
//
// Turning must never hand off to AdvancingInward while the heading is
// still meaningfully outward-facing (large error against the recovery
// target), no matter how many updates pass - AdvancingInward drives
// FORWARD, and doing so at a bad heading is exactly the kind of blind
// translation bugfix #3 removes.
TEST(TableEdgeSafetyControllerTest, HeadingOutwardCannotEnterAdvanceInward)
{
    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{0.0F, 0.125F, 5.80F}, 0.0F};
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);

    pose.position.z = 5.75F;
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);

    // Target heading is 180 (straight back toward the table center).
    // Hold the heading near 20 - still ~160 degrees of error, well
    // outside kRecoveryHeadingToleranceDegrees - for many updates.
    pose.headingDegrees = 20.0F;
    for (int i = 0; i < 30; ++i)
    {
        controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
        ASSERT_NE(controller.state(), TableEdgeSafetyController::RecoveryState::AdvancingInward);
    }

    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);
    EXPECT_GT(std::fabs(controller.currentHeadingErrorDegrees()), TableEdgeSafetyController::kRecoveryHeadingToleranceDegrees);
}

// AdvancingInwardWithBadHeadingReturnsToTurning
//
// Once in AdvancingInward, if the heading drifts outside tolerance
// (e.g. a rejected translation, or accumulated drift), the controller
// must reorient rather than keep driving forward at a bad angle.
TEST(TableEdgeSafetyControllerTest, AdvancingInwardWithBadHeadingReturnsToTurning)
{
    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{0.0F, 0.125F, 5.80F}, 0.0F};
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);

    pose.position.z = 5.75F;
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);

    // Heading aligned with the 180-degree target, but RearLeft/RearRight
    // sit exactly at the Z 6.0 boundary (only the front corners were
    // ever pulled back during BackingAway) - not margin-safe yet, so
    // this hands off to AdvancingInward (matches
    // ReleasesOnlyAfterSensorsSafeAndHeadingAligned above).
    pose.headingDegrees = 180.0F;
    ASSERT_FALSE(computeCliffSensorReadings(pose, kTable).anyCliff());
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::AdvancingInward);

    // Heading drifts 30 degrees off target - outside the 10-degree
    // tolerance.
    pose.headingDegrees = 150.0F;
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);

    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);
}

// RecoveryMakesSupportMarginMonotonicallySaferWhereApplicable
//
// The whole point of bugfix #3's proposed-motion check: whenever the
// controller is in one of the three TRANSLATING states (BackingAway,
// MovingForwardFromRearEdge, AdvancingInward), a step must never make
// the aggregate table overhang WORSE - translatingWouldNotHelp() is
// exactly the guard that reroutes to Turning instead of letting a
// translation increase it. "Where applicable" deliberately excludes
// Turning: in-place ROTATION has no such guard (this fix is scoped to
// translation only, per the bugfix brief), and rotating a non-point
// footprint about its center can genuinely swing an already-clear
// corner back out over the edge even though the center itself never
// moves - confirmed empirically while writing this test (overhang rose
// from 0 across several consecutive steps immediately after a
// BackingAway->Turning handoff). That is expected, unavoidable
// geometry, not a regression - so this test checks monotonicity only
// across the translating states' own steps.
TEST(TableEdgeSafetyControllerTest, RecoveryMakesSupportMarginMonotonicallySaferWhereApplicable)
{
    using RecoveryState = TableEdgeSafetyController::RecoveryState;

    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{6.05F, 0.125F, 1.5F}, 15.0F};
    ASSERT_TRUE(computeCliffSensorReadings(pose, kTable).anyCliff());

    DifferentialDrive drive;
    constexpr float dt = 0.05F;
    bool resolved = false;
    for (int step = 0; step < 400; ++step)
    {
        const CliffSensorReadings readings = computeCliffSensorReadings(pose, kTable);
        controller.update(readings, pose, kTable);
        if (!controller.active())
        {
            resolved = true;
            break;
        }
        const RecoveryState stateThisStep = controller.state();
        const bool translating = stateThisStep == RecoveryState::BackingAway ||
                                  stateThisStep == RecoveryState::MovingForwardFromRearEdge ||
                                  stateThisStep == RecoveryState::AdvancingInward;
        const float overhangBeforeStep = aggregateTableOverhang(pose, kTable);

        const WheelSpeeds speeds = controller.recoveryWheelSpeeds();
        drive.setWheelSpeeds(speeds.left, speeds.right);
        drive.update(pose, dt);

        const float overhangAfterStep = aggregateTableOverhang(pose, kTable);
        if (translating)
        {
            EXPECT_LE(overhangAfterStep, overhangBeforeStep + 1.0e-4F)
                << "aggregate overhang increased during a translating step (" << static_cast<int>(stateThisStep)
                << ") at step " << step << ": " << overhangBeforeStep << " -> " << overhangAfterStep;
        }
    }

    ASSERT_TRUE(resolved);
    EXPECT_LE(aggregateTableOverhang(pose, kTable), 1.0e-4F);
}

// ============================================================
// Phase 13X blocker fix (deadlock repair) - bounded recovery-stall escape
// ============================================================
// Root cause this closes: translatingWouldNotHelp() (bugfix #3, above)
// only ever reasons about TABLE geometry - it cannot see that a
// translating state's commanded motion is being externally vetoed every
// frame by VirtualRobotHardware's independent obstacle-collision guard
// (e.g. a desk object sitting between the robot and this incident's fixed
// recovery target). Without a bound, Safety's own always-highest drive
// authority could hold a doomed translating state forever, starving
// AutonomousAvoidance/Navigation of the wheels indefinitely. See
// kMaxRecoveryStallFrames's own docs (TableEdgeSafetyController.hpp) for
// the full reasoning.

// --- StallEscape 1: FrozenPositionEventuallyReleasesAndReportsBlocked ---
TEST(TableEdgeSafetyControllerTest, FrozenPositionDuringAdvancingInwardEventuallyReleasesAndReportsBlocked)
{
    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{0.0F, 0.125F, 5.80F}, 0.0F};
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::BackingAway);

    pose.position.z = 5.75F;
    controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
    ASSERT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Turning);

    // Same setup as TurningTransitionsToAdvancingInwardWhenHeadingSafeButCornerStillEdge:
    // heading already at the target, but a sensor permanently reports an
    // edge, so Turning hands off to AdvancingInward - which this test then
    // holds at a COMPLETELY FIXED pose (simulating every proposed forward
    // step being rejected by an external obstacle-collision guard, never
    // integrated into `pose` here) for far longer than
    // kMaxRecoveryStallFrames.
    pose.headingDegrees = 180.0F;
    const CliffSensorReadings stuckReadings{false, false, true, false}; // rearLeft only

    bool everBlocked = false;
    int firedAtIteration = -1;
    // Stops the INSTANT the bound fires - readings still (deliberately,
    // per this test's own setup) report a persistent cliff, so a real
    // caller's next update() would legitimately re-trigger a brand new
    // incident (Inactive -> MovingForwardFromRearEdge) immediately
    // afterward; this test's own subject is the release ITSELF, not what
    // happens several calls later.
    for (int i = 0; i < TableEdgeSafetyController::kMaxRecoveryStallFrames + 20 && !everBlocked; ++i)
    {
        controller.update(stuckReadings, pose, kTable);
        if (controller.recoveryBlockedThisUpdate())
        {
            everBlocked = true;
            firedAtIteration = i;
        }
    }

    // Hard requirement: a translating state pinned at an unmoving pose
    // must eventually give up - never spin/hold Safety authority forever.
    ASSERT_TRUE(everBlocked);
    // Never fires before the bound is actually reached - the stall must
    // be PROVEN over the full window, never guessed early.
    EXPECT_GE(firedAtIteration, TableEdgeSafetyController::kMaxRecoveryStallFrames - 1);
    EXPECT_EQ(controller.state(), TableEdgeSafetyController::RecoveryState::Inactive);
    EXPECT_FALSE(controller.active());
}

// --- StallEscape 2: GenuineProgressNeverTriggersTheBound ---
TEST(TableEdgeSafetyControllerTest, GenuineTranslatingProgressNeverTriggersTheStallBound)
{
    // Regression guard for the existing, validated RightEdgeRecoversInward
    // -style closed-loop tests: a normally-recovering robot (position
    // genuinely advancing every step via real DifferentialDrive
    // integration) must never spuriously hit the new bounded-stall escape.
    TableEdgeSafetyController controller;
    RobotPose pose{Vec3{6.05F, 0.125F, 1.5F}, 15.0F};
    ASSERT_TRUE(computeCliffSensorReadings(pose, kTable).anyCliff());

    DifferentialDrive drive;
    constexpr float dt = 0.05F;
    bool everBlocked = false;
    bool resolved = false;
    for (int step = 0; step < 400; ++step)
    {
        controller.update(computeCliffSensorReadings(pose, kTable), pose, kTable);
        if (controller.recoveryBlockedThisUpdate())
        {
            everBlocked = true;
        }
        if (!controller.active())
        {
            resolved = true;
            break;
        }
        const WheelSpeeds speeds = controller.recoveryWheelSpeeds();
        drive.setWheelSpeeds(speeds.left, speeds.right);
        drive.update(pose, dt);
    }

    ASSERT_TRUE(resolved) << "recovery never resolved within the normal closed-loop budget";
    EXPECT_FALSE(everBlocked);
}
