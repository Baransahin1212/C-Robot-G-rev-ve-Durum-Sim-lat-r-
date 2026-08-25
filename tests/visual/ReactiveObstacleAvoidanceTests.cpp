#include <cmath>
#include <cstddef>
#include <iterator>

#include <gtest/gtest.h>

#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"

namespace
{

using robot::visual::AvoidanceState;
using robot::visual::DifferentialDrive;
using robot::visual::ObstacleHazardSample;
using robot::visual::ReactiveObstacleAvoidance;
using robot::visual::RobotPose;
using robot::visual::VirtualRobotHardware;
using robot::visual::WheelSpeeds;

RobotPose poseAt(float x, float z) noexcept
{
    RobotPose pose{};
    pose.position.x = x;
    pose.position.z = z;
    return pose;
}

// No side information at all - the deterministic-default case.
const ObstacleHazardSample kNoHazard{};

// Obstacle closer on the left than the right.
const ObstacleHazardSample kLeftCloserHazard{/*leftDistance=*/0.2F, /*centerDistance=*/std::nullopt,
                                              /*rightDistance=*/0.8F};

// Obstacle closer on the right than the left.
const ObstacleHazardSample kRightCloserHazard{/*leftDistance=*/0.8F, /*centerDistance=*/std::nullopt,
                                               /*rightDistance=*/0.2F};

// Symmetric hazard - equally close on both sides.
const ObstacleHazardSample kSymmetricHazard{/*leftDistance=*/0.5F, /*centerDistance=*/std::nullopt,
                                             /*rightDistance=*/0.5F};

} // namespace

// --- Phase 13V human-validation fix: TurnAway -> AdvanceClear -> Inactive
// incident lifecycle ---
//
// Replaces the Phase 13R two-phase latch (Inactive / active-with-a-fixed-
// turn-direction), which released the instant the forward body corridor
// read clear - a fact about the robot's CURRENT heading, not about whether
// it had physically translated past the obstacle. That let HomeNavigator
// (which re-aims fresh every frame from the current, unmoved, position)
// immediately re-target back toward the same obstacle, producing the
// human-GUI-observed zero-progress heading oscillation. See
// ReactiveObstacleAvoidance.hpp's own class docs and
// docs/technical-decisions.md for the full root-cause writeup.

// 1: StartsInactive
TEST(ReactiveObstacleAvoidanceTest, StartsInactive)
{
    // Arrange / Act
    ReactiveObstacleAvoidance avoidance;

    // Assert
    EXPECT_EQ(avoidance.state(), AvoidanceState::Inactive);
    EXPECT_FALSE(avoidance.active());
}

// 2: TriggerEntersTurnAway
TEST(ReactiveObstacleAvoidanceTest, TriggerEntersTurnAway)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;

    // Act
    avoidance.update(/*enabled=*/true, /*triggerAvoidance=*/true, /*forwardCorridorClear=*/false, poseAt(0.0F, 0.0F),
                      kNoHazard);

    // Assert
    EXPECT_EQ(avoidance.state(), AvoidanceState::TurnAway);
    EXPECT_TRUE(avoidance.active());
}

// 3: TurnDirectionLatchedForIncident
//
// Real sensor readings fluctuate frame to frame near an obstacle's edge;
// the chosen turn direction must NOT follow those fluctuations once an
// incident has begun - only the very first TurnAway-entering frame may
// choose it.
TEST(ReactiveObstacleAvoidanceTest, TurnDirectionLatchedForIncident)
{
    // Arrange: begin the incident with the obstacle closer on the left.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kLeftCloserHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::TurnAway);
    const WheelSpeeds initial = avoidance.wheelSpeeds();

    // Act: keep feeding fluctuating (even opposite) hazard readings while
    // still blocked - the incident is already in progress.
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kRightCloserHazard);
    const WheelSpeeds afterFlip = avoidance.wheelSpeeds();
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kSymmetricHazard);
    const WheelSpeeds afterSymmetric = avoidance.wheelSpeeds();

    // Assert: wheel speeds (and therefore the latched direction) never
    // changed.
    EXPECT_FLOAT_EQ(initial.left, afterFlip.left);
    EXPECT_FLOAT_EQ(initial.right, afterFlip.right);
    EXPECT_FLOAT_EQ(initial.left, afterSymmetric.left);
    EXPECT_FLOAT_EQ(initial.right, afterSymmetric.right);
}

// 4: LeftHazardChoosesRightTurn
TEST(ReactiveObstacleAvoidanceTest, LeftHazardChoosesRightTurn)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;

    // Act: obstacle closer on the left.
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kLeftCloserHazard);
    const WheelSpeeds speeds = avoidance.wheelSpeeds();

    // Assert: turning right - matches this project's own convention proof
    // (heading increases under DifferentialDrive with left<0, right>0).
    EXPECT_LT(speeds.left, 0.0F);
    EXPECT_GT(speeds.right, 0.0F);
    DifferentialDrive drive;
    drive.setWheelSpeeds(speeds.left, speeds.right);
    RobotPose pose{};
    drive.update(pose, 0.1F);
    EXPECT_GT(pose.headingDegrees, 0.0F);
}

// 5: RightHazardChoosesLeftTurn
TEST(ReactiveObstacleAvoidanceTest, RightHazardChoosesLeftTurn)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;

    // Act: obstacle closer on the right.
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kRightCloserHazard);
    const WheelSpeeds speeds = avoidance.wheelSpeeds();

    // Assert: turning left - the mirror image of test 4.
    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_LT(speeds.right, 0.0F);
    // headingDegrees is normalized into [0, 360) by DifferentialDrive, so a
    // small negative turn wraps to just below 360, not below zero.
    DifferentialDrive drive;
    drive.setWheelSpeeds(speeds.left, speeds.right);
    RobotPose pose{};
    drive.update(pose, 0.1F);
    EXPECT_GT(pose.headingDegrees, 300.0F);
}

// 6: SymmetricHazardUsesDeterministicDefault
TEST(ReactiveObstacleAvoidanceTest, SymmetricHazardUsesDeterministicDefault)
{
    // Arrange
    ReactiveObstacleAvoidance symmetric;
    ReactiveObstacleAvoidance noInfo;

    // Act: equally-close-both-sides, and no side information at all.
    symmetric.update(true, true, false, poseAt(0.0F, 0.0F), kSymmetricHazard);
    noInfo.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);

    // Assert: both fall back to the same deterministic default direction
    // (preserving Phase 13R's original always-turn-this-way behavior for
    // the case where there is no reason to prefer a side).
    const WheelSpeeds symmetricSpeeds = symmetric.wheelSpeeds();
    const WheelSpeeds noInfoSpeeds = noInfo.wheelSpeeds();
    EXPECT_FLOAT_EQ(symmetricSpeeds.left, noInfoSpeeds.left);
    EXPECT_FLOAT_EQ(symmetricSpeeds.right, noInfoSpeeds.right);
    EXPECT_LT(symmetricSpeeds.left, 0.0F);
    EXPECT_GT(symmetricSpeeds.right, 0.0F);
}

// 7: SensorClearDoesNotImmediatelyRelease
//
// The Phase 13R regression this whole redesign fixes: the instant the
// forward corridor reads clear must NOT drop the incident straight back to
// Inactive.
TEST(ReactiveObstacleAvoidanceTest, SensorClearDoesNotImmediatelyRelease)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);
    ASSERT_TRUE(avoidance.active());

    // Act: forwardCorridorClear becomes true.
    avoidance.update(true, false, /*forwardCorridorClear=*/true, poseAt(0.0F, 0.0F), kNoHazard);

    // Assert: still an active incident - not Inactive.
    EXPECT_TRUE(avoidance.active());
    EXPECT_NE(avoidance.state(), AvoidanceState::Inactive);
}

// 8: ClearTransitionsToAdvanceClear
TEST(ReactiveObstacleAvoidanceTest, ClearTransitionsToAdvanceClear)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::TurnAway);

    // Act
    avoidance.update(true, false, true, poseAt(0.0F, 0.0F), kNoHazard);

    // Assert
    EXPECT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);
}

// 9: AdvanceClearCommandsTranslation
TEST(ReactiveObstacleAvoidanceTest, AdvanceClearCommandsTranslation)
{
    // Arrange: drive into AdvanceClear.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);
    avoidance.update(true, false, true, poseAt(0.0F, 0.0F), kNoHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);

    // Act
    const WheelSpeeds speeds = avoidance.wheelSpeeds();

    // Assert: equal, positive wheel speeds - real forward translation
    // (v > 0), not a turn (omega == 0), fed through the real
    // DifferentialDrive to prove the position actually advances.
    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
    DifferentialDrive drive;
    drive.setWheelSpeeds(speeds.left, speeds.right);
    RobotPose pose{};
    drive.update(pose, 0.5F);
    EXPECT_GT(pose.position.z, 0.0F);
    EXPECT_NEAR(pose.headingDegrees, 0.0F, 0.001F);
}

// 10: InsufficientAdvanceDistanceDoesNotRelease
TEST(ReactiveObstacleAvoidanceTest, InsufficientAdvanceDistanceDoesNotRelease)
{
    // Arrange: AdvanceClear begins at the origin.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);
    avoidance.update(true, false, true, poseAt(0.0F, 0.0F), kNoHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);

    // Act: the robot has only travelled a small fraction of the required
    // bypass distance, corridor still clear.
    const float shortDistance = ReactiveObstacleAvoidance::kMinimumBypassDistanceWorldUnits * 0.25F;
    avoidance.update(true, false, true, poseAt(0.0F, shortDistance), kNoHazard);

    // Assert: still AdvanceClear - not released yet.
    EXPECT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);
    EXPECT_TRUE(avoidance.active());
}

// 11: RequiredAdvanceDistanceAndClearCorridorReleases
TEST(ReactiveObstacleAvoidanceTest, RequiredAdvanceDistanceAndClearCorridorReleases)
{
    // Arrange: AdvanceClear begins at the origin.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);
    avoidance.update(true, false, true, poseAt(0.0F, 0.0F), kNoHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);

    // Act: the robot has now travelled at least the required bypass
    // distance, corridor still clear. Computed locally (not a namespace-
    // scope global) - kMinimumBypassDistanceWorldUnits is itself a
    // runtime-initialized cross-translation-unit const, so reading it
    // during another TU's own static-initialization phase would risk the
    // classic initialization-order fiasco; by the time a TEST body runs,
    // all global initializers have already completed.
    const float farAdvance = ReactiveObstacleAvoidance::kMinimumBypassDistanceWorldUnits + 0.5F;
    avoidance.update(true, false, true, poseAt(0.0F, farAdvance), kNoHazard);

    // Assert: released.
    EXPECT_EQ(avoidance.state(), AvoidanceState::Inactive);
    EXPECT_FALSE(avoidance.active());
}

// 12: ReblockedDuringAdvanceReturnsToTurnAway
TEST(ReactiveObstacleAvoidanceTest, ReblockedDuringAdvanceReturnsToTurnAway)
{
    // Arrange: AdvanceClear, partway through.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);
    avoidance.update(true, false, true, poseAt(0.0F, 0.0F), kNoHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);

    // Act: the corridor is blocked again (a second, closer obstacle edge
    // encountered mid-advance).
    avoidance.update(true, false, /*forwardCorridorClear=*/false, poseAt(0.0F, 0.1F), kNoHazard);

    // Assert: back to TurnAway, still an active incident.
    EXPECT_EQ(avoidance.state(), AvoidanceState::TurnAway);
    EXPECT_TRUE(avoidance.active());
}

// 13: ReblockedIncidentDoesNotFlipDirectionRepeatedly
TEST(ReactiveObstacleAvoidanceTest, ReblockedIncidentDoesNotFlipDirectionRepeatedly)
{
    // Arrange: begin the incident with the obstacle closer on the left
    // (turn right), reach AdvanceClear.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kLeftCloserHazard);
    const WheelSpeeds originalTurn = avoidance.wheelSpeeds();
    avoidance.update(true, false, true, poseAt(0.0F, 0.0F), kLeftCloserHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);

    // Act: re-blocked mid-advance, this time with hazard readings
    // suggesting the OPPOSITE side is now closer.
    avoidance.update(true, false, false, poseAt(0.0F, 0.1F), kRightCloserHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::TurnAway);
    const WheelSpeeds afterReblock = avoidance.wheelSpeeds();

    // Assert: the ORIGINAL turn direction is preserved, not recomputed
    // from the new (opposite) hazard reading - this is the core anti-
    // oscillation guarantee for a single incident.
    EXPECT_FLOAT_EQ(originalTurn.left, afterReblock.left);
    EXPECT_FLOAT_EQ(originalTurn.right, afterReblock.right);
}

// 14: DisableClearsState
TEST(ReactiveObstacleAvoidanceTest, DisableClearsState)
{
    // Arrange: deep into an incident (AdvanceClear).
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);
    avoidance.update(true, false, true, poseAt(0.0F, 0.0F), kNoHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);

    // Act: `A` toggled off.
    avoidance.update(/*enabled=*/false, true, false, poseAt(0.0F, 0.2F), kNoHazard);

    // Assert
    EXPECT_EQ(avoidance.state(), AvoidanceState::Inactive);
    EXPECT_FALSE(avoidance.active());
}

// 15: NewIncidentCanChooseFreshDirection
TEST(ReactiveObstacleAvoidanceTest, NewIncidentCanChooseFreshDirection)
{
    // Arrange: run a first incident to completion, latched toward the
    // right-turn direction (obstacle closer on the left). Computed
    // locally, not as a namespace-scope global - see the comment on the
    // equivalent local in RequiredAdvanceDistanceAndClearCorridorReleases
    // above for why.
    const float farAdvance = ReactiveObstacleAvoidance::kMinimumBypassDistanceWorldUnits + 0.5F;
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kLeftCloserHazard);
    const WheelSpeeds firstIncidentTurn = avoidance.wheelSpeeds();
    avoidance.update(true, false, true, poseAt(0.0F, 0.0F), kLeftCloserHazard);
    avoidance.update(true, false, true, poseAt(0.0F, farAdvance), kLeftCloserHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::Inactive);

    // Act: a NEW incident begins, this time with the obstacle closer on
    // the right.
    avoidance.update(true, true, false, poseAt(0.0F, farAdvance), kRightCloserHazard);
    const WheelSpeeds secondIncidentTurn = avoidance.wheelSpeeds();

    // Assert: the new incident chose the opposite (mirrored) direction -
    // proof this is a fresh per-incident choice, not a permanently frozen
    // one.
    EXPECT_FLOAT_EQ(secondIncidentTurn.left, -firstIncidentTurn.left);
    EXPECT_FLOAT_EQ(secondIncidentTurn.right, -firstIncidentTurn.right);
}

// --- Preserved cross-cutting properties from earlier phases ---

// 16: MagnitudesStayWithinForwardSpeedEnvelope
TEST(ReactiveObstacleAvoidanceTest, MagnitudesStayWithinForwardSpeedEnvelope)
{
    // Arrange
    ReactiveObstacleAvoidance turning;
    turning.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);
    ReactiveObstacleAvoidance advancing;
    advancing.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);
    advancing.update(true, false, true, poseAt(0.0F, 0.0F), kNoHazard);
    ASSERT_EQ(advancing.state(), AvoidanceState::AdvanceClear);

    // Act
    const WheelSpeeds turnSpeeds = turning.wheelSpeeds();
    const WheelSpeeds advanceSpeeds = advancing.wheelSpeeds();

    // Assert: neither phase commands an unreasonably fast maneuver.
    EXPECT_LE(std::fabs(turnSpeeds.left), VirtualRobotHardware::kForwardWheelSpeed);
    EXPECT_LE(std::fabs(turnSpeeds.right), VirtualRobotHardware::kForwardWheelSpeed);
    EXPECT_LE(std::fabs(advanceSpeeds.left), VirtualRobotHardware::kForwardWheelSpeed);
    EXPECT_LE(std::fabs(advanceSpeeds.right), VirtualRobotHardware::kForwardWheelSpeed);
}

// 17: InactiveReturnsZeroWheelSpeeds
TEST(ReactiveObstacleAvoidanceTest, InactiveReturnsZeroWheelSpeeds)
{
    // Arrange / Act
    ReactiveObstacleAvoidance avoidance;
    const WheelSpeeds speeds = avoidance.wheelSpeeds();

    // Assert
    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 18: ReEnableDoesNotActivateWithoutTrigger
TEST(ReactiveObstacleAvoidanceTest, ReEnableDoesNotActivateWithoutTrigger)
{
    // Arrange: was active, then disabled.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAt(0.0F, 0.0F), kNoHazard);
    avoidance.update(false, false, false, poseAt(0.0F, 0.0F), kNoHazard);
    ASSERT_FALSE(avoidance.active());

    // Act: re-enabled, but no trigger this frame.
    avoidance.update(/*enabled=*/true, /*triggerAvoidance=*/false, false, poseAt(0.0F, 0.0F), kNoHazard);

    // Assert
    EXPECT_FALSE(avoidance.active());
}

// --- Phase 13X blocker fix: bounded TurnAway sweep / LocalRouteBlocked ---
//
// TurnAway rotates in one latched direction; accumulated ABSOLUTE heading
// rotation for the current incident is the sum of |headingDelta| across
// consecutive update() calls (the very first call after entering TurnAway
// only SEEDS the previous heading, per the class' own docs, so it never
// contributes a delta itself). Driving heading in fixed 90-degree steps
// therefore reaches exactly kMaximumTurnAwaySweepDegrees (360) after four
// 90-degree deltas - a clean, exact way to land precisely on the sweep
// limit without depending on any wall-clock or frame-count notion.
namespace
{
RobotPose poseAtHeading(float headingDegrees) noexcept
{
    RobotPose pose{};
    pose.headingDegrees = headingDegrees;
    return pose;
}
} // namespace

// 19: TurnAwayFindsClearDirectionBeforeSweepLimit
TEST(ReactiveObstacleAvoidanceTest, TurnAwayFindsClearDirectionBeforeSweepLimit)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, /*triggerAvoidance=*/true, /*forwardCorridorClear=*/false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(0.0F), kNoHazard);   // seed
    avoidance.update(true, false, false, poseAtHeading(90.0F), kNoHazard); // 90 accumulated

    // Act: corridor clears well before the 360-degree sweep limit.
    avoidance.update(true, false, /*forwardCorridorClear=*/true, poseAtHeading(180.0F), kNoHazard);

    // Assert
    EXPECT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);
    EXPECT_FALSE(avoidance.localRouteBlockedThisUpdate());
}

// 20: TurnAwayCannotRotateForever
TEST(ReactiveObstacleAvoidanceTest, TurnAwayCannotRotateForever)
{
    // Arrange: a hazard that never clears at any sampled heading.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(0.0F), kNoHazard);    // seed, 0
    avoidance.update(true, false, false, poseAtHeading(90.0F), kNoHazard);   // 90
    avoidance.update(true, false, false, poseAtHeading(180.0F), kNoHazard); // 180
    avoidance.update(true, false, false, poseAtHeading(270.0F), kNoHazard); // 270

    // Act: the fourth 90-degree step reaches exactly the 360-degree bound.
    avoidance.update(true, false, false, poseAtHeading(360.0F), kNoHazard);

    // Assert: released, never left spinning indefinitely.
    EXPECT_EQ(avoidance.state(), AvoidanceState::Inactive);
    EXPECT_FALSE(avoidance.active());
}

// 21: FullSweepWithoutClearanceReportsLocalRouteBlocked
TEST(ReactiveObstacleAvoidanceTest, FullSweepWithoutClearanceReportsLocalRouteBlocked)
{
    // Arrange
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(90.0F), kNoHazard);
    EXPECT_FALSE(avoidance.localRouteBlockedThisUpdate());
    avoidance.update(true, false, false, poseAtHeading(180.0F), kNoHazard);
    EXPECT_FALSE(avoidance.localRouteBlockedThisUpdate());
    avoidance.update(true, false, false, poseAtHeading(270.0F), kNoHazard);
    EXPECT_FALSE(avoidance.localRouteBlockedThisUpdate());

    // Act: exactly the call that exhausts the sweep.
    avoidance.update(true, false, false, poseAtHeading(360.0F), kNoHazard);

    // Assert: true for precisely this one call - a one-frame edge signal.
    EXPECT_TRUE(avoidance.localRouteBlockedThisUpdate());

    // Assert: the very next call (even if still blocked) does not repeat it,
    // since the incident already released and a fresh one has not begun.
    avoidance.update(true, false, false, poseAtHeading(360.0F), kNoHazard);
    EXPECT_FALSE(avoidance.localRouteBlockedThisUpdate());
}

// 22: LocalRouteBlockedDoesNotCommandForward
TEST(ReactiveObstacleAvoidanceTest, LocalRouteBlockedDoesNotCommandForward)
{
    // Arrange: drive the sweep to exhaustion.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(90.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(180.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(270.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(360.0F), kNoHazard);
    ASSERT_TRUE(avoidance.localRouteBlockedThisUpdate());
    ASSERT_EQ(avoidance.state(), AvoidanceState::Inactive);

    // Act
    const WheelSpeeds speeds = avoidance.wheelSpeeds();

    // Assert: released to Inactive means zero commanded wheel speed -
    // it is the caller's (main3d.cpp / WaypointNavigator) job to drive,
    // never this class' once it has given up on this incident.
    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 23: NewIncidentResetsAccumulatedSweep
TEST(ReactiveObstacleAvoidanceTest, NewIncidentResetsAccumulatedSweep)
{
    // Arrange: exhaust one incident's full sweep.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(90.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(180.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(270.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(360.0F), kNoHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::Inactive);

    // Act: a brand new incident begins. If the accumulator had NOT been
    // reset, even a single small rotation step would immediately exceed
    // the (already-at-360) bound and falsely report blocked again.
    avoidance.update(true, /*triggerAvoidance=*/true, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, false, poseAtHeading(45.0F), kNoHazard);

    // Assert: only 45 degrees into the new incident - nowhere near blocked.
    EXPECT_EQ(avoidance.state(), AvoidanceState::TurnAway);
    EXPECT_FALSE(avoidance.localRouteBlockedThisUpdate());
}

// 24: SuccessfulIncidentStillUsesAdvanceClear
TEST(ReactiveObstacleAvoidanceTest, SuccessfulIncidentStillUsesAdvanceClear)
{
    // Arrange / Act: a normal single-obstacle bypass, well under the sweep
    // bound, must still behave exactly like the pre-Phase-13X-blocker-fix
    // TurnAway -> AdvanceClear -> Inactive lifecycle.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAtHeading(0.0F), kNoHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::TurnAway);
    avoidance.update(true, false, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, /*forwardCorridorClear=*/true, poseAtHeading(30.0F), kNoHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);
    EXPECT_FALSE(avoidance.localRouteBlockedThisUpdate());

    RobotPose advancing{};
    advancing.headingDegrees = 30.0F;
    advancing.position.x = ReactiveObstacleAvoidance::kMinimumBypassDistanceWorldUnits + 0.1F;
    avoidance.update(true, false, true, advancing, kNoHazard);

    // Assert: released normally, never via the blocked path.
    EXPECT_EQ(avoidance.state(), AvoidanceState::Inactive);
    EXPECT_FALSE(avoidance.localRouteBlockedThisUpdate());
}

// 25: BypassDistanceBehaviorUnchanged
TEST(ReactiveObstacleAvoidanceTest, BypassDistanceBehaviorUnchanged)
{
    // Arrange: reach AdvanceClear.
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(true, true, false, poseAtHeading(0.0F), kNoHazard);
    avoidance.update(true, false, true, poseAtHeading(0.0F), kNoHazard);
    ASSERT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);

    // Act: travel less than the required bypass distance.
    RobotPose shortOfBypass{};
    shortOfBypass.position.x = ReactiveObstacleAvoidance::kMinimumBypassDistanceWorldUnits - 0.05F;
    avoidance.update(true, false, true, shortOfBypass, kNoHazard);

    // Assert: still AdvanceClear - the bounded-sweep fix must not have
    // changed AdvanceClear's own, separate, pre-existing distance gate.
    EXPECT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);
    EXPECT_FALSE(avoidance.localRouteBlockedThisUpdate());
}

// 26: DeterministicSameInputSameOutcome
TEST(ReactiveObstacleAvoidanceTest, DeterministicSameInputSameOutcome)
{
    // Arrange: two independent instances driven with the identical input
    // sequence, including one that exhausts the sweep.
    ReactiveObstacleAvoidance first;
    ReactiveObstacleAvoidance second;
    const float headings[] = {0.0F, 0.0F, 90.0F, 180.0F, 270.0F, 360.0F};
    const bool triggers[] = {true, false, false, false, false, false};

    for (std::size_t i = 0; i < std::size(headings); ++i)
    {
        first.update(true, triggers[i], false, poseAtHeading(headings[i]), kLeftCloserHazard);
        second.update(true, triggers[i], false, poseAtHeading(headings[i]), kLeftCloserHazard);

        // Assert: identical state and blocked-signal at every single step.
        ASSERT_EQ(first.state(), second.state());
        ASSERT_EQ(first.localRouteBlockedThisUpdate(), second.localRouteBlockedThisUpdate());
        ASSERT_EQ(first.wheelSpeeds().left, second.wheelSpeeds().left);
        ASSERT_EQ(first.wheelSpeeds().right, second.wheelSpeeds().right);
    }
}
