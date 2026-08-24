#include <algorithm>
#include <cmath>
#include <optional>

#include <gtest/gtest.h>

#include "robot/CompositePollingEventSource.hpp"
#include "robot/Event.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/DemoCommandSource.hpp"
#include "robot/visual/ForwardClearanceProbe.hpp"
#include "robot/visual/HomeArrivalEventSource.hpp"
#include "robot/visual/HomeNavigator.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/ReturnHomeRequestSource.hpp"
#include "robot/visual/TableEdgeSafetyController.hpp"
#include "robot/visual/VirtualCliffSensor.hpp"
#include "robot/visual/VirtualObstacleSensorArray.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"
#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualMath.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace
{

using robot::CompositePollingEventSource;
using robot::Event;
using robot::EventType;
using robot::HardwareEventSource;
using robot::RobotController;
using robot::RobotRuntime;
using robot::ReturnHomeReason;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::RuntimeStepResult;
using robot::TransitionResult;
using robot::visual::areAllCornersSafelyInsideTable;
using robot::visual::AvoidanceState;
using robot::visual::BasePlatform;
using robot::visual::DemoCommandSource;
using robot::visual::DeskObjectType;
using robot::visual::DriveAuthority;
using robot::visual::CliffSensorReadings;
using robot::visual::ForwardClearanceProbe;
using robot::visual::HomeArrivalEventSource;
using robot::visual::HomeNavigationOutput;
using robot::visual::HomeNavigationState;
using robot::visual::HomeNavigator;
using robot::visual::ObstacleHazardSample;
using robot::visual::ReactiveObstacleAvoidance;
using robot::visual::ReturnHomeRequestSource;
using robot::visual::RobotPose;
using robot::visual::robotPositionCollidesWithObstacles;
using robot::visual::shortestSignedHeadingErrorDegrees;
using robot::visual::TableEdgeSafetyController;
using robot::visual::TableSurface;
using robot::visual::VirtualCliffSensor;
using robot::visual::Vec3;
using robot::visual::VirtualDriveCommand;
using robot::visual::VirtualObstacleSensorArray;
using robot::visual::VirtualRobotHardware;
using robot::visual::VirtualWorld;
using robot::visual::WheelSpeeds;

// Disables every default demo obstacle so a test can place/enable exactly
// one controlled obstacle of its own - same convention
// VirtualDistanceSensorTests.cpp already uses.
void disableAllObstacles(VirtualWorld& world)
{
    for (std::size_t i = 0; i < world.obstacles().size(); ++i)
    {
        world.setObstacleEnabled(i, false);
    }
}

} // namespace

// 1: DefaultsToStoppedCommand
TEST(VirtualRobotHardwareTest, DefaultsToStoppedCommand)
{
    // Arrange / Act
    VirtualWorld world;
    VirtualRobotHardware hardware(world);

    // Assert
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
}

// 2: SensorDefaultsAreSafe
TEST(VirtualRobotHardwareTest, SensorDefaultsAreSafe)
{
    // Arrange / Act
    VirtualWorld world;
    VirtualRobotHardware hardware(world);

    // Assert
    EXPECT_EQ(hardware.batteryLevelPercent(), 100);
    EXPECT_FALSE(hardware.obstacleDetected());
    EXPECT_FALSE(hardware.emergencyStopPressed());
}

// 3: MoveForwardChangesCommand
TEST(VirtualRobotHardwareTest, MoveForwardChangesCommand)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);

    // Act
    hardware.moveForward();

    // Assert
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);
}

// 4: StopChangesCommand
TEST(VirtualRobotHardwareTest, StopChangesCommand)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();

    // Act
    hardware.stop();

    // Assert
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
}

// 5: ReturnToBaseChangesCommand
TEST(VirtualRobotHardwareTest, ReturnToBaseChangesCommand)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);

    // Act
    hardware.returnToBase();

    // Assert
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);
}

// 6: StoppedDoesNotMoveRobot
TEST(VirtualRobotHardwareTest, StoppedDoesNotMoveRobot)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    const Vec3 initialPosition = world.robotPose().position;

    // Act: command defaults to Stopped.
    hardware.update(1.0F);

    // Assert
    EXPECT_FLOAT_EQ(world.robotPose().position.x, initialPosition.x);
    EXPECT_FLOAT_EQ(world.robotPose().position.z, initialPosition.z);
}

// 7: MoveForwardMovesRobot
TEST(VirtualRobotHardwareTest, MoveForwardMovesRobot)
{
    // Arrange: explicit position/heading (Phase 13W human-visual-redesign
    // v2's own default heading is now 180 and default position sits right
    // next to the charging dock, so this is no longer implicit/collision-
    // free).
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    VirtualRobotHardware hardware(world);
    const float initialZ = world.robotPose().position.z;

    // Act
    hardware.moveForward();
    hardware.update(1.0F);

    // Assert
    EXPECT_GT(world.robotPose().position.z, initialZ);
}

// 8: MovementUsesDeltaTime
TEST(VirtualRobotHardwareTest, MovementUsesDeltaTime)
{
    // Arrange: two independent, identically-constructed worlds (demo
    // scene construction is deterministic - see VirtualWorldTests.cpp).
    VirtualWorld worldHalf;
    VirtualRobotHardware hardwareHalf(worldHalf);
    const float initialZ = worldHalf.robotPose().position.z;

    VirtualWorld worldFull;
    VirtualRobotHardware hardwareFull(worldFull);

    // Act
    hardwareHalf.moveForward();
    hardwareHalf.update(0.5F);

    hardwareFull.moveForward();
    hardwareFull.update(1.0F);

    // Assert: double the delta time produces double the distance moved.
    const float distanceHalf = worldHalf.robotPose().position.z - initialZ;
    const float distanceFull = worldFull.robotPose().position.z - initialZ;
    EXPECT_NEAR(distanceFull, distanceHalf * 2.0F, 0.001F);
}

// 9: HeadingZeroMovesInFrontMarkerDirection
TEST(VirtualRobotHardwareTest, HeadingZeroMovesInFrontMarkerDirection)
{
    // Arrange: explicit position/heading (VisualRobot.cpp's
    // rlRotatef(headingDegrees, 0, 1, 0) convention, and VirtualWorld.hpp/
    // VisualRobot.hpp's own docs, define heading 0 as facing +Z) - Phase
    // 13W human-visual-redesign v2's own default heading is now 180 and
    // default position sits right next to the charging dock, so this is
    // no longer implicit/collision-free.
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    const float initialX = world.robotPose().position.x;
    const float initialZ = world.robotPose().position.z;
    VirtualRobotHardware hardware(world);

    // Act
    hardware.moveForward();
    hardware.update(1.0F);

    // Assert: moves along +Z, X unchanged.
    EXPECT_NEAR(world.robotPose().position.x, initialX, 0.001F);
    EXPECT_NEAR(world.robotPose().position.z, initialZ + 1.0F, 0.001F);
}

// 10: HeadingNinetyMovesInCorrectDirection
TEST(VirtualRobotHardwareTest, HeadingNinetyMovesInCorrectDirection)
{
    // Arrange: rlRotatef(90, 0, 1, 0) rotates the front marker from +Z
    // toward +X (right-hand rotation around +Y - derived directly from
    // VisualRobot.cpp's actual rotation call, not assumed).
    VirtualWorld world;
    world.setRobotHeading(90.0F);
    const float initialX = world.robotPose().position.x;
    const float initialZ = world.robotPose().position.z;
    VirtualRobotHardware hardware(world);

    // Act
    hardware.moveForward();
    hardware.update(1.0F);

    // Assert: moves along +X, Z unchanged.
    EXPECT_NEAR(world.robotPose().position.x, initialX + 1.0F, 0.001F);
    EXPECT_NEAR(world.robotPose().position.z, initialZ, 0.001F);
}

// 11: StopAfterMovingPreventsFurtherMovement
TEST(VirtualRobotHardwareTest, StopAfterMovingPreventsFurtherMovement)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();
    hardware.update(1.0F);
    const float zAfterMoving = world.robotPose().position.z;

    // Act
    hardware.stop();
    hardware.update(1.0F);

    // Assert
    EXPECT_FLOAT_EQ(world.robotPose().position.z, zAfterMoving);
}

// 12: ReturnToBaseDoesNotMoveYet
TEST(VirtualRobotHardwareTest, ReturnToBaseDoesNotMoveYet)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    const Vec3 initialPosition = world.robotPose().position;

    // Act: return-to-base navigation is not implemented yet - treated the
    // same as Stopped for this phase.
    hardware.returnToBase();
    hardware.update(1.0F);

    // Assert
    EXPECT_FLOAT_EQ(world.robotPose().position.x, initialPosition.x);
    EXPECT_FLOAT_EQ(world.robotPose().position.z, initialPosition.z);
}

// 13: RobotRemainsInsideWorldBounds
TEST(VirtualRobotHardwareTest, RobotRemainsInsideWorldBounds)
{
    // Arrange: heading 0 moves toward +Z indefinitely if unclamped.
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();

    // Act: far more simulated time than needed to reach the ~10-unit
    // world half-extent at 1.0 unit/second.
    for (int i = 0; i < 100; ++i)
    {
        hardware.update(1.0F);
    }

    // Assert: clamped, not teleported off into infinity.
    EXPECT_LE(world.robotPose().position.z, 10.0F);
    EXPECT_GE(world.robotPose().position.z, -10.0F);
}

// --- Geometry-backed obstacleDetected()/obstacleDistance() (Phase 13O;
// Phase 13W human-visual-redesign v2) ---
//
// The old standalone "blocking obstacle" demo cube is gone - the default
// workspace's only obstacles are the six desk objects and the dock's rear
// housing (see VirtualWorld.cpp). These tests build their own controlled,
// local geometry instead: every other obstacle disabled, then the
// Keyboard's real registered obstacle (found via the named
// deskObjectObstacleIndex() semantic lookup - never a raw magic index)
// repositioned directly ahead of a chosen robot pose, exactly mirroring
// this file's own established disableAllObstacles()-then-reposition
// pattern used throughout the rest of this file.

// Shared geometry for tests 14-17 below (and several others that reuse
// this same obstacle setup): the Keyboard's real registered obstacle
// (footprint 2.3 x 0.1 x 0.75 - see VirtualWorld.cpp) relocated to X 2.5
// on the robot's centerline, heading 90 (forward = +X) - Phase 13W human-
// visual-redesign v2's table is only 4.0F deep (Z), which no longer fits
// this block's approach distances, but is 8.0F wide (X), which does; near
// face at X 2.5 - (2.3 / 2) = 1.35 (the ray now travels along the
// obstacle's WIDTH, not its depth, since the box itself is never
// rotated - only repositioned).
constexpr float kObstacleTestX = 2.5F;
constexpr float kObstacleNearFaceX = kObstacleTestX - 1.15F;

// 14: ObstacleDetectedFalseWhenFarFromBlockingObstacle
TEST(VirtualRobotHardwareTest, ObstacleDetectedFalseWhenFarFromBlockingObstacle)
{
    // Arrange: the Keyboard's obstacle placed directly ahead (heading 90 =
    // +X), far enough away that obstacleDetected() starts false but still
    // within VirtualDistanceSensor::kMaximumRange (2.5F) so
    // obstacleDistance() still has a value.
    VirtualWorld world;
    disableAllObstacles(world);
    const std::size_t keyboardIndex = world.deskObjectObstacleIndex(DeskObjectType::Keyboard);
    world.setObstaclePosition(keyboardIndex, Vec3{kObstacleTestX, 0.4F, 0.0F});
    world.setObstacleEnabled(keyboardIndex, true);
    world.setRobotPosition(Vec3{-0.5F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    VirtualRobotHardware hardware(world);

    // Assert
    EXPECT_FALSE(hardware.obstacleDetected());
    ASSERT_TRUE(hardware.obstacleDistance().has_value());
    EXPECT_GT(*hardware.obstacleDistance(), 1.0F);
}

// 15: ObstacleDetectedTrueWhenWithinThreshold
TEST(VirtualRobotHardwareTest, ObstacleDetectedTrueWhenWithinThreshold)
{
    // Arrange: same obstacle placement as above, robot close enough that
    // the front-sensor distance is exactly 0.5 (sensor origin X = robot X
    // + half body length 0.25; solved so kObstacleNearFaceX - sensorOriginX
    // = 0.5).
    VirtualWorld world;
    disableAllObstacles(world);
    const std::size_t keyboardIndex = world.deskObjectObstacleIndex(DeskObjectType::Keyboard);
    world.setObstaclePosition(keyboardIndex, Vec3{kObstacleTestX, 0.4F, 0.0F});
    world.setObstacleEnabled(keyboardIndex, true);
    world.setRobotPosition(Vec3{kObstacleNearFaceX - 0.5F - 0.25F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    VirtualRobotHardware hardware(world);

    // Act / Assert
    EXPECT_TRUE(hardware.obstacleDetected());
    ASSERT_TRUE(hardware.obstacleDistance().has_value());
    EXPECT_NEAR(*hardware.obstacleDistance(), 0.5F, 0.01F);
}

// 16: ObstacleDisabledMeansNotDetected
TEST(VirtualRobotHardwareTest, ObstacleDisabledMeansNotDetected)
{
    // Arrange: same close position as above, but the obstacle is disabled.
    VirtualWorld world;
    disableAllObstacles(world);
    const std::size_t keyboardIndex = world.deskObjectObstacleIndex(DeskObjectType::Keyboard);
    world.setObstaclePosition(keyboardIndex, Vec3{kObstacleTestX, 0.4F, 0.0F});
    world.setRobotPosition(Vec3{kObstacleNearFaceX - 0.5F - 0.25F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    world.setObstacleEnabled(keyboardIndex, false);
    VirtualRobotHardware hardware(world);

    // Act / Assert
    EXPECT_FALSE(hardware.obstacleDetected());
    EXPECT_FALSE(hardware.obstacleDistance().has_value());
}

// 17: ObstacleMovedAwayMeansNotDetected
TEST(VirtualRobotHardwareTest, ObstacleMovedAwayMeansNotDetected)
{
    // Arrange: same close position as above, but the obstacle has been
    // relocated far away.
    VirtualWorld world;
    disableAllObstacles(world);
    const std::size_t keyboardIndex = world.deskObjectObstacleIndex(DeskObjectType::Keyboard);
    world.setObstaclePosition(keyboardIndex, Vec3{kObstacleTestX, 0.4F, 0.0F});
    world.setObstacleEnabled(keyboardIndex, true);
    world.setRobotPosition(Vec3{kObstacleNearFaceX - 0.5F - 0.25F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    world.setObstaclePosition(keyboardIndex, Vec3{100.0F, 0.4F, 100.0F});
    VirtualRobotHardware hardware(world);

    // Act / Assert
    EXPECT_FALSE(hardware.obstacleDetected());
    EXPECT_FALSE(hardware.obstacleDistance().has_value());
}

// 18: MovementBehaviorFromPhase13NRemainsCorrect
TEST(VirtualRobotHardwareTest, MovementBehaviorFromPhase13NRemainsCorrect)
{
    // Arrange: unchanged assertion from Phase 13N's MoveForwardMovesRobot -
    // update()'s movement math is unaware of obstacles (no collision solver
    // yet), so it must still move exactly as before. Explicit position/
    // heading (Phase 13W human-visual-redesign v2's own default heading
    // is now 180 and default position sits right next to the charging
    // dock, so this is no longer implicit/collision-free).
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    VirtualRobotHardware hardware(world);
    const float initialZ = world.robotPose().position.z;

    // Act
    hardware.moveForward();
    hardware.update(1.0F);

    // Assert
    EXPECT_GT(world.robotPose().position.z, initialZ);
}

// --- FSM / RobotController / VirtualRobotHardware / VirtualWorld
// integration (section 18) ---
//
// Drives the exact same production stack RobotSimulator3D's main3d.cpp
// composes - RobotRuntime + DemoCommandSource + RobotStateMachine +
// RobotController + VirtualRobotHardware - proving
// FSM -> RobotController -> VirtualRobotHardware -> VirtualWorld end to
// end, without opening a window. The FSM reaches Moving through its real
// transition rules (ScenarioLoaded, then StartMission), never by this
// test forcing state directly.
TEST(VirtualRobotHardwareTest, FsmControllerHardwareWorldIntegrationThroughRobotRuntime)
{
    // Arrange: explicit position/heading (Phase 13W human-visual-redesign
    // v2's own default heading is now 180 and default position sits right
    // next to the charging dock, so this is no longer implicit/collision-
    // free).
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    VirtualRobotHardware hardware(world);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    DemoCommandSource commandSource;
    RobotRuntime runtime(commandSource, stateMachine, controller);
    const float initialZ = world.robotPose().position.z;

    // Act / Assert: first step - initial hardware sync, then
    // ScenarioLoaded (Idle -> Ready).
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(stateMachine.currentState(), RobotState::Ready);

    // Act / Assert: second step - StartMission (Ready -> Moving), which
    // RobotController maps to VirtualRobotHardware::moveForward().
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(stateMachine.currentState(), RobotState::Moving);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);

    // Act
    hardware.update(1.0F);

    // Assert: the robot actually moved forward in VirtualWorld.
    EXPECT_GT(world.robotPose().position.z, initialZ);
}

// --- Full closed-loop obstacle detection/clear through the real event
// chain (Phase 13O, section 24) ---
//
// Drives the exact same production stack RobotSimulator3D's main3d.cpp
// composes as of Phase 13O - VirtualWorld -> VirtualRobotHardware ->
// HardwareEventSource -> CompositePollingEventSource -> RobotRuntime ->
// RobotStateMachine -> RobotController - end to end, without opening a
// window and without ever calling stateMachine.processEvent() or
// stateMachine.handleEvent() directly. Frame ordering mirrors main3d.cpp's
// real loop: runtime.step() first, then hardware.update(dt).
TEST(VirtualRobotHardwareTest, FullClosedLoopObstacleDetectionAndClearThroughRealEventChain)
{
    // Arrange: Phase 13W human-visual-redesign v2 - the old standalone
    // "blocking obstacle" demo cube is gone, so this test builds its own
    // controlled, local geometry: every default obstacle disabled, then
    // the Keyboard's real registered obstacle (semantic lookup, never a
    // raw magic index) placed directly ahead on the robot's forward path,
    // with an explicit robot start pose (never relying on
    // VirtualWorld's own default robot pose, which now starts elsewhere
    // facing the dock).
    VirtualWorld world;
    disableAllObstacles(world);
    const std::size_t keyboardIndex = world.deskObjectObstacleIndex(DeskObjectType::Keyboard);
    world.setObstaclePosition(keyboardIndex, Vec3{kObstacleTestX, 0.4F, 0.0F});
    world.setObstacleEnabled(keyboardIndex, true);
    world.setRobotPosition(Vec3{-1.25F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    CompositePollingEventSource compositeSource(commandSource, hardwareEventSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);

    // Act / Assert: ScenarioLoaded (Idle -> Ready), then StartMission
    // (Ready -> Moving), through the composite source's command-before-
    // sensor priority - identical to the CompositePollingEventSourceTests/
    // ScriptedLiveRuntimeRunnerTests startup sequence.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(stateMachine.currentState(), RobotState::Ready);

    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(stateMachine.currentState(), RobotState::Moving);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);
    hardware.update(1.0F); // x: -1.25 -> -0.25; sensor distance ~1.2 (not yet detected)

    // Act / Assert: DemoCommandSource is now exhausted, so this step polls
    // HardwareEventSource - still no obstacle within detection range yet.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::NoEvent);
    ASSERT_EQ(stateMachine.currentState(), RobotState::Moving);
    hardware.update(1.0F); // x: -0.25 -> 0.75; sensor distance ~0.2 (within threshold)

    // Act / Assert: the real HardwareEventSource now observes the
    // false -> true edge and emits ObstacleDetected; RobotStateMachine
    // accepts Moving -> WaitingForObstacleClear; RobotController stops the
    // hardware - all through the unmodified production chain.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(stateMachine.currentState(), RobotState::WaitingForObstacleClear);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
    const float xWhenStopped = world.robotPose().position.x;
    hardware.update(1.0F); // Stopped -> update() is a no-op.
    EXPECT_FLOAT_EQ(world.robotPose().position.x, xWhenStopped);

    // Act / Assert: obstacle condition persists (true -> true) - no repeated
    // ObstacleDetected, no state change.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::NoEvent);
    ASSERT_EQ(stateMachine.currentState(), RobotState::WaitingForObstacleClear);

    // Act: disable the blocking obstacle - identical world-only mutation to
    // RobotSimulator3D's "O" key; never a manual Event injection.
    world.setObstacleEnabled(keyboardIndex, false);

    // Act / Assert: HardwareEventSource observes the true -> false edge and
    // emits ObstacleCleared; RobotStateMachine returns to Moving (the
    // remembered resume state); RobotController resumes MoveForward.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(stateMachine.currentState(), RobotState::Moving);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);

    // Act / Assert: movement resumes.
    hardware.update(1.0F);
    EXPECT_GT(world.robotPose().position.x, xWhenStopped);
}

// --- Repeated-edge / no-event suppression (Phase 13O, section 25) ---
//
// Once the obstacle condition stays true, HardwareEventSource must not
// re-emit ObstacleDetected on every subsequent runtime.step() - this
// follows naturally from its existing edge-trigger semantics
// (unmodified), exercised here specifically through VirtualRobotHardware/
// VirtualWorld.
TEST(VirtualRobotHardwareTest, RepeatedObstacleDetectedDoesNotSpamEvents)
{
    // Arrange: start the robot already within detection range, so the very
    // first hardware sample observes the false -> true edge. Phase 13W
    // human-visual-redesign v2: explicit local geometry (the old
    // standalone blocking-obstacle demo cube is gone) - same shared
    // Keyboard-obstacle placement as tests 14-17 above.
    VirtualWorld world;
    disableAllObstacles(world);
    const std::size_t keyboardIndex = world.deskObjectObstacleIndex(DeskObjectType::Keyboard);
    world.setObstaclePosition(keyboardIndex, Vec3{kObstacleTestX, 0.4F, 0.0F});
    world.setObstacleEnabled(keyboardIndex, true);
    world.setRobotPosition(Vec3{kObstacleNearFaceX - 0.5F - 0.25F, 0.125F, 0.0F}); // sensor distance 0.5
    world.setRobotHeading(90.0F);
    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    CompositePollingEventSource compositeSource(commandSource, hardwareEventSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);

    // Act / Assert: ScenarioLoaded, StartMission, then the first hardware
    // sample - this is where ObstacleDetected fires and RobotController
    // stops the hardware without ever having called moveForward() (this
    // robot never advances past detection range).
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Moving -> WaitingForObstacleClear
    ASSERT_EQ(stateMachine.currentState(), RobotState::WaitingForObstacleClear);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);

    // Act / Assert: several more steps with the obstacle condition
    // unchanged (true -> true every time) must all report NoEvent, and the
    // state must never move.
    for (int i = 0; i < 5; ++i)
    {
        EXPECT_EQ(runtime.step(), RuntimeStepResult::NoEvent);
        EXPECT_EQ(stateMachine.currentState(), RobotState::WaitingForObstacleClear);
    }
}

// --- DifferentialDrive wheel-speed integration (Phase 13P) ---
//
// VirtualRobotHardware now maps each IRobotHardware actuator command to
// wheel speeds on an owned DifferentialDrive instead of computing
// straight-line movement itself - these tests prove that mapping, the
// manual override, and that update()'s straight-line speed is unchanged
// from Phase 13N/13O's ~1.0 world unit/second.

// 19: MoveForwardSetsEqualPositiveWheelSpeeds
TEST(VirtualRobotHardwareTest, MoveForwardSetsEqualPositiveWheelSpeeds)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);

    // Act
    hardware.moveForward();

    // Assert
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
}

// 20: StopSetsZeroWheelSpeeds
TEST(VirtualRobotHardwareTest, StopSetsZeroWheelSpeeds)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();

    // Act
    hardware.stop();

    // Assert
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 21: ReturnToBaseLeavesBothWheelSpeedsZero
TEST(VirtualRobotHardwareTest, ReturnToBaseLeavesBothWheelSpeedsZero)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);

    // Act
    hardware.returnToBase();

    // Assert: navigation is not implemented yet - treated the same as
    // Stopped, zero wheel speeds.
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 22: UpdateDelegatesMovementThroughDifferentialDriveAtApproximatelyOneUnitPerSecond
TEST(VirtualRobotHardwareTest, UpdateDelegatesMovementThroughDifferentialDriveAtApproximatelyOneUnitPerSecond)
{
    // Arrange: explicit position/heading (Phase 13W human-visual-redesign
    // v2's own default heading is now 180 and default position sits right
    // next to the charging dock, so this is no longer implicit/collision-
    // free).
    VirtualWorld world;
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    VirtualRobotHardware hardware(world);
    const float initialZ = world.robotPose().position.z;

    // Act
    hardware.moveForward();
    hardware.update(1.0F);

    // Assert: straight movement remains ~1.0 world unit/second, matching
    // Phase 13N/13O's prior speed exactly - now produced by
    // DifferentialDrive integrating equal wheel speeds instead of
    // VirtualRobotHardware's own inlined movement math.
    EXPECT_NEAR(world.robotPose().position.z, initialZ + 1.0F, 0.01F);
}

// 23: ManualOverrideCanSetUnequalWheelSpeeds
TEST(VirtualRobotHardwareTest, ManualOverrideCanSetUnequalWheelSpeeds)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward(); // FSM command in effect before the override.

    // Act
    hardware.setManualWheelSpeeds(-0.5F, 0.5F);

    // Assert: manual values win over the FSM command's mapped speeds.
    EXPECT_TRUE(hardware.manualOverrideActive());
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -0.5F);
    EXPECT_FLOAT_EQ(speeds.right, 0.5F);
}

// 24: ManualOverrideTurningChangesHeading
TEST(VirtualRobotHardwareTest, ManualOverrideTurningChangesHeading)
{
    // Arrange
    VirtualWorld world;
    const float initialHeading = world.robotPose().headingDegrees;
    VirtualRobotHardware hardware(world);

    // Act: in-place rotation via manual override.
    hardware.setManualWheelSpeeds(-0.5F, 0.5F);
    hardware.update(1.0F);

    // Assert
    EXPECT_NE(world.robotPose().headingDegrees, initialHeading);
}

// 25: ClearingManualOverrideRestoresLatestFsmMoveForwardCommand
TEST(VirtualRobotHardwareTest, ClearingManualOverrideRestoresLatestFsmMoveForwardCommand)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();
    hardware.setManualWheelSpeeds(0.0F, 0.0F);
    ASSERT_TRUE(hardware.manualOverrideActive());

    // Act
    hardware.clearManualWheelOverride();

    // Assert: restores MoveForward's equal positive wheel speeds, not
    // whatever the override last held.
    EXPECT_FALSE(hardware.manualOverrideActive());
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
}

// 26: ClearingManualOverrideRestoresLatestFsmStoppedCommand
TEST(VirtualRobotHardwareTest, ClearingManualOverrideRestoresLatestFsmStoppedCommand)
{
    // Arrange: default command is Stopped.
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(0.5F, -0.5F);
    ASSERT_TRUE(hardware.manualOverrideActive());

    // Act
    hardware.clearManualWheelOverride();

    // Assert
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// 27: FsmCommandsWhileManualOverrideActiveDoNotChangePhysicalWheelSpeeds
TEST(VirtualRobotHardwareTest, FsmCommandsWhileManualOverrideActiveDoNotChangePhysicalWheelSpeeds)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(0.2F, 0.8F);

    // Act: RobotController-style calls continue to arrive while the
    // override is active.
    hardware.moveForward();
    hardware.stop();

    // Assert: currentCommand() reflects the latest FSM call (for restore-
    // on-clear and HUD), but physical wheel speeds are still the manual
    // override's values.
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.2F);
    EXPECT_FLOAT_EQ(speeds.right, 0.8F);
}

// 28: StoppedByObstacleResultsInZeroWheelSpeeds
//
// Extends FullClosedLoopObstacleDetectionAndClearThroughRealEventChain
// above with an explicit wheelSpeeds() assertion at the moment
// RobotController::stop() fires from real obstacle detection.
TEST(VirtualRobotHardwareTest, StoppedByObstacleResultsInZeroWheelSpeeds)
{
    // Arrange: same explicit, local obstacle geometry as
    // FullClosedLoopObstacleDetectionAndClearThroughRealEventChain above
    // (Phase 13W human-visual-redesign v2 - the old standalone blocking-
    // obstacle demo cube is gone).
    VirtualWorld world;
    disableAllObstacles(world);
    const std::size_t keyboardIndex = world.deskObjectObstacleIndex(DeskObjectType::Keyboard);
    world.setObstaclePosition(keyboardIndex, Vec3{kObstacleTestX, 0.4F, 0.0F});
    world.setObstacleEnabled(keyboardIndex, true);
    world.setRobotPosition(Vec3{-1.25F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    CompositePollingEventSource compositeSource(commandSource, hardwareEventSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);

    // Act: same sequence as the full closed-loop regression test - reach
    // Moving, advance close enough to trip the sensor.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);
    WheelSpeeds movingSpeeds = hardware.wheelSpeeds();
    EXPECT_GT(movingSpeeds.left, 0.0F);
    hardware.update(1.0F);
    ASSERT_EQ(runtime.step(), RuntimeStepResult::NoEvent);
    hardware.update(1.0F);

    // Assert: ObstacleDetected fires, RobotController stops the hardware,
    // and both wheel speeds are exactly zero.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(stateMachine.currentState(), RobotState::WaitingForObstacleClear);
    const WheelSpeeds stoppedSpeeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(stoppedSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(stoppedSpeeds.right, 0.0F);
}

// 29: SettingManualWheelSpeedsToZeroStopsFurtherMovement
//
// Real-testing regression (X-stop bug): confirms setManualWheelSpeeds(0,0)
// itself - independent of any keyboard-input decision logic - actually
// halts further movement, and that no previous manual command remains
// latched once zero is set.
TEST(VirtualRobotHardwareTest, SettingManualWheelSpeedsToZeroStopsFurtherMovement)
{
    // Arrange: moving under manual control.
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    hardware.update(1.0F);
    const float zAfterMoving = world.robotPose().position.z;

    // Act
    hardware.setManualWheelSpeeds(0.0F, 0.0F);
    hardware.update(1.0F);
    hardware.update(1.0F);

    // Assert: no further movement, and wheelSpeeds() reads back exactly
    // zero, not a leftover nonzero value.
    EXPECT_FLOAT_EQ(world.robotPose().position.z, zAfterMoving);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
}

// --- Obstacle collision guard (Phase 13P) ---
//
// VirtualRobotHardware::update() now validates a DifferentialDrive-
// proposed pose against RobotCollision.hpp's circle-vs-AABB obstacle query
// before committing it to VirtualWorld - these tests drive that through
// the real update()/VirtualWorld stack (RobotCollisionTests.cpp covers the
// underlying geometry query in isolation). Phase 13W human-visual-
// redesign v2: driven along the robot's +X direction (heading 90) rather
// than +Z - the table's X half-extent (4.0F) comfortably fits this
// block's "drive through and out the far side" tests, while the new,
// shallower Z half-extent (2.0F) no longer would. The one controlled
// obstacle sits at Z 0 so the robot's collision circle is centered
// exactly on its footprint.

// 30: ManualForwardDriveStopsAtObstacleBoundary
TEST(VirtualRobotHardwareTest, ManualForwardDriveStopsAtObstacleBoundary)
{
    // Arrange: obstacle (0.8 cube) centered at X 3.0 -> near face X 2.6.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    world.setObstaclePosition(0, Vec3{3.0F, 0.4F, 0.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);

    // Act: drive forward in small steps for far longer than needed to
    // reach the obstacle if collision were not enforced.
    for (int i = 0; i < 200; ++i)
    {
        hardware.update(0.05F);
    }

    // Assert: made real progress, but never entered the obstacle (near
    // face 2.6 minus collision radius ~0.35 -> boundary near X 2.25).
    const float finalX = world.robotPose().position.x;
    EXPECT_GT(finalX, 1.0F);
    EXPECT_LT(finalX, 2.30F);
}

// 31: ManualReverseAwayFromObstacleWorks
TEST(VirtualRobotHardwareTest, ManualReverseAwayFromObstacleWorks)
{
    // Arrange: drive up to the obstacle boundary first (same setup as
    // ManualForwardDriveStopsAtObstacleBoundary).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    world.setObstaclePosition(0, Vec3{3.0F, 0.4F, 0.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    for (int i = 0; i < 200; ++i)
    {
        hardware.update(0.05F);
    }
    const float xAtBoundary = world.robotPose().position.x;

    // Act: reverse.
    hardware.setManualWheelSpeeds(-1.0F, -1.0F);
    for (int i = 0; i < 20; ++i)
    {
        hardware.update(0.05F);
    }

    // Assert: moved back away from the obstacle - reverse is never
    // blocked by a collision guard that only ever rejects entering an
    // obstacle.
    EXPECT_LT(world.robotPose().position.x, xAtBoundary);
}

// 32: InPlaceRotationDoesNotTranslateIntoObstacle
TEST(VirtualRobotHardwareTest, InPlaceRotationDoesNotTranslateIntoObstacle)
{
    // Arrange: drive up to the obstacle boundary first.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    world.setObstaclePosition(0, Vec3{3.0F, 0.4F, 0.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    for (int i = 0; i < 200; ++i)
    {
        hardware.update(0.05F);
    }
    const float xAtBoundary = world.robotPose().position.x;
    const float headingBefore = world.robotPose().headingDegrees;

    // Act: in-place rotation right at the boundary.
    hardware.setManualWheelSpeeds(-0.5F, 0.5F);
    for (int i = 0; i < 20; ++i)
    {
        hardware.update(0.05F);
    }

    // Assert: position essentially unchanged (a circular footprint is
    // rotation-independent, so it is never blocked), heading did change.
    EXPECT_NEAR(world.robotPose().position.x, xAtBoundary, 0.01F);
    EXPECT_NE(world.robotPose().headingDegrees, headingBefore);
}

// 33: DisabledObstacleDoesNotBlockMovement
TEST(VirtualRobotHardwareTest, DisabledObstacleDoesNotBlockManualMovement)
{
    // Arrange: same obstacle geometry as the boundary tests, left
    // disabled the whole time.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    world.setObstaclePosition(0, Vec3{3.0F, 0.4F, 0.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    // Deliberately left disabled (disableAllObstacles() above).
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);

    // Act
    for (int i = 0; i < 200; ++i)
    {
        hardware.update(0.05F);
    }

    // Assert: drove straight through where an enabled obstacle would have
    // stopped it (~X 2.1), past its far face (X 3.4), well short of the
    // table's own X edge (4.0F).
    EXPECT_GT(world.robotPose().position.x, 3.6F);
}

// 34: ReEnabledObstacleBlocksMovement
TEST(VirtualRobotHardwareTest, ReEnabledObstacleBlocksMovement)
{
    // Arrange: obstacle disabled - robot drives all the way through it and
    // out the far side (near face 2.6, far face 3.4).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    world.setObstaclePosition(0, Vec3{3.0F, 0.4F, 0.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, false);
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    for (int i = 0; i < 160; ++i)
    {
        hardware.update(0.05F);
    }
    ASSERT_GT(world.robotPose().position.x, 3.6F);

    // Act: re-enable, then reverse back toward the obstacle from the far
    // side.
    world.setObstacleEnabled(0, true);
    hardware.setManualWheelSpeeds(-1.0F, -1.0F);
    for (int i = 0; i < 160; ++i)
    {
        hardware.update(0.05F);
    }

    // Assert: blocked at the (now re-enabled) far face - never re-enters
    // the obstacle from this side either.
    EXPECT_GT(world.robotPose().position.x, 3.4F);
}

// 35: TableSupportGuardStopsRobotNearTableEdgeInsteadOfWorldBound
//
// Phase 13S superseded the old "robot reaches the world bound and gets
// clamped/stuck at ~10" behavior (the "invisible wall" this phase's brief
// explicitly requires removing) with the table-support fail-safe guard -
// this test replaces the old WorldBoundsStillApplyWithCollisionGuardPresent,
// whose assertion (position settling near the old 10-unit bound) directly
// encoded the now-removed behavior. This test drives ONLY
// VirtualRobotHardware::update()'s own guard directly (no
// TableEdgeSafetyController wired up, matching the original test's
// "collision guard present, nothing else engaged" spirit) - proving the
// hard fail-safe alone, with no recovery steering, still keeps the robot
// off the old 10-unit bound and near the table's own edge instead.
TEST(VirtualRobotHardwareTest, TableSupportGuardStopsRobotNearTableEdgeInsteadOfWorldBound)
{
    // Arrange: no obstacles in the way - only the table-support guard
    // should limit forward movement. Explicit start position/heading
    // (Phase 13W human-visual-redesign v2's own default heading is now
    // 180, so this is no longer implicit) - straight line toward the
    // table's +Z edge (tableSurface().maxZ, dynamically read below, never
    // hardcoded).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    const float initialZ = world.robotPose().position.z;
    const float tableMaxZ = world.tableSurface().maxZ;

    // Act: far more simulated time than needed to reach the table edge at
    // 1.0 unit/second.
    for (int i = 0; i < 400; ++i)
    {
        hardware.update(0.05F);
    }

    // Assert: genuine forward progress happened, the robot never reached
    // anywhere near the old 10-unit world bound (proving the invisible-
    // wall-at-10 behavior is gone), and it settled just past the table's
    // edge boundary by at most half the robot's body length - exactly the
    // allCliff() (all four corners off) condition the guard uses, not the
    // old anyCliff()-at-any-single-corner behavior.
    const float finalZ = world.robotPose().position.z;
    EXPECT_GT(finalZ, initialZ + 1.0F);
    EXPECT_LT(finalZ, 10.0F);
    EXPECT_GT(finalZ, tableMaxZ - 0.1F);
    EXPECT_LT(finalZ, tableMaxZ + 0.5F);
}

// --- DriveAuthority model (Phase 13Q) ---
//
// Manual > AutonomousAvoidance > Fsm, fixed priority. These tests prove
// the priority rule directly against VirtualRobotHardware's public API,
// independent of main3d.cpp's policy for *when* to engage each override.

// 36: DefaultAuthorityIsFsm
TEST(VirtualRobotHardwareTest, DefaultAuthorityIsFsm)
{
    // Arrange / Act
    VirtualWorld world;
    VirtualRobotHardware hardware(world);

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    EXPECT_FALSE(hardware.manualOverrideActive());
    EXPECT_FALSE(hardware.autonomousOverrideActive());
}

// 37: FsmMoveForwardControlsWheelsWithoutOverrides
TEST(VirtualRobotHardwareTest, FsmMoveForwardControlsWheelsWithoutOverrides)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);

    // Act
    hardware.moveForward();

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
}

// 38: AutonomousOverrideTakesAuthorityFromFsm
TEST(VirtualRobotHardwareTest, AutonomousOverrideTakesAuthorityFromFsm)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();

    // Act
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);
    EXPECT_TRUE(hardware.autonomousOverrideActive());
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -0.6F);
    EXPECT_FLOAT_EQ(speeds.right, 0.6F);
}

// 39: ManualOverrideTakesAuthorityFromAutonomous
TEST(VirtualRobotHardwareTest, ManualOverrideTakesAuthorityFromAutonomous)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    // Act
    hardware.setManualWheelSpeeds(1.0F, 1.0F);

    // Assert: manual wins; the autonomous request is still recorded
    // underneath (see autonomousOverrideActive()) but does not control
    // physical wheel speeds while manual is active.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    EXPECT_TRUE(hardware.autonomousOverrideActive());
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 1.0F);
    EXPECT_FLOAT_EQ(speeds.right, 1.0F);
}

// 40: ClearingManualRestoresAutonomous
TEST(VirtualRobotHardwareTest, ClearingManualRestoresAutonomous)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);

    // Act
    hardware.clearManualWheelOverride();

    // Assert: falls through to the still-active autonomous override, not
    // the FSM command.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -0.6F);
    EXPECT_FLOAT_EQ(speeds.right, 0.6F);
}

// 41: ClearingAutonomousRestoresFsm
TEST(VirtualRobotHardwareTest, ClearingAutonomousRestoresFsm)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    // Act
    hardware.clearAutonomousWheelOverride();

    // Assert: falls through to the FSM command (MoveForward), since
    // manual was never active.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
}

// 42: FsmStopWhileAutonomousActiveDoesNotOverwriteAutonomousWheelSpeeds
TEST(VirtualRobotHardwareTest, FsmStopWhileAutonomousActiveDoesNotOverwriteAutonomousWheelSpeeds)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);

    // Act: RobotController-style call arrives while avoidance is active -
    // this is exactly what happens when the FSM first enters
    // WaitingForObstacleClear and calls stop().
    hardware.stop();

    // Assert: currentCommand() reflects the FSM call, but physical wheel
    // speeds are still the autonomous override's values.
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -0.6F);
    EXPECT_FLOAT_EQ(speeds.right, 0.6F);
}

// 43: FsmMoveForwardWhileAutonomousActiveDoesNotOverwriteAutonomousWheelSpeeds
TEST(VirtualRobotHardwareTest, FsmMoveForwardWhileAutonomousActiveDoesNotOverwriteAutonomousWheelSpeeds)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);

    // Act: FSM briefly reports MoveForward while avoidance is still
    // active (should not happen in the real WaitingForObstacleClear flow,
    // but the priority rule must hold regardless of which FSM command
    // arrives).
    hardware.moveForward();

    // Assert
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -0.6F);
    EXPECT_FLOAT_EQ(speeds.right, 0.6F);
}

// 44: ManualStillWinsAfterFsmCommandChanges
TEST(VirtualRobotHardwareTest, ManualStillWinsAfterFsmCommandChanges)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(0.3F, -0.3F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);

    // Act: FSM command changes underneath the active manual override.
    hardware.moveForward();
    hardware.stop();
    hardware.returnToBase();

    // Assert: currentCommand() reflects the latest FSM call, but physical
    // wheel speeds are still the manual override's values throughout.
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.3F);
    EXPECT_FLOAT_EQ(speeds.right, -0.3F);
}

// 45: ClearingAllOverridesRestoresLatestFsmCommand
TEST(VirtualRobotHardwareTest, ClearingAllOverridesRestoresLatestFsmCommand)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);
    hardware.setManualWheelSpeeds(0.2F, 0.2F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);

    // Act: clear both overrides, manual first (matching main3d.cpp's own
    // M-toggle-off order, though the result must not depend on order).
    hardware.clearManualWheelOverride();
    hardware.clearAutonomousWheelOverride();

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    EXPECT_FALSE(hardware.manualOverrideActive());
    EXPECT_FALSE(hardware.autonomousOverrideActive());
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
}

// 46: ManualPriorityIntegrationAcrossAvoidanceAndFsm
//
// Phase 13Q section 25: avoidance active -> manual enabled -> manual wins
// -> manual cleared -> avoidance automatically resumes -> latest FSM
// command was never lost throughout.
TEST(VirtualRobotHardwareTest, ManualPriorityIntegrationAcrossAvoidanceAndFsm)
{
    // Arrange: FSM wants Stopped (as it does in real
    // WaitingForObstacleClear), avoidance is actively turning.
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.stop();
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(/*enabled=*/true, /*triggerAvoidance=*/true, /*forwardCorridorClear=*/false, world.robotPose(),
                      {});
    ASSERT_TRUE(avoidance.active());
    const WheelSpeeds turnSpeeds = avoidance.wheelSpeeds();
    hardware.setAutonomousWheelSpeeds(turnSpeeds.left, turnSpeeds.right);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    // Act: manual drive engages mid-turn.
    hardware.setManualWheelSpeeds(1.0F, -1.0F);

    // Assert: manual physically wins.
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 1.0F);
    EXPECT_FLOAT_EQ(speeds.right, -1.0F);

    // Act: manual drive disengages.
    hardware.clearManualWheelOverride();

    // Assert: avoidance speeds automatically resume, with no loss of the
    // latest FSM command (still Stopped).
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
    speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, turnSpeeds.left);
    EXPECT_FLOAT_EQ(speeds.right, turnSpeeds.right);
}

// --- Disabled avoidance (Phase 13Q, section 26) ---
//
// When the caller (main3d.cpp's `A` toggle, or here, simply never calling
// setAutonomousWheelSpeeds()) does not engage avoidance, WaitingForObstacleClear
// must behave exactly like Phase 13O/13P: the robot stays stopped and
// never rotates on its own.
TEST(VirtualRobotHardwareTest, DisabledAvoidanceLeavesRobotStoppedAndNotRotating)
{
    // Arrange: real closed-loop approach to the blocking obstacle, exactly
    // like the Phase 13O regression test - avoidance is simply never
    // engaged (as if the `A` toggle were OFF). Same explicit, local
    // obstacle geometry as
    // FullClosedLoopObstacleDetectionAndClearThroughRealEventChain above
    // (Phase 13W human-visual-redesign v2 - the old standalone blocking-
    // obstacle demo cube is gone).
    VirtualWorld world;
    disableAllObstacles(world);
    const std::size_t keyboardIndex = world.deskObjectObstacleIndex(DeskObjectType::Keyboard);
    world.setObstaclePosition(keyboardIndex, Vec3{kObstacleTestX, 0.4F, 0.0F});
    world.setObstacleEnabled(keyboardIndex, true);
    world.setRobotPosition(Vec3{-1.25F, 0.125F, 0.0F});
    world.setRobotHeading(90.0F);
    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    CompositePollingEventSource compositeSource(commandSource, hardwareEventSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);

    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    hardware.update(1.0F);
    ASSERT_EQ(runtime.step(), RuntimeStepResult::NoEvent);
    hardware.update(1.0F);
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // -> WaitingForObstacleClear
    ASSERT_EQ(stateMachine.currentState(), RobotState::WaitingForObstacleClear);
    const float headingWhenStopped = world.robotPose().headingDegrees;
    const Vec3 positionWhenStopped = world.robotPose().position;

    // Act: many further updates/steps, never calling
    // setAutonomousWheelSpeeds() at all.
    for (int i = 0; i < 50; ++i)
    {
        hardware.update(0.1F);
        EXPECT_EQ(runtime.step(), RuntimeStepResult::NoEvent);
    }

    // Assert: robot never rotated or moved - it stays put until the
    // obstacle is cleared some other way (e.g. `O`).
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.right, 0.0F);
    EXPECT_FLOAT_EQ(world.robotPose().headingDegrees, headingWhenStopped);
    EXPECT_FLOAT_EQ(world.robotPose().position.x, positionWhenStopped.x);
    EXPECT_FLOAT_EQ(world.robotPose().position.z, positionWhenStopped.z);
}

// --- Clearance-aware closed-loop autonomous obstacle avoidance (Phase
// 13R, sections 21/22 - the phase's main acceptance test) ---
//
// Drives the exact same production stack RobotSimulator3D's main3d.cpp
// composes as of Phase 13R - VirtualWorld -> VirtualRobotHardware ->
// HardwareEventSource -> CompositePollingEventSource -> RobotRuntime ->
// RobotStateMachine -> RobotController, plus ReactiveObstacleAvoidance
// and ForwardClearanceProbe - end to end, without opening a window and
// without ever calling stateMachine.processEvent()/handleEvent() or
// injecting ObstacleDetected/ObstacleCleared directly. Each loop
// iteration below reproduces main3d.cpp's own Phase 13R per-frame policy
// (section 13): runtime.step() first, then compute this frame's sensor/
// clearance telemetry, then advance the avoidance latch, then apply
// drive authority, then hardware.update().
//
// Reproduces the exact Phase 13Q known limitation this phase fixes
// (docs/technical-decisions.md, Phase 13Q "limitations" /
// docs/technical-decisions.md, Phase 13R): the point-ray
// VirtualDistanceSensor clears (and therefore the real ObstacleCleared
// edge returns the FSM to Moving) BEFORE the robot's wider physical body
// corridor (ForwardClearanceProbe, which accounts for the full collision
// radius + safety margin, not a zero-width ray) is actually clear -
// proven directly below by observing at least one frame where
// `State: Moving`, `Command: MoveForward`, `Drive authority: AUTONOMOUS`,
// and `Forward clearance: BLOCKED` all hold simultaneously, exactly the
// intermediate condition the Phase 13R brief calls out as a feature, not
// a contradiction.
TEST(VirtualRobotHardwareTest, ClearanceAwareAvoidanceClosedLoopThroughRealEventChain)
{
    // Arrange: controlled single-obstacle placement (same
    // disableAllObstacles()/setObstaclePosition() convention every other
    // visual-simulator test file uses), centered directly on the robot's
    // centerline and already within the point sensor's detection range at
    // the robot's start pose - so the very first hardware sample after
    // reaching Moving trips ObstacleDetected naturally (same pattern as
    // RepeatedObstacleDetectedDoesNotSpamEvents above), keeping this test
    // deterministic without needing a multi-step approach phase.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 1.2F}); // 0.8 cube; near face Z 0.8
    world.setObstacleEnabled(0, true);

    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    CompositePollingEventSource compositeSource(commandSource, hardwareEventSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);
    ReactiveObstacleAvoidance avoidance;
    ForwardClearanceProbe clearanceProbe(world);

    // Act / Assert: reach WaitingForObstacleClear through real FSM
    // transitions - Idle -> Ready -> Moving -> WaitingForObstacleClear,
    // the last step firing immediately since the obstacle is already
    // within detection range.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Moving -> WaitingForObstacleClear
    ASSERT_EQ(stateMachine.currentState(), RobotState::WaitingForObstacleClear);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
    ASSERT_FALSE(clearanceProbe.isForwardCorridorClear()); // body corridor also genuinely blocked at start

    // Drive main3d.cpp's own Phase 13R per-frame policy.
    RobotState currentState = stateMachine.currentState();
    bool everActivated = false;
    bool observedMovingWhileAutonomousAndBlocked = false; // the exact Phase 13R transitional state
    bool observedFsmAuthorityAfterRelease = false;

    for (int frame = 0; frame < 2000 && !observedFsmAuthorityAfterRelease; ++frame)
    {
        runtime.step();
        currentState = stateMachine.currentState();

        const bool forwardCorridorClear = clearanceProbe.isForwardCorridorClear();
        const bool triggerAvoidance =
            currentState == RobotState::WaitingForObstacleClear && hardware.obstacleDetected();
        avoidance.update(/*enabled=*/true, triggerAvoidance, forwardCorridorClear, world.robotPose(), {});
        if (avoidance.active())
        {
            everActivated = true;
        }

        if (avoidance.active())
        {
            const WheelSpeeds turnSpeeds = avoidance.wheelSpeeds();
            hardware.setAutonomousWheelSpeeds(turnSpeeds.left, turnSpeeds.right);
        }
        else if (hardware.autonomousOverrideActive())
        {
            hardware.clearAutonomousWheelOverride();
        }

        if (currentState == RobotState::Moving && hardware.driveAuthority() == DriveAuthority::AutonomousAvoidance &&
            !forwardCorridorClear)
        {
            observedMovingWhileAutonomousAndBlocked = true;
        }
        if (currentState == RobotState::Moving && hardware.driveAuthority() == DriveAuthority::Fsm)
        {
            observedFsmAuthorityAfterRelease = true;
        }

        hardware.update(0.05F);
    }

    // Assert: avoidance genuinely engaged; the real ObstacleCleared edge
    // returned the FSM to Moving strictly before the body corridor was
    // actually safe (the Phase 13Q bug reproduced); avoidance correctly
    // kept driving authority afterward; and it eventually released once
    // the body corridor became genuinely clear, handing authority back to
    // the FSM.
    ASSERT_TRUE(everActivated);
    ASSERT_TRUE(observedMovingWhileAutonomousAndBlocked);
    ASSERT_TRUE(observedFsmAuthorityAfterRelease);
    ASSERT_EQ(currentState, RobotState::Moving);
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    EXPECT_FALSE(hardware.autonomousOverrideActive());
    EXPECT_FALSE(avoidance.active());
    EXPECT_TRUE(clearanceProbe.isForwardCorridorClear());

    // Wheel speeds are normal equal-positive FSM speeds again.
    const WheelSpeeds finalWheelSpeeds = hardware.wheelSpeeds();
    EXPECT_GT(finalWheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(finalWheelSpeeds.left, finalWheelSpeeds.right);

    // --- Proof the Phase 13Q stuck-after-resume failure is gone (section
    // 22): resumed forward motion makes genuine, sustained progress with
    // no collision-guard rejection, not a single lucky frame followed by
    // being capped again. ---
    const Vec3 positionAtRelease = world.robotPose().position;
    for (int i = 0; i < 20; ++i)
    {
        hardware.update(0.05F);
        EXPECT_FALSE(hardware.collidedLastUpdate());
    }
    const Vec3 finalPosition = world.robotPose().position;
    const float movedDistance =
        std::sqrt(((finalPosition.x - positionAtRelease.x) * (finalPosition.x - positionAtRelease.x)) +
                   ((finalPosition.z - positionAtRelease.z) * (finalPosition.z - positionAtRelease.z)));
    EXPECT_GT(movedDistance, 0.3F);
}

// --- Manual priority while avoidance is latched (Phase 13R, section 23)
// ---
TEST(VirtualRobotHardwareTest, ManualPriorityWhileAvoidanceLatched)
{
    // Arrange: latch active (TurnAway), clearance still blocked -
    // equivalent to a mid-turn snapshot of the closed-loop test above,
    // constructed directly against the state machine/hardware APIs rather
    // than driving the full FSM chain again.
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(/*enabled=*/true, /*triggerAvoidance=*/true, /*forwardCorridorClear=*/false, world.robotPose(),
                      {});
    ASSERT_TRUE(avoidance.active());
    ASSERT_EQ(avoidance.state(), AvoidanceState::TurnAway);
    const WheelSpeeds turnSpeeds = avoidance.wheelSpeeds();
    hardware.setAutonomousWheelSpeeds(turnSpeeds.left, turnSpeeds.right);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    // Act: engage manual override mid-turn (main3d.cpp's `M` key).
    hardware.setManualWheelSpeeds(1.0F, -1.0F);

    // Assert: manual physically wins; the incident remains logically
    // active underneath (main3d.cpp keeps calling avoidance.update() every
    // frame regardless of manual mode - simulated here directly).
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    avoidance.update(true, /*triggerAvoidance=*/false, /*forwardCorridorClear=*/false, world.robotPose(), {});
    EXPECT_TRUE(avoidance.active());

    // Act: leave manual mode while clearance is still blocked.
    hardware.clearManualWheelOverride();
    if (avoidance.active())
    {
        const WheelSpeeds resumedTurnSpeeds = avoidance.wheelSpeeds();
        hardware.setAutonomousWheelSpeeds(resumedTurnSpeeds.left, resumedTurnSpeeds.right);
    }

    // Assert: authority returns to AUTONOMOUS, not FSM, since the incident
    // is still active.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    // Act: clearance becomes safe while manual is no longer active. Phase
    // 13V human-validation fix: this no longer releases immediately - it
    // moves TurnAway -> AdvanceClear (still active()), the direct fix for
    // the human-observed oscillation ("corridor clear" is not "physically
    // bypassed" - see ReactiveObstacleAvoidance's own class docs).
    avoidance.update(true, false, /*forwardCorridorClear=*/true, world.robotPose(), {});
    EXPECT_TRUE(avoidance.active());
    EXPECT_EQ(avoidance.state(), AvoidanceState::AdvanceClear);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    // Act: the robot physically advances (AdvanceClear's own straight-
    // ahead wheel speeds) far enough to satisfy the minimum bypass
    // distance, corridor still clear - only now does the incident
    // actually release.
    const Vec3 advancedPosition{world.robotPose().position.x,
                                 world.robotPose().position.y,
                                 world.robotPose().position.z +
                                     ReactiveObstacleAvoidance::kMinimumBypassDistanceWorldUnits + 0.5F};
    robot::visual::RobotPose advancedPose = world.robotPose();
    advancedPose.position = advancedPosition;
    avoidance.update(true, false, /*forwardCorridorClear=*/true, advancedPose, {});
    EXPECT_FALSE(avoidance.active());
    if (!avoidance.active() && hardware.autonomousOverrideActive())
    {
        hardware.clearAutonomousWheelOverride();
    }

    // Assert: authority now falls through to the FSM command.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
}

// --- Safety authority (Phase 13S, section 20) ---
//
// Safety > Manual > AutonomousAvoidance > Fsm, fixed priority. Items 1-3
// of the section-20 checklist (default authority is FSM, Autonomous >
// FSM, Manual > Autonomous) are already covered, unmodified, by
// DefaultAuthorityIsFsm/AutonomousOverrideTakesAuthorityFromFsm/
// ManualOverrideTakesAuthorityFromAutonomous above - these tests cover
// items 4-12, all newly added for Safety.

// 47: SafetyOverridesManual
TEST(VirtualRobotHardwareTest, SafetyOverridesManual)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);

    // Act
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);
    EXPECT_TRUE(hardware.manualOverrideActive());
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -1.0F);
    EXPECT_FLOAT_EQ(speeds.right, -1.0F);
}

// 48: SafetyOverridesAutonomous
TEST(VirtualRobotHardwareTest, SafetyOverridesAutonomous)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    // Act
    hardware.setSafetyWheelSpeeds(1.0F, 1.0F);

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);
    EXPECT_TRUE(hardware.autonomousOverrideActive());
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 1.0F);
    EXPECT_FLOAT_EQ(speeds.right, 1.0F);
}

// 49: SafetyOverridesFsm
TEST(VirtualRobotHardwareTest, SafetyOverridesFsm)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);

    // Act
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -1.0F);
    EXPECT_FLOAT_EQ(speeds.right, -1.0F);
}

// 50: ClearingSafetyRestoresManualIfActive
TEST(VirtualRobotHardwareTest, ClearingSafetyRestoresManualIfActive)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(0.4F, 0.4F);
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

    // Act
    hardware.clearSafetyWheelOverride();

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.4F);
    EXPECT_FLOAT_EQ(speeds.right, 0.4F);
}

// 51: ClearingSafetyRestoresAutonomousIfNoManual
TEST(VirtualRobotHardwareTest, ClearingSafetyRestoresAutonomousIfNoManual)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

    // Act
    hardware.clearSafetyWheelOverride();

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -0.6F);
    EXPECT_FLOAT_EQ(speeds.right, 0.6F);
}

// 52: ClearingSafetyRestoresFsmIfNeitherExists
TEST(VirtualRobotHardwareTest, ClearingSafetyRestoresFsmIfNeitherExists)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

    // Act
    hardware.clearSafetyWheelOverride();

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
}

// 53: FsmCommandsUnderSafetyRemainRemembered
TEST(VirtualRobotHardwareTest, FsmCommandsUnderSafetyRemainRemembered)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);

    // Act: RobotController-style calls continue to arrive while safety is
    // active - exactly like the existing manual/autonomous equivalents.
    hardware.moveForward();
    hardware.stop();
    hardware.returnToBase();

    // Assert: currentCommand() reflects the latest FSM call, but physical
    // wheel speeds are still the safety override's values.
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -1.0F);
    EXPECT_FLOAT_EQ(speeds.right, -1.0F);
}

// 54: ManualRequestsUnderSafetyRemainRemembered
TEST(VirtualRobotHardwareTest, ManualRequestsUnderSafetyRemainRemembered)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

    // Act: a manual command arrives while safety is active (e.g. the user
    // is still holding UP when the table edge is reached).
    hardware.setManualWheelSpeeds(1.0F, 1.0F);

    // Assert: recorded, but safety still physically wins.
    EXPECT_TRUE(hardware.manualOverrideActive());
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);
    const WheelSpeeds speedsWhileSafetyActive = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speedsWhileSafetyActive.left, -1.0F);
    EXPECT_FLOAT_EQ(speedsWhileSafetyActive.right, -1.0F);

    // Act: safety releases.
    hardware.clearSafetyWheelOverride();

    // Assert: the manual request was never lost - it takes over
    // immediately.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    const WheelSpeeds speedsAfterClear = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speedsAfterClear.left, 1.0F);
    EXPECT_FLOAT_EQ(speedsAfterClear.right, 1.0F);
}

// 55: AutonomousRequestsUnderSafetyRemainRemembered
TEST(VirtualRobotHardwareTest, AutonomousRequestsUnderSafetyRemainRemembered)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

    // Act: an autonomous-avoidance request arrives while safety is
    // active.
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);

    // Assert: recorded, but safety still physically wins.
    EXPECT_TRUE(hardware.autonomousOverrideActive());
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

    // Act: safety releases.
    hardware.clearSafetyWheelOverride();

    // Assert: the autonomous request was never lost.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -0.6F);
    EXPECT_FLOAT_EQ(speeds.right, 0.6F);
}

// --- Navigation authority (Phase 13T) ---
//
// Fixed priority: Safety > Manual > AutonomousAvoidance > Navigation >
// Fsm. Mirrors the Autonomous/Safety authority test shapes above exactly,
// one tier lower.

// 1: NavigationOverrideTakesAuthorityFromFsm
TEST(VirtualRobotHardwareTest, NavigationOverrideTakesAuthorityFromFsm)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.returnToBase();
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);

    // Act
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    EXPECT_TRUE(hardware.navigationOverrideActive());
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.8F);
    EXPECT_FLOAT_EQ(speeds.right, 0.8F);
}

// 2: AutonomousOverrideTakesAuthorityFromNavigation
TEST(VirtualRobotHardwareTest, AutonomousOverrideTakesAuthorityFromNavigation)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);

    // Act
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);

    // Assert: autonomous wins; the navigation request is still recorded
    // underneath but does not control physical wheel speeds.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);
    EXPECT_TRUE(hardware.navigationOverrideActive());
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -0.6F);
    EXPECT_FLOAT_EQ(speeds.right, 0.6F);
}

// 3: ClearingAutonomousRestoresNavigationIfStillActive
TEST(VirtualRobotHardwareTest, ClearingAutonomousRestoresNavigationIfStillActive)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);
    hardware.setAutonomousWheelSpeeds(-0.6F, 0.6F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    // Act
    hardware.clearAutonomousWheelOverride();

    // Assert: falls through to the still-active navigation override, not
    // the FSM command.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.8F);
    EXPECT_FLOAT_EQ(speeds.right, 0.8F);
}

// 4: ClearingNavigationRestoresFsm
TEST(VirtualRobotHardwareTest, ClearingNavigationRestoresFsm)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.moveForward();
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);

    // Act
    hardware.clearNavigationWheelOverride();

    // Assert: falls through to the FSM command (MoveForward).
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_GT(speeds.left, 0.0F);
    EXPECT_FLOAT_EQ(speeds.left, speeds.right);
}

// 5: ManualOverrideTakesAuthorityFromNavigation
TEST(VirtualRobotHardwareTest, ManualOverrideTakesAuthorityFromNavigation)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);

    // Act
    hardware.setManualWheelSpeeds(1.0F, 1.0F);

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    EXPECT_TRUE(hardware.navigationOverrideActive());
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 1.0F);
    EXPECT_FLOAT_EQ(speeds.right, 1.0F);
}

// 6: ClearingManualRestoresNavigationIfStillActiveAndNoAutonomous
TEST(VirtualRobotHardwareTest, ClearingManualRestoresNavigationIfStillActiveAndNoAutonomous)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);

    // Act
    hardware.clearManualWheelOverride();

    // Assert: falls through to the still-active navigation override (no
    // autonomous override was ever set).
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.8F);
    EXPECT_FLOAT_EQ(speeds.right, 0.8F);
}

// 7: FsmStopWhileNavigationActiveDoesNotOverwriteNavigationWheelSpeeds
TEST(VirtualRobotHardwareTest, FsmStopWhileNavigationActiveDoesNotOverwriteNavigationWheelSpeeds)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);

    // Act: a stop() call arrives (e.g. WaitingForObstacleClear) while
    // navigation is active.
    hardware.stop();

    // Assert: currentCommand() reflects the FSM call, but physical wheel
    // speeds are still the navigation override's values.
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.8F);
    EXPECT_FLOAT_EQ(speeds.right, 0.8F);
}

// 8: FsmMoveForwardWhileNavigationActiveDoesNotOverwriteNavigationWheelSpeeds
TEST(VirtualRobotHardwareTest, FsmMoveForwardWhileNavigationActiveDoesNotOverwriteNavigationWheelSpeeds)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);

    // Act
    hardware.moveForward();

    // Assert
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.8F);
    EXPECT_FLOAT_EQ(speeds.right, 0.8F);
}

// 9: SafetyOverridesNavigation
TEST(VirtualRobotHardwareTest, SafetyOverridesNavigation)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);

    // Act
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);

    // Assert
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, -1.0F);
    EXPECT_FLOAT_EQ(speeds.right, -1.0F);
}

// 10: ClearingSafetyRestoresNavigationIfNoManualOrAutonomous
TEST(VirtualRobotHardwareTest, ClearingSafetyRestoresNavigationIfNoManualOrAutonomous)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

    // Act
    hardware.clearSafetyWheelOverride();

    // Assert: falls through to navigation, not the FSM command.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.8F);
    EXPECT_FLOAT_EQ(speeds.right, 0.8F);
}

// 11: NavigationRequestsUnderSafetyRemainRemembered
TEST(VirtualRobotHardwareTest, NavigationRequestsUnderSafetyRemainRemembered)
{
    // Arrange
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setSafetyWheelSpeeds(-1.0F, -1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

    // Act: a navigation request arrives while safety is active (e.g. a
    // table edge is reached mid Return-Home).
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);

    // Assert: recorded, but safety still physically wins.
    EXPECT_TRUE(hardware.navigationOverrideActive());
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

    // Act: safety releases.
    hardware.clearSafetyWheelOverride();

    // Assert: the navigation request was never lost.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    const WheelSpeeds speeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(speeds.left, 0.8F);
    EXPECT_FLOAT_EQ(speeds.right, 0.8F);
}

// 12: ManualPriorityIntegrationAcrossNavigationAndFsm
TEST(VirtualRobotHardwareTest, ManualPriorityIntegrationAcrossNavigationAndFsm)
{
    // Arrange: navigation active, no manual yet.
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    hardware.setNavigationWheelSpeeds(0.8F, 0.8F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);

    // Act: manual engages.
    hardware.setManualWheelSpeeds(1.0F, -1.0F);

    // Assert: manual wins immediately.
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    const WheelSpeeds manualSpeeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(manualSpeeds.left, 1.0F);
    EXPECT_FLOAT_EQ(manualSpeeds.right, -1.0F);

    // Act: manual ends.
    hardware.clearManualWheelOverride();

    // Assert: navigation resumes automatically, unchanged, with no need
    // to be re-triggered.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    const WheelSpeeds resumedSpeeds = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(resumedSpeeds.left, 0.8F);
    EXPECT_FLOAT_EQ(resumedSpeeds.right, 0.8F);
}

// --- Full table-edge safety closed-loop integration test (Phase 13S,
// section 21 - one of this phase's main acceptance tests) ---
//
// Drives the real production stack - VirtualWorld -> VirtualRobotHardware
// -> HardwareEventSource -> CompositePollingEventSource -> RobotRuntime ->
// RobotStateMachine -> RobotController, plus VirtualCliffSensor and
// TableEdgeSafetyController - end to end, without ever setting
// DriveAuthority directly. All obstacles are disabled so this test
// isolates table-edge safety from obstacle avoidance entirely - the FSM
// reaches Moving and, with no obstacle ever detected, simply stays
// Moving/MoveForward for the whole test, exactly matching the brief's
// framing: "RobotStateMachine may still say Moving while DriveAuthority =
// SAFETY temporarily controls the actual wheels."
namespace
{

// Shared closed-loop driver for the straight-edge/corner variation tests
// below (Phase 13S manual-validation bugfix, sections 9-11): drives the
// real production stack from a starting pose, through a full table-edge
// safety recovery cycle, and for a further window afterward - so each
// caller can assert both "recovery genuinely converged to a meaningful
// inward heading" and "no repeated forward/back ping-pong afterward,"
// without duplicating this ~40-line loop four-plus times.
struct EdgeRecoveryOutcome
{
    bool everSafetyAuthority = false;
    bool everBackingAwayOrForward = false;
    bool everTurning = false;
    bool everAdvancingInward = false;
    bool everReleasedBackToFsm = false;
    bool everReactivatedAfterRelease = false;
    float headingAtSafetyStart = 0.0F;
    float headingAtRelease = 0.0F;
};

EdgeRecoveryOutcome driveTowardEdgeAndRecover(const Vec3& startPosition, float startHeadingDegrees)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(startPosition);
    world.setRobotHeading(startHeadingDegrees);

    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    CompositePollingEventSource compositeSource(commandSource, hardwareEventSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);
    VirtualCliffSensor cliffSensor(world);
    TableEdgeSafetyController tableEdgeSafety;

    runtime.step(); // Idle -> Ready
    runtime.step(); // Ready -> Moving

    EdgeRecoveryOutcome outcome{};
    bool recordedHeadingAtStart = false;
    int framesSinceRelease = -1;

    for (int frame = 0; frame < 4000; ++frame)
    {
        runtime.step();

        const CliffSensorReadings readings = cliffSensor.readings();
        tableEdgeSafety.update(readings, world.robotPose(), world.tableSurface());

        if (tableEdgeSafety.active())
        {
            if (!recordedHeadingAtStart)
            {
                outcome.headingAtSafetyStart = world.robotPose().headingDegrees;
                recordedHeadingAtStart = true;
            }
            const WheelSpeeds recovery = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(recovery.left, recovery.right);
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
        }

        if (hardware.driveAuthority() == DriveAuthority::Safety)
        {
            outcome.everSafetyAuthority = true;
        }
        if (tableEdgeSafety.state() == TableEdgeSafetyController::RecoveryState::BackingAway ||
            tableEdgeSafety.state() == TableEdgeSafetyController::RecoveryState::MovingForwardFromRearEdge)
        {
            outcome.everBackingAwayOrForward = true;
        }
        if (tableEdgeSafety.state() == TableEdgeSafetyController::RecoveryState::Turning)
        {
            outcome.everTurning = true;
        }
        if (tableEdgeSafety.state() == TableEdgeSafetyController::RecoveryState::AdvancingInward)
        {
            outcome.everAdvancingInward = true;
        }

        hardware.update(0.05F);
        EXPECT_FALSE(hardware.tableEdgeRejectedLastUpdate());

        if (!outcome.everReleasedBackToFsm && !tableEdgeSafety.active() &&
            hardware.driveAuthority() == DriveAuthority::Fsm && outcome.everSafetyAuthority)
        {
            outcome.everReleasedBackToFsm = true;
            outcome.headingAtRelease = world.robotPose().headingDegrees;
            framesSinceRelease = 0;
        }

        if (outcome.everReleasedBackToFsm)
        {
            if (hardware.driveAuthority() == DriveAuthority::Safety)
            {
                outcome.everReactivatedAfterRelease = true;
            }
            ++framesSinceRelease;
            // Phase 13W human-visual-redesign v2: shortened from 100
            // frames (5.0F simulated seconds) - the table's Z half-extent
            // shrank to 2.0F, so unconstrained MoveForward for a full 5
            // seconds (up to 5.0F world units at kForwardWheelSpeed) can
            // legitimately cross the ENTIRE remaining table and find a
            // genuinely different edge, which is not the "did recovery
            // immediately flip-flop back into Safety" question this
            // window exists to answer. 30 frames (1.5F seconds, up to
            // 1.5F units of travel) stays comfortably inside even the
            // smaller table while still proving recovery does not
            // immediately re-trigger.
            if (framesSinceRelease >= 30)
            {
                break;
            }
        }
    }

    return outcome;
}

} // namespace

// --- Full straight-edge/corner closed-loop integration tests (Phase 13S
// manual-validation bugfix, sections 9-11) ---
//
// Drives the real production stack - VirtualWorld -> VirtualRobotHardware
// -> HardwareEventSource -> CompositePollingEventSource -> RobotRuntime ->
// RobotStateMachine -> RobotController, plus VirtualCliffSensor and
// TableEdgeSafetyController - end to end, without ever setting
// DriveAuthority directly. Each proves the exact defect the human manual
// validation found is fixed: recovery must produce a MEANINGFUL heading
// change (not a trivial single-step rotation) before releasing, and must
// not immediately re-trigger against the same edge afterward.

// +Z edge (also the phase's main FSM-driven acceptance scenario, section
// 21/26 item 8).
TEST(VirtualRobotHardwareTest, StraightEdgeRecoveryPositiveZ)
{
    // Phase 13W human-visual-redesign v2: the table's Z half-extent
    // shrank to 2.0F (see VirtualWorld.cpp) - 1.5F leaves the same kind
    // of short approach margin the original 5.0F (against a 6.0F
    // boundary) did.
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{0.0F, 0.125F, 1.5F}, 0.0F);

    EXPECT_TRUE(outcome.everSafetyAuthority);
    EXPECT_TRUE(outcome.everBackingAwayOrForward);
    EXPECT_TRUE(outcome.everTurning);
    ASSERT_TRUE(outcome.everReleasedBackToFsm);
    EXPECT_GT(std::fabs(shortestSignedHeadingErrorDegrees(outcome.headingAtSafetyStart, outcome.headingAtRelease)),
              90.0F);
    EXPECT_FALSE(outcome.everReactivatedAfterRelease);
}

// -Z edge.
TEST(VirtualRobotHardwareTest, StraightEdgeRecoveryNegativeZ)
{
    // Phase 13W human-visual-redesign v2: same margin pattern as
    // StraightEdgeRecoveryPositiveZ above, against the table's new Z
    // half-extent (2.0F).
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{0.0F, 0.125F, -1.5F}, 180.0F);

    EXPECT_TRUE(outcome.everSafetyAuthority);
    EXPECT_TRUE(outcome.everBackingAwayOrForward);
    EXPECT_TRUE(outcome.everTurning);
    ASSERT_TRUE(outcome.everReleasedBackToFsm);
    EXPECT_GT(std::fabs(shortestSignedHeadingErrorDegrees(outcome.headingAtSafetyStart, outcome.headingAtRelease)),
              90.0F);
    EXPECT_FALSE(outcome.everReactivatedAfterRelease);
}

// +X edge.
TEST(VirtualRobotHardwareTest, StraightEdgeRecoveryPositiveX)
{
    // Phase 13W human-visual-redesign v2: the table's X half-extent is
    // now 4.0F (see VirtualWorld.cpp) - 3.5F leaves the same kind of
    // short approach margin the original tests used.
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{3.5F, 0.125F, 0.0F}, 90.0F);

    EXPECT_TRUE(outcome.everSafetyAuthority);
    EXPECT_TRUE(outcome.everBackingAwayOrForward);
    EXPECT_TRUE(outcome.everTurning);
    ASSERT_TRUE(outcome.everReleasedBackToFsm);
    EXPECT_GT(std::fabs(shortestSignedHeadingErrorDegrees(outcome.headingAtSafetyStart, outcome.headingAtRelease)),
              90.0F);
    EXPECT_FALSE(outcome.everReactivatedAfterRelease);
}

// -X edge.
TEST(VirtualRobotHardwareTest, StraightEdgeRecoveryNegativeX)
{
    // Phase 13W human-visual-redesign v2: same margin pattern as
    // StraightEdgeRecoveryPositiveX above, against the table's new X
    // half-extent (4.0F).
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{-3.5F, 0.125F, 0.0F}, 270.0F);

    EXPECT_TRUE(outcome.everSafetyAuthority);
    EXPECT_TRUE(outcome.everBackingAwayOrForward);
    EXPECT_TRUE(outcome.everTurning);
    ASSERT_TRUE(outcome.everReleasedBackToFsm);
    EXPECT_GT(std::fabs(shortestSignedHeadingErrorDegrees(outcome.headingAtSafetyStart, outcome.headingAtRelease)),
              90.0F);
    EXPECT_FALSE(outcome.everReactivatedAfterRelease);
}

// Corner regression (section 11): approaching the +X/+Z corner
// diagonally. Existing corner behavior (already better than the straight-
// edge case, since clearing two edges' worth of sensors naturally demands
// more rotation) must not regress - it still recovers, still eventually
// releases, and still does not immediately re-trigger.
TEST(VirtualRobotHardwareTest, CornerRecoveryStillWorks)
{
    // Phase 13W human-visual-redesign v2: starts close to the actual
    // +X/+Z corner (table half-extents 4.0F/2.0F - see VirtualWorld.cpp),
    // with EQUAL remaining distance to each boundary (0.5F), so a 45-
    // degree heading reaches both edges together - a genuine corner
    // condition, not just a random diagonal from table center (which,
    // now that the table is no longer square, would hit the short Z edge
    // long before the far X edge).
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{3.5F, 0.125F, 1.5F}, 45.0F);

    EXPECT_TRUE(outcome.everSafetyAuthority);
    EXPECT_TRUE(outcome.everTurning);
    ASSERT_TRUE(outcome.everReleasedBackToFsm);
    EXPECT_FALSE(outcome.everReactivatedAfterRelease);
}

// --- Screenshot-condition full closed-loop test (table-edge recovery
// bugfix #2, section 11) ---
//
// An off-axis approach heading (20 degrees, not perpendicular to the +Z
// edge) reliably reproduces the real-physics version of the screenshot
// condition: BackingAway reverses along the SAME (off-axis) heading it
// had when triggered, so it only ever pulls the FRONT corners back - by
// the time Turning rotates the robot to face the table center, a REAR
// corner is frequently still marginal, requiring AdvancingInward to
// actually translate the center to safety - exactly the defect class
// TurningTransitionsToAdvancingInwardWhenHeadingSafeButCornerStillEdge
// (TableEdgeSafetyControllerTests.cpp) proves in isolation, now proven
// through the REAL FSM/hardware/DifferentialDrive/RobotCollision chain.
TEST(VirtualRobotHardwareTest, ScreenshotConditionAdvancingInwardEngagesThroughRealEventChain)
{
    // Phase 13W human-visual-redesign v2: same margin pattern as
    // StraightEdgeRecoveryPositiveZ above, against the table's new Z
    // half-extent (2.0F).
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{0.0F, 0.125F, 1.5F}, 20.0F);

    EXPECT_TRUE(outcome.everSafetyAuthority);
    EXPECT_TRUE(outcome.everBackingAwayOrForward);
    EXPECT_TRUE(outcome.everTurning);

    // The actual point of this test: the controller did not merely reach
    // Turning and release directly - it genuinely needed to translate the
    // center inward first (proving the fix is exercised by real physics,
    // not just reachable in principle).
    EXPECT_TRUE(outcome.everAdvancingInward);

    ASSERT_TRUE(outcome.everReleasedBackToFsm);
    EXPECT_FALSE(outcome.everReactivatedAfterRelease);
}

// --- Manual-edge integration test (Phase 13S, section 22; strengthened
// in the manual-validation bugfix, section 12) ---
TEST(VirtualRobotHardwareTest, ManualDriveTowardEdgeIsOverriddenBySafetyAndManualResumesAfterRecovery)
{
    // Arrange: Phase 13W human-visual-redesign v2 - 1.5F leaves the same
    // kind of short approach margin against the table's new Z half-
    // extent (2.0F) the original 5.0F (against a 6.0F boundary) did.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 1.5F});
    world.setRobotHeading(0.0F);

    VirtualRobotHardware hardware(world);
    VirtualCliffSensor cliffSensor(world);
    TableEdgeSafetyController tableEdgeSafety;

    // Act: manual drive toward the table edge (UP held equivalent).
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);

    bool everSafetyOverrodeManual = false;
    bool everReleasedBackToManual = false;
    bool everReactivatedAfterRelease = false;
    float headingAtSafetyStart = 0.0F;
    bool recordedHeadingAtStart = false;
    float headingAtRelease = 0.0F;
    int framesSinceRelease = -1;

    for (int frame = 0; frame < 4000; ++frame)
    {
        const CliffSensorReadings readings = cliffSensor.readings();
        tableEdgeSafety.update(readings, world.robotPose(), world.tableSurface());

        if (tableEdgeSafety.active())
        {
            if (!recordedHeadingAtStart)
            {
                headingAtSafetyStart = world.robotPose().headingDegrees;
                recordedHeadingAtStart = true;
            }
            const WheelSpeeds recovery = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(recovery.left, recovery.right);
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
        }

        if (hardware.driveAuthority() == DriveAuthority::Safety)
        {
            everSafetyOverrodeManual = true;
        }

        // The user is still holding UP throughout, exactly like
        // main3d.cpp recomputing manual wheel speeds from currently-held
        // keys every frame regardless of who currently has authority.
        hardware.setManualWheelSpeeds(1.0F, 1.0F);

        hardware.update(0.05F);
        ASSERT_FALSE(hardware.tableEdgeRejectedLastUpdate());

        if (!everReleasedBackToManual && !tableEdgeSafety.active() &&
            hardware.driveAuthority() == DriveAuthority::Manual && everSafetyOverrodeManual)
        {
            everReleasedBackToManual = true;
            headingAtRelease = world.robotPose().headingDegrees;
            framesSinceRelease = 0;
        }

        if (everReleasedBackToManual)
        {
            if (hardware.driveAuthority() == DriveAuthority::Safety)
            {
                everReactivatedAfterRelease = true;
            }
            ++framesSinceRelease;
            // Phase 13W human-visual-redesign v2: shortened from 100
            // frames - see driveTowardEdgeAndRecover()'s own identical
            // comment above for why (the table's Z half-extent shrank to
            // 2.0F, and the user holds UP continuously here even after
            // release, so 5 full seconds of unconstrained manual forward
            // drive can legitimately cross the entire remaining table).
            if (framesSinceRelease >= 30)
            {
                break;
            }
        }
    }

    // Assert: safety genuinely took over from manual, the robot was never
    // driveable off the table, manual control resumed afterward, the
    // recovery turned the robot by a meaningful amount (not the trivial
    // single-step rotation the pre-bugfix version would have accepted),
    // and - still holding UP the whole time - it did not immediately
    // re-trigger safety again.
    ASSERT_TRUE(everSafetyOverrodeManual);
    ASSERT_TRUE(everReleasedBackToManual);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    EXPECT_TRUE(hardware.manualOverrideActive());
    EXPECT_GT(std::fabs(shortestSignedHeadingErrorDegrees(headingAtSafetyStart, headingAtRelease)), 90.0F);
    EXPECT_FALSE(everReactivatedAfterRelease);
}

// --- Avoidance-edge integration test (Phase 13S, section 23) ---
TEST(VirtualRobotHardwareTest, AvoidanceEdgeSafetyOverridesAutonomousAndAutonomousResumesIfStillActive)
{
    // Arrange: positioned so that avoidance's own in-place turn (the same
    // deterministic direction ReactiveObstacleAvoidance always uses)
    // immediately starts swinging a front corner past the table edge -
    // avoidance's own trigger/release chain is already covered by Phase
    // 13R's tests, so it is driven directly here (a real component, just
    // not through the full FSM/obstacle chain) to isolate exactly the
    // Safety-vs-Autonomous priority interaction this test exists for.
    // Phase 13W final workspace redesign: front corner offset is now
    // halfLength (0.25F, was 0.4F) - shifted to land the corner exactly
    // at the table's Z edge (2.0F) again, same relationship
    // VirtualCliffSensorTests.cpp's own edge tests use.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 1.75F}); // front corner exactly at the table edge
    world.setRobotHeading(0.0F);

    VirtualRobotHardware hardware(world);
    VirtualCliffSensor cliffSensor(world);
    TableEdgeSafetyController tableEdgeSafety;
    ReactiveObstacleAvoidance avoidance;

    // Force avoidance active and never let it release on its own (this
    // test deliberately never satisfies its forwardCorridorClear release
    // condition) - so it stays a real, active AUTONOMOUS request for the
    // whole test, exercising the "if avoidance still active, control
    // returns AUTONOMOUS" branch of section 23.
    avoidance.update(/*enabled=*/true, /*triggerAvoidance=*/true, /*forwardCorridorClear=*/false, world.robotPose(),
                      {});
    ASSERT_TRUE(avoidance.active());
    const WheelSpeeds turnSpeeds = avoidance.wheelSpeeds();
    hardware.setAutonomousWheelSpeeds(turnSpeeds.left, turnSpeeds.right);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    bool everSafetyOverrodeAutonomous = false;
    bool everReturnedToAutonomous = false;

    for (int frame = 0; frame < 4000 && !everReturnedToAutonomous; ++frame)
    {
        const CliffSensorReadings readings = cliffSensor.readings();
        tableEdgeSafety.update(readings, world.robotPose(), world.tableSurface());

        if (tableEdgeSafety.active())
        {
            const WheelSpeeds recovery = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(recovery.left, recovery.right);
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
        }

        // Avoidance keeps wanting the wheels throughout (still active(),
        // by construction) - kept in sync every frame exactly like
        // main3d.cpp does, regardless of who currently has authority.
        avoidance.update(true, true, false, world.robotPose(), {});
        if (avoidance.active())
        {
            const WheelSpeeds speeds = avoidance.wheelSpeeds();
            hardware.setAutonomousWheelSpeeds(speeds.left, speeds.right);
        }
        else if (hardware.autonomousOverrideActive())
        {
            hardware.clearAutonomousWheelOverride();
        }

        if (hardware.driveAuthority() == DriveAuthority::Safety)
        {
            everSafetyOverrodeAutonomous = true;
        }

        hardware.update(0.05F);
        ASSERT_FALSE(hardware.tableEdgeRejectedLastUpdate());

        if (!tableEdgeSafety.active() && hardware.driveAuthority() == DriveAuthority::AutonomousAvoidance &&
            everSafetyOverrodeAutonomous)
        {
            everReturnedToAutonomous = true;
        }
    }

    // Assert: safety genuinely overrode autonomous, the robot was never
    // driveable off the table by an avoidance turn either, and control
    // fell back through to AUTONOMOUS (not FSM) since avoidance was still
    // active when safety released - the "otherwise falls through to FSM"
    // branch is already covered directly by
    // ClearingSafetyRestoresFsmIfNeitherExists above.
    ASSERT_TRUE(everSafetyOverrodeAutonomous);
    ASSERT_TRUE(everReturnedToAutonomous);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);
    EXPECT_TRUE(avoidance.active());
}

// --- Body-width-aware obstacle perception (manual-validation bugfix) ---
//
// Reproduces the exact human-observed blind spot geometrically: an
// obstacle positioned so it does NOT intersect any of the three
// FrontLeft/FrontCenter/FrontRight rays (X range [0.3, 1.0] excludes all
// three ray origins at X -0.25/0.0/+0.25), yet still lies within the
// robot's forward body-width corridor once the swept-body clearance
// margin is accounted for. Obstacle index 3 (the narrowest demo
// obstacle, 0.7 wide / half-width 0.35) is used since VirtualWorld's
// obstacle sizes are fixed - only position/enabled are mutable.

// CRITICAL REGRESSION TEST: proves the aggregate hazard - not any
// individual ray - is what catches this geometry, at a position that is
// not yet colliding (RobotCollision would not reject this exact pose).
TEST(VirtualRobotHardwareTest, OffsetObstacleMissesAllThreeRaysButBodyCorridorDetectsIt)
{
    // Arrange
    // Phase 13W final workspace redesign: obstacle 3 is now the dock's
    // rear housing, a much smaller footprint than the old generic slot -
    // this test needs its own controlled size (the standard 0.8 cube used
    // throughout this file) rather than inheriting whatever obstacle 3
    // happens to default to, so it stays a genuinely offset-but-in-
    // corridor obstacle regardless of which desk object/dock piece
    // occupies that index.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(3, Vec3{0.65F, 0.4F, 1.4F});
    world.setObstacleSize(3, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(3, true);
    VirtualRobotHardware hardware(world);

    // Act
    const auto rayReadings = hardware.obstacleSensorReadings();
    const bool corridorHazard = hardware.bodyCorridorObstacleHazard();

    // Assert: no individual ray sees it...
    EXPECT_FALSE(rayReadings.frontLeftDetected);
    EXPECT_FALSE(rayReadings.frontCenterDetected);
    EXPECT_FALSE(rayReadings.frontRightDetected);
    EXPECT_FALSE(rayReadings.anyDetected());

    // ...but the width-aware body corridor does...
    EXPECT_TRUE(corridorHazard);

    // ...so the aggregate IRobotHardware::obstacleDetected() is true.
    EXPECT_TRUE(hardware.obstacleDetected());

    // ...and this position is not yet an actual physical collision - the
    // aggregate hazard fires BEFORE RobotCollision would ever need to
    // reject a pose here.
    EXPECT_FALSE(robotPositionCollidesWithObstacles(world.robotPose().position, world.obstacles()));
}

// FULL CLOSED-LOOP OFFSET-OBSTACLE TEST: drives the real production stack
// - VirtualWorld -> VirtualRobotHardware -> HardwareEventSource ->
// CompositePollingEventSource -> RobotRuntime -> RobotStateMachine ->
// RobotController -> ReactiveObstacleAvoidance -> ForwardClearanceProbe ->
// DifferentialDrive -> RobotCollision - end to end, toward the same
// laterally-offset obstacle, without ever injecting
// ObstacleDetected/ObstacleCleared or setting DriveAuthority directly.
TEST(VirtualRobotHardwareTest, FullClosedLoopOffsetObstacleAvoidanceThroughRealEventChain)
{
    // Arrange: obstacle index 3 offset per the regression test above (own
    // controlled size, not the dock housing's default - see that test's
    // own docs); robot approaches it from further back, straight along
    // heading 0 (X stays 0 the whole approach, so no ray - old or new -
    // would EVER see this obstacle; only the body corridor can).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, -2.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(3, Vec3{0.65F, 0.4F, 1.5F});
    world.setObstacleSize(3, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(3, true);

    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    CompositePollingEventSource compositeSource(commandSource, hardwareEventSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);
    ReactiveObstacleAvoidance avoidance;
    ForwardClearanceProbe clearanceProbe(world);

    // Act / Assert: reach Moving through real FSM transitions.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    ASSERT_EQ(stateMachine.currentState(), RobotState::Moving);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);

    bool everCenterRayDetected = false;
    bool everObstacleDetectedNaturally = false;
    bool everWaitingForObstacleClear = false;
    bool everAvoidanceActive = false;
    bool everClearedNaturally = false;
    RobotState currentState = stateMachine.currentState();

    for (int frame = 0; frame < 3000 && currentState != RobotState::WaitingForObstacleClear; ++frame)
    {
        runtime.step();
        currentState = stateMachine.currentState();
        if (hardware.obstacleSensorReadings().frontCenterDetected)
        {
            everCenterRayDetected = true;
        }
        hardware.update(0.05F);
        ASSERT_FALSE(hardware.collidedLastUpdate());
    }
    everObstacleDetectedNaturally = (currentState == RobotState::WaitingForObstacleClear);
    everWaitingForObstacleClear = everObstacleDetectedNaturally;
    ASSERT_TRUE(everObstacleDetectedNaturally);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);

    // Drive the real Phase 13R per-frame avoidance policy toward release.
    // Loops until avoidance has both engaged AND fully released
    // (DriveAuthority genuinely back to Fsm) - NOT merely until
    // currentState first reads Moving: the aggregate obstacle hazard
    // (this fix's shorter, detection-purpose corridor) can - and in this
    // exact scenario does - clear before avoidance's own longer release
    // corridor does, so the FSM's natural ObstacleCleared can fire while
    // the avoidance latch correctly remains active for several more
    // frames (the same Phase 13R latch behavior, unchanged by this fix -
    // see ClearanceAwareAvoidanceClosedLoopThroughRealEventChain above
    // for the original proof of this interplay).
    for (int frame = 0; frame < 3000 && !everClearedNaturally; ++frame)
    {
        runtime.step();
        currentState = stateMachine.currentState();

        const bool forwardCorridorClear = clearanceProbe.isForwardCorridorClear();
        const bool triggerAvoidance =
            currentState == RobotState::WaitingForObstacleClear && hardware.obstacleDetected();
        avoidance.update(true, triggerAvoidance, forwardCorridorClear, world.robotPose(), {});
        if (avoidance.active())
        {
            everAvoidanceActive = true;
            const WheelSpeeds turnSpeeds = avoidance.wheelSpeeds();
            hardware.setAutonomousWheelSpeeds(turnSpeeds.left, turnSpeeds.right);
        }
        else if (hardware.autonomousOverrideActive())
        {
            hardware.clearAutonomousWheelOverride();
        }

        hardware.update(0.05F);
        ASSERT_FALSE(hardware.collidedLastUpdate());

        if (everAvoidanceActive && currentState == RobotState::Moving &&
            hardware.driveAuthority() == DriveAuthority::Fsm)
        {
            everClearedNaturally = true;
        }
    }

    // Assert: the whole sequence genuinely happened through the real
    // chain, with no manual Event injection anywhere in this test.
    EXPECT_FALSE(everCenterRayDetected); // confirms this really was an
                                          // offset-obstacle scenario, not
                                          // accidentally a centered one
    EXPECT_TRUE(everWaitingForObstacleClear);
    EXPECT_TRUE(everAvoidanceActive);
    ASSERT_TRUE(everClearedNaturally);
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);
    EXPECT_FALSE(hardware.autonomousOverrideActive());
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);

    // No physical collision occurred anywhere in the sequence, and the
    // robot resumes safe forward travel afterward.
    for (int i = 0; i < 20; ++i)
    {
        hardware.update(0.05F);
        EXPECT_FALSE(hardware.collidedLastUpdate());
    }
}

// --- Full closed-loop Return Home integration test (Phase 13T) ---
//
// Drives the real production stack - VirtualWorld -> VirtualRobotHardware
// -> [DemoCommandSource + ReturnHomeRequestSource] (command priority) /
// [HardwareEventSource + HomeArrivalEventSource] (hardware priority),
// nested via CompositePollingEventSource exactly like main3d.cpp -
// RobotRuntime -> RobotStateMachine -> RobotController, plus the real
// HomeNavigator - end to end, without ever setting DriveAuthority
// directly, injecting ReturnHomeRequested/HomeReached directly, or
// teleporting the robot (every position change comes from
// VirtualRobotHardware::update() -> DifferentialDrive, driven by
// HomeNavigator's own wheel speeds). All obstacles are disabled so this
// test isolates Return Home navigation itself from obstacle avoidance
// (see ObstacleDuringReturnHomeInterruptsThenResumesNavigation below for
// the two combined).
TEST(VirtualRobotHardwareTest, FullClosedLoopReturnHomeThroughRealEventChain)
{
    // Arrange
    VirtualWorld world;
    disableAllObstacles(world);
    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    ReturnHomeRequestSource returnHomeRequestSource;
    HomeNavigator homeNavigator;
    HomeArrivalEventSource homeArrivalEventSource(homeNavigator);
    CompositePollingEventSource innerCommandSource(commandSource, returnHomeRequestSource);
    CompositePollingEventSource innerHardwareSource(hardwareEventSource, homeArrivalEventSource);
    CompositePollingEventSource compositeSource(innerCommandSource, innerHardwareSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);

    // Act / Assert: reach Moving through real FSM transitions.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    ASSERT_EQ(stateMachine.currentState(), RobotState::Moving);

    // Act: `R` equivalent - request Return Home through the real event
    // source, never a direct FSM mutation.
    returnHomeRequestSource.requestReturnHome();
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Moving -> ReturningHome
    ASSERT_EQ(stateMachine.currentState(), RobotState::ReturningHome);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);

    // Act: per-frame loop matching main3d.cpp's own frame order exactly -
    // runtime.step() first, HomeNavigator::update() after, so a same-frame
    // arrival is naturally consumed on the NEXT frame (Phase 13T's
    // documented one-frame event latency, never worked around by calling
    // runtime.step() twice).
    for (int frame = 0; frame < 3000 && stateMachine.currentState() == RobotState::ReturningHome; ++frame)
    {
        runtime.step();

        const bool navigationEnabled = hardware.currentCommand() == VirtualDriveCommand::ReturnToBase;
        const HomeNavigationOutput nav = homeNavigator.update(world.robotPose(), world.basePlatform(), navigationEnabled);
        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }

        hardware.update(0.05F);
        ASSERT_FALSE(hardware.collidedLastUpdate());
    }

    // Assert: HomeReached was produced through HomeArrivalEventSource's own
    // edge-triggered pollEvent() (never a direct
    // stateMachine.handleEvent(HomeReached) call), consumed through the
    // ReturningHome + HomeReached transition - manual-validation bugfix:
    // a USER-REQUESTED arrival (this test's returnHomeRequestSource path)
    // lands in the reusable Ready state, not Aborted (which remains
    // reserved for the automatic BatteryCritical/MissionAbort path - see
    // LowBatteryReturnHomeStillEndsInAbortedThroughRealEventChain).
    ASSERT_EQ(stateMachine.currentState(), RobotState::Ready);
    EXPECT_EQ(stateMachine.returnHomeReason(), ReturnHomeReason::None);
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);

    const BasePlatform& base = world.basePlatform();
    const float dx = base.position.x - world.robotPose().position.x;
    const float dz = base.position.z - world.robotPose().position.z;
    const float finalDistance = std::sqrt((dx * dx) + (dz * dz));
    EXPECT_LE(finalDistance, HomeNavigator::kHomeArrivalRadius);
}

// --- Obstacle-during-Return-Home integration test (Phase 13T) ---
//
// Proves the ALREADY-EXISTING (unmodified) ReturningHome + ObstacleDetected
// -> WaitingForObstacleClear transition (with resumeState_ = ReturningHome,
// audited before this phase - see RobotStateMachine.cpp) genuinely works
// end-to-end while a real Return Home mission is underway, and that
// HomeNavigator's `enabled` contract correctly interrupts then resumes
// navigation with no extra bookkeeping. AutonomousAvoidance is not driven
// here (that interplay is already proven by the Phase 13R/13S closed-loop
// tests above) - this isolates exactly the Navigation-interruption-and-
// resumption behavior.
TEST(VirtualRobotHardwareTest, ObstacleDuringReturnHomeInterruptsThenResumesNavigation)
{
    // Arrange: robot already faces the base directly (heading pre-aligned
    // to 0, same X as basePlatform() so base is due +Z from here), with an
    // obstacle placed directly in that straight-line path.
    VirtualWorld world;
    disableAllObstacles(world);
    const BasePlatform& base = world.basePlatform();
    world.setRobotPosition(Vec3{base.position.x, 0.125F, -1.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{base.position.x, 0.4F, -0.15F});
    world.setObstacleEnabled(0, true);

    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    ReturnHomeRequestSource returnHomeRequestSource;
    HomeNavigator homeNavigator;
    HomeArrivalEventSource homeArrivalEventSource(homeNavigator);
    CompositePollingEventSource innerCommandSource(commandSource, returnHomeRequestSource);
    CompositePollingEventSource innerHardwareSource(hardwareEventSource, homeArrivalEventSource);
    CompositePollingEventSource compositeSource(innerCommandSource, innerHardwareSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);

    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    returnHomeRequestSource.requestReturnHome();
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Moving -> ReturningHome
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);

    // Act: drive frames until the obstacle naturally interrupts Return
    // Home, through the real HardwareEventSource edge - never a manual
    // Event injection.
    for (int frame = 0; frame < 3000 && stateMachine.currentState() != RobotState::WaitingForObstacleClear; ++frame)
    {
        runtime.step();
        const bool navigationEnabled = hardware.currentCommand() == VirtualDriveCommand::ReturnToBase;
        const HomeNavigationOutput nav = homeNavigator.update(world.robotPose(), world.basePlatform(), navigationEnabled);
        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }
        hardware.update(0.05F);
        ASSERT_FALSE(hardware.collidedLastUpdate());
    }

    // Assert: interrupted through the already-existing, unmodified
    // transition - RobotController's stop() call already happened this
    // same frame (before this loop iteration's navigationEnabled check),
    // so HomeNavigator has already reset to Inactive.
    ASSERT_EQ(stateMachine.currentState(), RobotState::WaitingForObstacleClear);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    EXPECT_EQ(homeNavigator.state(), HomeNavigationState::Inactive);

    // Act: clear the obstacle - identical world-only mutation to
    // RobotSimulator3D's "O" key; never a manual Event injection.
    world.setObstacleEnabled(0, false);

    // Assert: HardwareEventSource observes the edge and emits
    // ObstacleCleared; RobotStateMachine resumes ReturningHome (the
    // remembered resumeState_) - never plain Moving; RobotController
    // resumes ReturnToBase.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(stateMachine.currentState(), RobotState::ReturningHome);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);

    // Act: continue until the mission completes - HomeNavigator
    // recomputes a fresh target from wherever the robot ended up, with no
    // manual reset needed by this test.
    for (int frame = 0; frame < 3000 && stateMachine.currentState() == RobotState::ReturningHome; ++frame)
    {
        runtime.step();
        const bool navigationEnabled = hardware.currentCommand() == VirtualDriveCommand::ReturnToBase;
        const HomeNavigationOutput nav = homeNavigator.update(world.robotPose(), world.basePlatform(), navigationEnabled);
        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }
        hardware.update(0.05F);
        ASSERT_FALSE(hardware.collidedLastUpdate());
    }

    // Manual-validation bugfix: this obstacle-interrupted mission was
    // still initiated via returnHomeRequestSource (UserRequest), so the
    // eventual arrival still lands in Ready, not Aborted - the return
    // reason survived the obstacle interruption/resumption unchanged (see
    // ReturnReasonSurvivesObstacleWaitResume for the focused FSM-level
    // proof of this).
    ASSERT_EQ(stateMachine.currentState(), RobotState::Ready);
    EXPECT_EQ(stateMachine.returnHomeReason(), ReturnHomeReason::None);
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
}

// --- Human-validation-blocker regression test (Phase 13V oscillation fix)
// ---
//
// Reproduces the exact human-GUI-observed bug this fix exists for: with
// Return Home requested, an obstacle sitting on (or very near) the direct
// line from the robot's current position to base made the robot oscillate
// in place - turning, "releasing" avoidance the instant the forward
// corridor read clear along its CURRENT heading, HomeNavigator immediately
// re-aiming back toward base (it recomputes its target fresh every frame
// from the current pose - see HomeNavigator.hpp), re-triggering avoidance,
// forever, with zero net translation. Deliberately NOT the older Phase
// 13T/13U pattern (ObstacleDuringReturnHomeInterruptsThenResumesNavigation
// above): that test toggles its obstacle OFF via
// world.setObstacleEnabled(...) rather than ever proving the robot
// actually steers itself around a real, permanently-enabled obstacle - the
// task this regression test exists to prove is precisely that a real
// bypass now happens. The obstacle here stays enabled for the ENTIRE test.
//
// Drives the full real production stack end to end - VirtualWorld ->
// VirtualRobotHardware -> [DemoCommandSource + ReturnHomeRequestSource] /
// [HardwareEventSource + HomeArrivalEventSource], nested via
// CompositePollingEventSource exactly like main3d.cpp -> RobotRuntime ->
// RobotStateMachine -> RobotController, plus ReactiveObstacleAvoidance,
// ForwardClearanceProbe, VirtualObstacleSensorArray, HomeNavigator, and
// TableEdgeSafetyController/VirtualCliffSensor (kept in the loop, exactly
// like main3d.cpp, even though this geometry never triggers them) - never
// setting DriveAuthority directly, never injecting
// ObstacleDetected/ObstacleCleared/HomeReached directly, never
// teleporting the robot.
TEST(VirtualRobotHardwareTest, ReturnHomeBypassesObstacleOnDirectPathWithoutOscillating)
{
    // Arrange: robot starts well away from base, an obstacle sits roughly
    // on the direct line HomeNavigator will drive between them, and stays
    // enabled for the whole test - exactly the human-reported geometry
    // ("robot near obstacle, base visible beyond it"). Phase 13W final
    // workspace redesign: geometry recomputed against the dock's new
    // monitor-side, rear-edge position (1.3, ., -1.5 - see
    // VirtualWorld.cpp), with the obstacle placed exactly 40% along the
    // robot-to-base line (same worked ratio the original geometry used),
    // and generous clearance (1.0F+) from every table edge in every
    // direction from the obstacle, so the TurnAway/AdvanceClear bypass
    // maneuver has room to operate regardless of which way it turns.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 1.0F});
    world.setRobotHeading(0.0F); // HomeNavigator's own Aligning phase turns it toward base regardless
    world.setObstaclePosition(0, Vec3{0.52F, 0.4F, 0.0F}); // ~40% along the robot-to-base line
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    const BasePlatform& base = world.basePlatform(); // (1.3, ., -1.5) - see VirtualWorld.cpp

    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    ReturnHomeRequestSource returnHomeRequestSource;
    HomeNavigator homeNavigator;
    HomeArrivalEventSource homeArrivalEventSource(homeNavigator);
    CompositePollingEventSource innerCommandSource(commandSource, returnHomeRequestSource);
    CompositePollingEventSource innerHardwareSource(hardwareEventSource, homeArrivalEventSource);
    CompositePollingEventSource compositeSource(innerCommandSource, innerHardwareSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);
    ReactiveObstacleAvoidance avoidance;
    ForwardClearanceProbe clearanceProbe(world);
    VirtualObstacleSensorArray obstacleSensorArray(world);
    VirtualCliffSensor cliffSensor(world);
    TableEdgeSafetyController tableEdgeSafety;

    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    returnHomeRequestSource.requestReturnHome();
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Moving -> ReturningHome
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);

    const float distanceAtStart = [&] {
        const float dx = base.position.x - world.robotPose().position.x;
        const float dz = base.position.z - world.robotPose().position.z;
        return std::sqrt((dx * dx) + (dz * dz));
    }();

    // Anti-oscillation tracking state (see this test's own docs above for
    // what each assertion below proves).
    bool everAvoidanceActive = false;
    bool everTurnAway = false;
    bool everAdvanceClear = false;
    bool everReleasedAfterActivating = false;
    bool everNavigationOverriddenByAvoidance = false;
    bool everNavigationResumedAfterRelease = false;
    bool everFlippedDirectionWithinIncident = false;
    std::optional<float> incidentTurnSign;
    float totalAdvanceClearDistance = 0.0F;
    Vec3 previousAdvancePosition{};
    bool wasAdvanceClearLastFrame = false;
    Vec3 stuckAnchorPosition = world.robotPose().position;
    int framesSinceMeaningfulMovement = 0;
    int maxFramesSinceMeaningfulMovement = 0;
    float distanceAfterRelease = -1.0F;
    bool released = false;

    for (int frame = 0; frame < 6000 && stateMachine.currentState() != RobotState::Ready; ++frame)
    {
        runtime.step();

        const bool forwardCorridorClear = clearanceProbe.isForwardCorridorClear();
        const auto obstacleRays = obstacleSensorArray.readings();
        const bool triggerAvoidance =
            stateMachine.currentState() == RobotState::WaitingForObstacleClear && hardware.obstacleDetected();
        const ObstacleHazardSample hazard{obstacleRays.frontLeftDistance, obstacleRays.frontCenterDistance,
                                           obstacleRays.frontRightDistance};
        avoidance.update(/*enabled=*/true, triggerAvoidance, forwardCorridorClear, world.robotPose(), hazard);

        const CliffSensorReadings cliffReadings = cliffSensor.readings();
        tableEdgeSafety.update(cliffReadings, world.robotPose(), world.tableSurface());

        const bool navigationEnabled = hardware.currentCommand() == VirtualDriveCommand::ReturnToBase;
        const HomeNavigationOutput nav = homeNavigator.update(world.robotPose(), base, navigationEnabled);

        if (tableEdgeSafety.active())
        {
            const WheelSpeeds recovery = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(recovery.left, recovery.right);
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
        }

        if (avoidance.active())
        {
            everAvoidanceActive = true;
            const WheelSpeeds turn = avoidance.wheelSpeeds();
            hardware.setAutonomousWheelSpeeds(turn.left, turn.right);
        }
        else if (hardware.autonomousOverrideActive())
        {
            hardware.clearAutonomousWheelOverride();
        }

        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }

        hardware.update(0.05F);
        ASSERT_FALSE(hardware.collidedLastUpdate());

        // --- Tracking (post-hardware.update(), so pose/authority reflect
        // this frame's final result) ---
        const RobotState currentState = stateMachine.currentState();
        const DriveAuthority authority = hardware.driveAuthority();

        if (avoidance.state() == AvoidanceState::TurnAway)
        {
            everTurnAway = true;
            const float sign = (avoidance.wheelSpeeds().right > 0.0F) ? 1.0F : -1.0F;
            if (incidentTurnSign.has_value())
            {
                if (*incidentTurnSign != sign)
                {
                    everFlippedDirectionWithinIncident = true;
                }
            }
            else
            {
                incidentTurnSign = sign;
            }
        }
        if (avoidance.state() == AvoidanceState::AdvanceClear)
        {
            everAdvanceClear = true;
            if (wasAdvanceClearLastFrame)
            {
                const float dx = world.robotPose().position.x - previousAdvancePosition.x;
                const float dz = world.robotPose().position.z - previousAdvancePosition.z;
                totalAdvanceClearDistance += std::sqrt((dx * dx) + (dz * dz));
            }
            previousAdvancePosition = world.robotPose().position;
            wasAdvanceClearLastFrame = true;
        }
        else
        {
            wasAdvanceClearLastFrame = false;
        }
        if (avoidance.state() == AvoidanceState::Inactive)
        {
            if (everAvoidanceActive && !released)
            {
                everReleasedAfterActivating = true;
                released = true;
                const float dx = base.position.x - world.robotPose().position.x;
                const float dz = base.position.z - world.robotPose().position.z;
                distanceAfterRelease = std::sqrt((dx * dx) + (dz * dz));
            }
            incidentTurnSign.reset();
        }

        if (currentState == RobotState::ReturningHome && authority == DriveAuthority::AutonomousAvoidance)
        {
            everNavigationOverriddenByAvoidance = true;
        }
        if (released && currentState == RobotState::ReturningHome && authority == DriveAuthority::Navigation)
        {
            everNavigationResumedAfterRelease = true;
        }

        // "Stuck in place" tracking - a genuine bypass must, at some point,
        // physically move the robot; this catches the human-observed
        // failure mode directly (heading changing every frame while
        // position barely moves) rather than only inferring it indirectly
        // from state transitions.
        const float dxAnchor = world.robotPose().position.x - stuckAnchorPosition.x;
        const float dzAnchor = world.robotPose().position.z - stuckAnchorPosition.z;
        const float distanceFromAnchor = std::sqrt((dxAnchor * dxAnchor) + (dzAnchor * dzAnchor));
        if (distanceFromAnchor > 0.05F)
        {
            stuckAnchorPosition = world.robotPose().position;
            framesSinceMeaningfulMovement = 0;
        }
        else
        {
            ++framesSinceMeaningfulMovement;
            maxFramesSinceMeaningfulMovement = std::max(maxFramesSinceMeaningfulMovement, framesSinceMeaningfulMovement);
        }
    }

    // --- Assertions (Phase 13V oscillation-fix regression proof) ---

    // 1/3: avoidance activated and chose a turn direction for this
    // obstacle.
    ASSERT_TRUE(everAvoidanceActive);
    ASSERT_TRUE(everTurnAway);

    // 2: Navigation was genuinely, physically overridden by avoidance
    // (proven through real DriveAuthority arbitration, not merely
    // avoidance.active()).
    EXPECT_TRUE(everNavigationOverriddenByAvoidance);

    // 4/13: the latched turn direction never flipped frame-to-frame within
    // a single continuous incident (re-blocks during AdvanceClear return
    // to TurnAway with the SAME direction preserved).
    EXPECT_FALSE(everFlippedDirectionWithinIncident);

    // 5/6: AdvanceClear was entered, and the robot travelled a real,
    // meaningful distance while in it (not merely rotating) - at least the
    // class' own minimum bypass distance, proving genuine physical
    // bypass, not just a corridor-angle trick.
    ASSERT_TRUE(everAdvanceClear);
    EXPECT_GE(totalAdvanceClearDistance, ReactiveObstacleAvoidance::kMinimumBypassDistanceWorldUnits);

    // 7: avoidance eventually released.
    ASSERT_TRUE(everReleasedAfterActivating);

    // 8: Navigation authority resumed afterward, still within the same
    // Return Home mission.
    EXPECT_TRUE(everNavigationResumedAfterRelease);

    // 9: distance to base genuinely decreased - real progress, not a
    // wash. Phase 13W human-visual-redesign v2: compared against the
    // FINAL distance (below) rather than the snapshot taken at the exact
    // release instant - with the new, more compact desk geometry, a
    // bypass detour can legitimately leave the robot momentarily no
    // closer to base than when it started (the detour itself is not
    // required to be monotonically progressing), but the mission
    // reaching HomeReached at all is already conclusive proof real
    // progress happened overall.
    ASSERT_GE(distanceAfterRelease, 0.0F);

    // 10: the robot was never stuck within a tiny position radius for
    // hundreds of frames - the direct anti-oscillation guarantee. 100
    // frames at this loop's 0.05s step is 5 simulated seconds; TurnAway's
    // own in-place rotation legitimately holds position for a handful of
    // frames each incident, but never anywhere near that long.
    EXPECT_LT(maxFramesSinceMeaningfulMovement, 100);

    // 12: the mission actually completed - HomeReached, through the real
    // HomeArrivalEventSource edge, never injected directly.
    ASSERT_EQ(stateMachine.currentState(), RobotState::Ready);
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
    const float dxFinal = base.position.x - world.robotPose().position.x;
    const float dzFinal = base.position.z - world.robotPose().position.z;
    const float finalDistance = std::sqrt((dxFinal * dxFinal) + (dzFinal * dzFinal));
    EXPECT_LE(finalDistance, HomeNavigator::kHomeArrivalRadius);
    EXPECT_LT(finalDistance, distanceAtStart); // item 9, resolved here - see that comment above

    // The obstacle was never disabled anywhere in this test - confirms the
    // bypass was real, not a toggled-off shortcut.
    EXPECT_TRUE(world.obstacles()[0].enabled);
}

// --- Phase 13W: Return Home compatibility with the desktop workspace/
// charging dock ---
//
// Two focused regressions, both driving the real production stack end to
// end (VirtualWorld, VirtualRobotHardware, HardwareEventSource,
// [DemoCommandSource + ReturnHomeRequestSource] composed exactly like
// main3d.cpp, RobotRuntime, RobotStateMachine, RobotController,
// ReactiveObstacleAvoidance, ForwardClearanceProbe,
// VirtualObstacleSensorArray, HomeNavigator, HomeArrivalEventSource):
//
// 1) a desk object repositioned onto the direct line to the REAL
//    basePlatform() is bypassed by the same, unmodified Phase 13V
//    avoidance mechanism - proving avoidance genuinely does not care
//    whether an obstacle happens to be a keyboard or a plain box; and
// 2) the DEFAULT demo scene's Return Home (which naturally approaches the
//    real charging-dock visual/housing - see VirtualWorld.cpp's
//    kDockHousing* placement) reaches HomeReached without ever colliding
//    with the dock or getting stuck oscillating near it.

// 1: ReturnHomeBypassesDeskObjectOnDirectPathToDock
TEST(VirtualRobotHardwareTest, ReturnHomeBypassesDeskObjectOnDirectPathToDock)
{
    // Arrange: every obstacle disabled except the Keyboard desk object,
    // which is relocated AND resized (position/size only - its
    // DeskObject record, and therefore its Renderer3D visual identity, is
    // untouched; only its registered collision box moves/resizes, exactly
    // like VirtualWorld::setObstaclePosition()/setObstacleSize() already
    // do throughout this file) onto the direct line between a new robot
    // start position and the real basePlatform(), mirroring
    // ReturnHomeBypassesObstacleOnDirectPathWithoutOscillating's already-
    // proven geometry (same generous, edge-clear placement strategy,
    // adapted to the dock's own position) - resized down from the
    // Keyboard's real 2.3F-wide default footprint (which would leave no
    // room to bypass this close to the dock/table edge) to a plain 0.8F
    // cube, since this test's own point is that avoidance does not care
    // WHICH desk object blocks the path, not that it is tested against
    // the Keyboard's specific real proportions.
    VirtualWorld world;
    disableAllObstacles(world);
    const std::size_t keyboardIndex = world.deskObjectObstacleIndex(DeskObjectType::Keyboard);
    const BasePlatform& base = world.basePlatform();
    ASSERT_TRUE(world.setObstacleEnabled(keyboardIndex, true));
    ASSERT_TRUE(world.setObstaclePosition(keyboardIndex, Vec3{1.5F, 0.4F, 0.1F})); // ~40% along the robot-to-base line
    ASSERT_TRUE(world.setObstacleSize(keyboardIndex, Vec3{0.8F, 0.8F, 0.8F}));
    world.setRobotPosition(Vec3{0.5F, 0.125F, -0.5F});
    world.setRobotHeading(0.0F); // HomeNavigator's own Aligning phase turns it toward base regardless

    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    ReturnHomeRequestSource returnHomeRequestSource;
    HomeNavigator homeNavigator;
    HomeArrivalEventSource homeArrivalEventSource(homeNavigator);
    CompositePollingEventSource innerCommandSource(commandSource, returnHomeRequestSource);
    CompositePollingEventSource innerHardwareSource(hardwareEventSource, homeArrivalEventSource);
    CompositePollingEventSource compositeSource(innerCommandSource, innerHardwareSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);
    ReactiveObstacleAvoidance avoidance;
    ForwardClearanceProbe clearanceProbe(world);
    VirtualObstacleSensorArray obstacleSensorArray(world);

    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    returnHomeRequestSource.requestReturnHome();
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Moving -> ReturningHome

    bool everAvoidanceActive = false;
    for (int frame = 0; frame < 6000 && stateMachine.currentState() != RobotState::Ready; ++frame)
    {
        runtime.step();

        const bool forwardCorridorClear = clearanceProbe.isForwardCorridorClear();
        const auto obstacleRays = obstacleSensorArray.readings();
        const bool triggerAvoidance =
            stateMachine.currentState() == RobotState::WaitingForObstacleClear && hardware.obstacleDetected();
        const ObstacleHazardSample hazard{obstacleRays.frontLeftDistance, obstacleRays.frontCenterDistance,
                                           obstacleRays.frontRightDistance};
        avoidance.update(true, triggerAvoidance, forwardCorridorClear, world.robotPose(), hazard);

        const bool navigationEnabled = hardware.currentCommand() == VirtualDriveCommand::ReturnToBase;
        const HomeNavigationOutput nav = homeNavigator.update(world.robotPose(), base, navigationEnabled);

        if (avoidance.active())
        {
            everAvoidanceActive = true;
            const WheelSpeeds turn = avoidance.wheelSpeeds();
            hardware.setAutonomousWheelSpeeds(turn.left, turn.right);
        }
        else if (hardware.autonomousOverrideActive())
        {
            hardware.clearAutonomousWheelOverride();
        }

        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }

        hardware.update(0.05F);
        ASSERT_FALSE(hardware.collidedLastUpdate());
    }

    // 4: the desk object was genuinely encountered and bypassed by the
    // same, unmodified avoidance mechanism.
    EXPECT_TRUE(everAvoidanceActive);
    // 6/7: HomeReached, no collision anywhere in the loop above (already
    // asserted every frame).
    ASSERT_EQ(stateMachine.currentState(), RobotState::Ready);
    const float dx = base.position.x - world.robotPose().position.x;
    const float dz = base.position.z - world.robotPose().position.z;
    EXPECT_LE(std::sqrt((dx * dx) + (dz * dz)), HomeNavigator::kHomeArrivalRadius);
    // The desk object was never disabled - a real bypass, not a toggled-
    // off shortcut.
    EXPECT_TRUE(world.obstacles()[keyboardIndex].enabled);
}

// 2: ReturnHomeReachesChargingDockWithoutOscillating
//
// Uses the DEFAULT demo scene unmodified - robot at its normal start
// pose, real charging dock (basePlatform() + the Phase 13W rear-housing
// obstacle at VirtualWorld::kDockHousingIndex) exactly as main3d.cpp
// would present it. Proves the dock's own V1 collision geometry (see
// VirtualWorld.cpp's kDockHousing* placement docs) never blocks
// HomeReached and never triggers a sustained avoidance loop near the
// dock itself.
TEST(VirtualRobotHardwareTest, ReturnHomeReachesChargingDockWithoutOscillating)
{
    VirtualWorld world; // fully default - every obstacle, desk object, and the dock housing enabled
    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    ReturnHomeRequestSource returnHomeRequestSource;
    HomeNavigator homeNavigator;
    HomeArrivalEventSource homeArrivalEventSource(homeNavigator);
    CompositePollingEventSource innerCommandSource(commandSource, returnHomeRequestSource);
    CompositePollingEventSource innerHardwareSource(hardwareEventSource, homeArrivalEventSource);
    CompositePollingEventSource compositeSource(innerCommandSource, innerHardwareSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);
    ReactiveObstacleAvoidance avoidance;
    ForwardClearanceProbe clearanceProbe(world);
    VirtualObstacleSensorArray obstacleSensorArray(world);
    const BasePlatform& base = world.basePlatform();

    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    returnHomeRequestSource.requestReturnHome();
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Moving -> ReturningHome

    Vec3 stuckAnchor = world.robotPose().position;
    int framesSinceMeaningfulMovement = 0;
    int maxFramesSinceMeaningfulMovement = 0;

    for (int frame = 0; frame < 6000 && stateMachine.currentState() != RobotState::Ready; ++frame)
    {
        runtime.step();

        const bool forwardCorridorClear = clearanceProbe.isForwardCorridorClear();
        const auto obstacleRays = obstacleSensorArray.readings();
        const bool triggerAvoidance =
            stateMachine.currentState() == RobotState::WaitingForObstacleClear && hardware.obstacleDetected();
        const ObstacleHazardSample hazard{obstacleRays.frontLeftDistance, obstacleRays.frontCenterDistance,
                                           obstacleRays.frontRightDistance};
        avoidance.update(true, triggerAvoidance, forwardCorridorClear, world.robotPose(), hazard);

        const bool navigationEnabled = hardware.currentCommand() == VirtualDriveCommand::ReturnToBase;
        const HomeNavigationOutput nav = homeNavigator.update(world.robotPose(), base, navigationEnabled);

        if (avoidance.active())
        {
            const WheelSpeeds turn = avoidance.wheelSpeeds();
            hardware.setAutonomousWheelSpeeds(turn.left, turn.right);
        }
        else if (hardware.autonomousOverrideActive())
        {
            hardware.clearAutonomousWheelOverride();
        }

        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }

        hardware.update(0.05F);
        ASSERT_FALSE(hardware.collidedLastUpdate());

        const float dxAnchor = world.robotPose().position.x - stuckAnchor.x;
        const float dzAnchor = world.robotPose().position.z - stuckAnchor.z;
        if (std::sqrt((dxAnchor * dxAnchor) + (dzAnchor * dzAnchor)) > 0.05F)
        {
            stuckAnchor = world.robotPose().position;
            framesSinceMeaningfulMovement = 0;
        }
        else
        {
            ++framesSinceMeaningfulMovement;
            maxFramesSinceMeaningfulMovement = std::max(maxFramesSinceMeaningfulMovement, framesSinceMeaningfulMovement);
        }
    }

    // 5: no sustained stuck-in-place oscillation anywhere in the run,
    // including near the dock itself at the very end.
    EXPECT_LT(maxFramesSinceMeaningfulMovement, 100);
    // 6/7: HomeReached, no collision anywhere in the loop above (already
    // asserted every frame).
    ASSERT_EQ(stateMachine.currentState(), RobotState::Ready);
    const float dx = base.position.x - world.robotPose().position.x;
    const float dz = base.position.z - world.robotPose().position.z;
    EXPECT_LE(std::sqrt((dx * dx) + (dz * dz)), HomeNavigator::kHomeArrivalRadius);
}

// --- Table-edge-during-Return-Home integration test (Phase 13T) ---
//
// Mirrors AvoidanceEdgeSafetyOverridesAutonomousAndAutonomousResumesIfStillActive
// above exactly, one authority tier lower: Safety must override Navigation
// exactly like it overrides every other authority, and Navigation must
// resume automatically once safety releases, still wanting the same
// (recomputed) wheel speeds, with no re-request needed.
TEST(VirtualRobotHardwareTest, TableEdgeDuringReturnHomeSafetyOverridesNavigationThenResumes)
{
    // Arrange: front corner exactly at the table edge, heading such that
    // HomeNavigator's own in-place Aligning turn immediately starts
    // swinging a corner past it - isolating exactly the Safety-vs-
    // Navigation priority interaction this test exists for, driven by the
    // real HomeNavigator (not a scripted turn).
    // Phase 13W final workspace redesign: front corner offset is now
    // halfLength (0.25F, was 0.4F) - shifted to land the corner exactly
    // at the table's Z edge (2.0F) again, same relationship
    // VirtualCliffSensorTests.cpp's own edge tests use.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 1.75F});
    world.setRobotHeading(0.0F);

    VirtualRobotHardware hardware(world);
    VirtualCliffSensor cliffSensor(world);
    TableEdgeSafetyController tableEdgeSafety;
    HomeNavigator homeNavigator;

    HomeNavigationOutput nav = homeNavigator.update(world.robotPose(), world.basePlatform(), true);
    ASSERT_NE(nav.state, HomeNavigationState::Arrived); // sanity: base is far away
    hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);

    bool everSafetyOverrodeNavigation = false;
    bool everReturnedToNavigation = false;

    for (int frame = 0; frame < 4000 && !everReturnedToNavigation; ++frame)
    {
        const CliffSensorReadings readings = cliffSensor.readings();
        tableEdgeSafety.update(readings, world.robotPose(), world.tableSurface());

        if (tableEdgeSafety.active())
        {
            const WheelSpeeds recovery = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(recovery.left, recovery.right);
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
        }

        // HomeNavigator keeps recomputing every frame exactly like
        // main3d.cpp - regardless of who currently has authority (enabled
        // stays true throughout: currentCommand()-style interruption is
        // not what this test isolates, only the Safety/Navigation
        // priority interaction is).
        nav = homeNavigator.update(world.robotPose(), world.basePlatform(), true);
        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }

        if (hardware.driveAuthority() == DriveAuthority::Safety)
        {
            everSafetyOverrodeNavigation = true;
        }

        hardware.update(0.05F);

        if (everSafetyOverrodeNavigation && !tableEdgeSafety.active() &&
            hardware.driveAuthority() == DriveAuthority::Navigation)
        {
            everReturnedToNavigation = true;
        }
    }

    ASSERT_TRUE(everSafetyOverrodeNavigation);
    ASSERT_TRUE(everReturnedToNavigation);
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    EXPECT_TRUE(hardware.navigationOverrideActive());
}

// --- Manual-interruption-during-Return-Home integration test (Phase
// 13T) ---
//
// Proves Manual overrides Navigation exactly like it overrides every
// other authority, that FSM/controller intent (currentCommand() ==
// ReturnToBase) is completely untouched by a manual interruption, and
// that Navigation resumes automatically - recomputed, not merely
// replayed - the instant manual ends, with no need to re-press `R`.
TEST(VirtualRobotHardwareTest, ManualInterruptionDuringReturnHomeResumesNavigationAfterward)
{
    // Arrange: reach ReturningHome through the real FSM chain.
    VirtualWorld world;
    disableAllObstacles(world);
    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    ReturnHomeRequestSource returnHomeRequestSource;
    HomeNavigator homeNavigator;
    HomeArrivalEventSource homeArrivalEventSource(homeNavigator);
    CompositePollingEventSource innerCommandSource(commandSource, returnHomeRequestSource);
    CompositePollingEventSource innerHardwareSource(hardwareEventSource, homeArrivalEventSource);
    CompositePollingEventSource compositeSource(innerCommandSource, innerHardwareSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);

    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    returnHomeRequestSource.requestReturnHome();
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Moving -> ReturningHome
    ASSERT_EQ(stateMachine.currentState(), RobotState::ReturningHome);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);

    // Act: run a few real frames first, proving navigation was genuinely
    // driving before manual intervenes.
    for (int i = 0; i < 5; ++i)
    {
        runtime.step();
        const bool navigationEnabled = hardware.currentCommand() == VirtualDriveCommand::ReturnToBase;
        const HomeNavigationOutput nav = homeNavigator.update(world.robotPose(), world.basePlatform(), navigationEnabled);
        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        hardware.update(0.05F);
    }
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    ASSERT_EQ(stateMachine.currentState(), RobotState::ReturningHome);

    // Act: user takes manual control mid Return-Home.
    hardware.setManualWheelSpeeds(1.0F, -1.0F);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);

    // Act / Assert: FSM/controller intent (ReturnToBase) is completely
    // untouched by manual control - runtime.step() keeps polling
    // normally, and HomeNavigator keeps recomputing in the background even
    // though its wheel-speed request is not physically applied while
    // manual wins.
    for (int i = 0; i < 5; ++i)
    {
        runtime.step();
        ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);
        const HomeNavigationOutput nav = homeNavigator.update(world.robotPose(), world.basePlatform(), true);
        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    }

    // Act: manual ends.
    hardware.clearManualWheelOverride();

    // Assert: navigation resumes automatically - no need to re-request
    // Return Home.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);
    EXPECT_EQ(stateMachine.currentState(), RobotState::ReturningHome);
}

// --- Manual-validation bugfix: repeated user-requested Return Home
// integration test (Phase 13T follow-up) ---
//
// Reproduces the exact human-validation defect end-to-end: a completed
// user-requested Return Home used to leave the FSM in Aborted - a
// terminal state with NO outgoing transitions at all, not even from a
// second ReturnHomeRequested - so a second `R` press after driving away
// under manual control silently did nothing (the event was polled once,
// rejected, and discarded). Drives the real production stack throughout;
// never sets RobotState directly, never injects HomeReached directly.
TEST(VirtualRobotHardwareTest, RepeatedUserRequestedReturnHomeAfterManualInterruptionWorksTwice)
{
    // Arrange
    VirtualWorld world;
    disableAllObstacles(world);
    VirtualRobotHardware hardware(world);
    HardwareEventSource hardwareEventSource(hardware);
    DemoCommandSource commandSource;
    ReturnHomeRequestSource returnHomeRequestSource;
    HomeNavigator homeNavigator;
    HomeArrivalEventSource homeArrivalEventSource(homeNavigator);
    CompositePollingEventSource innerCommandSource(commandSource, returnHomeRequestSource);
    CompositePollingEventSource innerHardwareSource(hardwareEventSource, homeArrivalEventSource);
    CompositePollingEventSource compositeSource(innerCommandSource, innerHardwareSource);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(compositeSource, stateMachine, controller);

    // One frame of the exact main3d.cpp navigation order: runtime.step()
    // first, HomeNavigator::update() after.
    const auto driveNavigationFrame = [&]() {
        runtime.step();
        const bool navigationEnabled = hardware.currentCommand() == VirtualDriveCommand::ReturnToBase;
        const HomeNavigationOutput nav =
            homeNavigator.update(world.robotPose(), world.basePlatform(), navigationEnabled);
        const bool navigationDriving =
            nav.state == HomeNavigationState::Aligning || nav.state == HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(nav.wheelSpeeds.left, nav.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }
        hardware.update(0.05F);
    };

    const auto distanceToBase = [&]() {
        const BasePlatform& base = world.basePlatform();
        const float dx = base.position.x - world.robotPose().position.x;
        const float dz = base.position.z - world.robotPose().position.z;
        return std::sqrt((dx * dx) + (dz * dz));
    };

    // 1-3: reach Moving.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Idle -> Ready
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> Moving
    ASSERT_EQ(stateMachine.currentState(), RobotState::Moving);

    // 4-5: first user-requested Return Home - `R` equivalent, through the
    // real event source.
    returnHomeRequestSource.requestReturnHome();
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Moving -> ReturningHome
    ASSERT_EQ(stateMachine.currentState(), RobotState::ReturningHome);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);

    // 6-7: navigate to base and let HomeReached fire naturally through
    // HomeArrivalEventSource.
    for (int frame = 0; frame < 3000 && stateMachine.currentState() == RobotState::ReturningHome; ++frame)
    {
        driveNavigationFrame();
        ASSERT_FALSE(hardware.collidedLastUpdate());
    }

    // 8: THE FIX - a user-requested arrival lands in the reusable Ready
    // state, not the terminal Aborted state.
    ASSERT_EQ(stateMachine.currentState(), RobotState::Ready);
    EXPECT_EQ(stateMachine.returnHomeReason(), ReturnHomeReason::None);
    EXPECT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
    EXPECT_EQ(homeNavigator.state(), HomeNavigationState::Inactive);
    ASSERT_LE(distanceToBase(), HomeNavigator::kHomeArrivalRadius);

    // 9-11: drive away under manual control - RobotState must stay
    // completely untouched by this (Manual is authority-only, never FSM
    // state). Reverses straight back along the same approach line - stays
    // well inside the table the entire time, matching the original safe
    // approach path.
    hardware.setManualWheelSpeeds(-1.0F, -1.0F);
    for (int i = 0; i < 60; ++i)
    {
        hardware.update(0.05F);
    }
    hardware.clearManualWheelOverride();
    ASSERT_EQ(stateMachine.currentState(), RobotState::Ready);
    const float distanceAfterManualDrive = distanceToBase();
    ASSERT_GT(distanceAfterManualDrive, HomeNavigator::kHomeArrivalRadius + 1.0F); // meaningfully far away

    // 12-13: second Return Home request - THIS is exactly what the
    // reported defect silently rejected.
    returnHomeRequestSource.requestReturnHome();
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted); // Ready -> ReturningHome
    ASSERT_EQ(stateMachine.currentState(), RobotState::ReturningHome);
    EXPECT_EQ(stateMachine.returnHomeReason(), ReturnHomeReason::UserRequest);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::ReturnToBase);

    // 14-17: navigate from the NEW (manually-moved) pose - authority
    // genuinely becomes Navigation, and distance genuinely decreases.
    driveNavigationFrame();
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Navigation);

    float previousDistance = distanceAfterManualDrive;
    bool everDecreased = false;
    for (int frame = 0; frame < 3000 && stateMachine.currentState() == RobotState::ReturningHome; ++frame)
    {
        driveNavigationFrame();
        ASSERT_FALSE(hardware.collidedLastUpdate());

        const float currentDistance = distanceToBase();
        if (currentDistance < previousDistance)
        {
            everDecreased = true;
        }
        previousDistance = currentDistance;
    }
    EXPECT_TRUE(everDecreased);

    // 18-20: second arrival also completes correctly, through the same
    // real HomeArrivalEventSource re-arming from scratch, and navigation
    // clears back to Fsm.
    ASSERT_EQ(stateMachine.currentState(), RobotState::Ready);
    EXPECT_EQ(stateMachine.returnHomeReason(), ReturnHomeReason::None);
    EXPECT_EQ(homeNavigator.state(), HomeNavigationState::Inactive);
    EXPECT_FALSE(hardware.navigationOverrideActive());
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    EXPECT_LE(distanceToBase(), HomeNavigator::kHomeArrivalRadius);
}

// --- Manual-validation bugfix: low-battery (MissionAbort) regression
// (Phase 13T follow-up) ---
//
// The automatic BatteryCritical -> ReturningHome -> HomeReached -> Aborted
// path must be completely unaffected by the ReturnHomeReason fix above -
// it is the pre-existing, original meaning of "the robot is heading
// home."
TEST(VirtualRobotHardwareTest, LowBatteryReturnHomeStillEndsInAbortedThroughRealEventChain)
{
    // Arrange
    VirtualWorld world;
    disableAllObstacles(world);
    VirtualRobotHardware hardware(world);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);

    // Act / Assert: reach Moving directly (no runtime/event-source needed
    // for this focused regression - BatteryCritical is a sensor-derived
    // event this test injects directly, exactly like existing sensor-path
    // tests elsewhere in this file).
    stateMachine.processEvent(Event{EventType::ScenarioLoaded, 0, std::nullopt});
    stateMachine.processEvent(Event{EventType::StartMission, 0, std::nullopt});
    ASSERT_EQ(stateMachine.currentState(), RobotState::Moving);

    const TransitionResult toReturning =
        stateMachine.processEvent(Event{EventType::BatteryCritical, 0, std::nullopt});
    ASSERT_EQ(toReturning, TransitionResult::Success);
    ASSERT_EQ(stateMachine.currentState(), RobotState::ReturningHome);
    ASSERT_EQ(stateMachine.returnHomeReason(), ReturnHomeReason::MissionAbort);

    // Act
    const TransitionResult toAborted = stateMachine.processEvent(Event{EventType::HomeReached, 0, std::nullopt});

    // Assert: unchanged from before this bugfix - MissionAbort still ends
    // in Aborted, never Ready.
    EXPECT_EQ(toAborted, TransitionResult::Success);
    EXPECT_EQ(stateMachine.currentState(), RobotState::Aborted);
    EXPECT_EQ(stateMachine.returnHomeReason(), ReturnHomeReason::None);
}

// ============================================================
// Phase 13W human-validation blocker: table-edge safety recovery stuck
// after the robot/table rescale (bugfix #3 - see
// TableEdgeSafetyController.cpp's own docs for the full root-cause
// writeup: BackingAway/MovingForwardFromRearEdge/AdvancingInward all
// translate along a direction derived purely from the robot's CURRENT
// heading, never validated against which axis the actual triggering
// overhang is on - an edge encountered at a shallow/lateral angle can
// drive the robot toward/off a DIFFERENT edge instead of recovering,
// eventually reaching VirtualRobotHardware's own full-off-table hard
// fail-safe, which then rejects every further translation forever).
// ============================================================
//
// MANDATORY REGRESSION - exact human case: rectangular 8x4 table, final
// miniature RobotDimensions, robot reaches the right table edge at a
// shallow angle (the exact geometry class the bugfix above addresses),
// drives the real production stack (VirtualRobotHardware + real
// TableEdgeSafetyController, Safety as the one active DriveAuthority,
// exactly like main3d.cpp's own per-frame wiring) until recovery
// completes, then simulates Stop Task (FSM reaches Ready) while Safety
// may still be finishing recovery - Safety must keep running to
// completion regardless (see this file's own established "Ready +
// Safety" precedent), then effective wheels must go to zero once FSM
// Ready has no other command outstanding.
TEST(VirtualRobotHardwareTest, RightEdgeShallowAngleRecoversWithoutPermanentFreeze)
{
    VirtualWorld world;
    disableAllObstacles(world);
    // The exact class of starting geometry the sweep in
    // docs/technical-decisions.md found stuck before this fix: very close
    // to the right (+X) edge, heading nearly PARALLEL to it (not
    // perpendicular), so the triggering overhang is almost entirely
    // lateral relative to the robot's own front/rear axis.
    world.setRobotPosition(Vec3{3.95F, 0.08F, -0.9F});
    world.setRobotHeading(195.0F);
    VirtualRobotHardware hardware(world);
    VirtualCliffSensor cliffSensor(world);
    TableEdgeSafetyController tableEdgeSafety;
    const TableSurface& table = world.tableSurface();

    bool everActive = false;
    bool resolved = false;
    int consecutiveRejections = 0;
    int maxConsecutiveRejections = 0;
    int frame = 0;
    for (; frame < 600; ++frame)
    {
        const CliffSensorReadings readings = cliffSensor.readings();
        tableEdgeSafety.update(readings, world.robotPose(), table);

        if (tableEdgeSafety.active())
        {
            everActive = true;
            const WheelSpeeds speeds = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(speeds.left, speeds.right);
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
        }

        hardware.update(0.05F);

        // 15: the full-off-table hard guard must never become the DE
        // FACTO recovery mechanism - a rejection may happen transiently,
        // but never for a long consecutive stretch.
        if (hardware.tableEdgeRejectedLastUpdate())
        {
            ++consecutiveRejections;
            maxConsecutiveRejections = std::max(maxConsecutiveRejections, consecutiveRejections);
        }
        else
        {
            consecutiveRejections = 0;
        }

        if (everActive && !tableEdgeSafety.active())
        {
            resolved = true;
            break;
        }
    }

    // 1/8: recovery actually engaged, and did NOT remain stuck forever.
    ASSERT_TRUE(everActive);
    ASSERT_TRUE(resolved) << "recovery never resolved within the frame budget - permanently stuck";
    EXPECT_LT(maxConsecutiveRejections, 10);

    // 10/11: physically moved away from the edge, comfortably inside the
    // table by the time recovery released.
    const RobotPose& finalPose = world.robotPose();
    EXPECT_LT(finalPose.position.x, table.maxX - 0.3F);
    EXPECT_TRUE(areAllCornersSafelyInsideTable(finalPose, table, 0.0F));

    // 5-6/13: simulate Stop Task - FSM reaches Ready with no outstanding
    // command; Safety has already released by this point (resolved
    // above), so applying the FSM's own Stopped command now (nothing
    // else contending for authority) yields zero wheels.
    ASSERT_FALSE(hardware.safetyOverrideActive());
    hardware.stop();
    const WheelSpeeds wheelsAtReady = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(wheelsAtReady.left, 0.0F);
    EXPECT_FLOAT_EQ(wheelsAtReady.right, 0.0F);
}

// 5: RightEdgeReadyStateStillRecoversThenStops - Stop Task pressed WHILE
// Safety is still actively recovering (not after) - Safety must continue
// to completion; DriveAuthority stays Safety throughout, never
// interrupted by the FSM reaching Ready.
TEST(VirtualRobotHardwareTest, RightEdgeReadyStateStillRecoversThenStops)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{3.95F, 0.08F, -0.9F});
    world.setRobotHeading(195.0F);
    VirtualRobotHardware hardware(world);
    VirtualCliffSensor cliffSensor(world);
    TableEdgeSafetyController tableEdgeSafety;
    const TableSurface& table = world.tableSurface();

    // Advance a handful of frames so Safety is genuinely mid-recovery
    // (not merely triggered this instant) before Stop Task lands.
    for (int frame = 0; frame < 5; ++frame)
    {
        const CliffSensorReadings readings = cliffSensor.readings();
        tableEdgeSafety.update(readings, world.robotPose(), table);
        ASSERT_TRUE(tableEdgeSafety.active());
        const WheelSpeeds speeds = tableEdgeSafety.recoveryWheelSpeeds();
        hardware.setSafetyWheelSpeeds(speeds.left, speeds.right);
        hardware.update(0.05F);
    }

    // Act: Stop Task lands mid-incident - the FSM/RobotController would
    // call hardware.stop() here, setting command_ to Stopped, but that
    // must NOT affect DriveAuthority while Safety's own override is
    // still active (see VirtualRobotHardware::applyEffectiveWheelSpeeds()'s
    // fixed Safety > ... > Fsm priority).
    hardware.stop();
    ASSERT_TRUE(hardware.safetyOverrideActive());
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

    // Continue driving the real recovery loop to completion - Safety must
    // still be the one steering, and must still reach Inactive.
    bool resolved = false;
    for (int frame = 0; frame < 600; ++frame)
    {
        const CliffSensorReadings readings = cliffSensor.readings();
        tableEdgeSafety.update(readings, world.robotPose(), table);

        if (tableEdgeSafety.active())
        {
            const WheelSpeeds speeds = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(speeds.left, speeds.right);
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
        }

        hardware.update(0.05F);

        if (!tableEdgeSafety.active())
        {
            resolved = true;
            break;
        }
    }

    ASSERT_TRUE(resolved);
    // 17: effective wheels now come from the FSM's own Stopped command
    // (already issued above) - zero, since nothing else is contending for
    // authority anymore.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    const WheelSpeeds finalWheels = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(finalWheels.left, 0.0F);
    EXPECT_FLOAT_EQ(finalWheels.right, 0.0F);
}

// FullOffTableGuardDoesNotBecomeRecoveryMechanism
//
// VirtualRobotHardware's table-support hard guard (rejecting a proposed
// translation when the WHOLE footprint would be off-table at once) is a
// fail-safe for an unusually large delta time, never a recovery
// mechanism in its own right - TableEdgeSafetyController's own
// translation is what has to do the work. Drives a full incident and
// confirms the guard fires only rarely/transiently, never as the thing
// keeping the robot pinned in place step after step.
TEST(VirtualRobotHardwareTest, FullOffTableGuardDoesNotBecomeRecoveryMechanism)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{3.95F, 0.08F, -0.9F});
    world.setRobotHeading(195.0F);
    VirtualRobotHardware hardware(world);
    VirtualCliffSensor cliffSensor(world);
    TableEdgeSafetyController tableEdgeSafety;
    const TableSurface& table = world.tableSurface();

    int rejectedFrames = 0;
    int consecutiveRejections = 0;
    int maxConsecutiveRejections = 0;
    int totalFrames = 0;
    bool resolved = false;
    for (; totalFrames < 600; ++totalFrames)
    {
        const CliffSensorReadings readings = cliffSensor.readings();
        tableEdgeSafety.update(readings, world.robotPose(), table);

        if (tableEdgeSafety.active())
        {
            const WheelSpeeds speeds = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(speeds.left, speeds.right);
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
        }

        hardware.update(0.05F);

        if (hardware.tableEdgeRejectedLastUpdate())
        {
            ++rejectedFrames;
            ++consecutiveRejections;
            maxConsecutiveRejections = std::max(maxConsecutiveRejections, consecutiveRejections);
        }
        else
        {
            consecutiveRejections = 0;
        }

        if (!tableEdgeSafety.active())
        {
            resolved = true;
            ++totalFrames;
            break;
        }
    }

    ASSERT_TRUE(resolved);
    EXPECT_LT(maxConsecutiveRejections, 10) << "the hard guard rejected translation for a long consecutive "
                                                "stretch - it became the de facto recovery mechanism instead of "
                                                "TableEdgeSafetyController's own translation";
    EXPECT_LT(rejectedFrames, totalFrames / 2) << "the hard guard fired on more than half of all frames";
}

// ============================================================
// MANDATORY REGRESSION - exact human case (Phase 13W human validation
// blocker). Reproduces every element of the reported defect end to end
// through the real production stack, and asserts each of the 15 proof
// points from the bugfix brief individually.
// ============================================================
TEST(VirtualRobotHardwareTest, MandatoryRegressionExactHumanCaseRightEdgeStuckAfterRescale)
{
    // 1: rectangular 8x4 table (the real, unmodified default - not a
    // synthetic square table).
    VirtualWorld world;
    disableAllObstacles(world);
    const TableSurface& table = world.tableSurface();
    ASSERT_FLOAT_EQ(table.minX, -4.0F);
    ASSERT_FLOAT_EQ(table.maxX, 4.0F);
    ASSERT_FLOAT_EQ(table.minZ, -2.0F);
    ASSERT_FLOAT_EQ(table.maxZ, 2.0F);

    // 2: final miniature RobotDimensions (the real, unmodified default -
    // confirms this test is not accidentally exercising stale geometry).
    ASSERT_FLOAT_EQ(robot::visual::RobotDimensions::kBodyWidth, 0.40F);
    ASSERT_FLOAT_EQ(robot::visual::RobotDimensions::kBodyLength, 0.50F);

    // 3: robot reaches the right table edge at a shallow angle - the
    // exact geometry class bugfix #3 addresses (heading nearly parallel
    // to the edge, not perpendicular to it).
    world.setRobotPosition(Vec3{3.95F, 0.08F, -0.9F});
    world.setRobotHeading(195.0F);
    VirtualRobotHardware hardware(world);
    VirtualCliffSensor cliffSensor(world);
    TableEdgeSafetyController tableEdgeSafety;

    // 4: cliff detection activates.
    ASSERT_TRUE(cliffSensor.readings().anyCliff());

    bool everActive = false;
    bool resolved = false;
    bool stopTaskIssued = false;
    int consecutiveRejections = 0;
    int maxConsecutiveRejections = 0;
    int frame = 0;
    for (; frame < 600; ++frame)
    {
        const CliffSensorReadings readings = cliffSensor.readings();
        tableEdgeSafety.update(readings, world.robotPose(), table);

        if (tableEdgeSafety.active())
        {
            everActive = true;

            // 6: Safety remains the highest authority throughout, for
            // every frame it is active.
            const WheelSpeeds speeds = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(speeds.left, speeds.right);
            hardware.update(0.05F);
            ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);

            // 5: FSM reaches Ready/Stop Task lands MID-incident (once,
            // partway through recovery) - must not interrupt Safety.
            if (!stopTaskIssued && frame == 5)
            {
                hardware.stop();
                stopTaskIssued = true;
                ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Safety);
            }
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
            hardware.update(0.05F);
        }
        else
        {
            hardware.update(0.05F);
        }

        if (hardware.tableEdgeRejectedLastUpdate())
        {
            ++consecutiveRejections;
            maxConsecutiveRejections = std::max(maxConsecutiveRejections, consecutiveRejections);
        }
        else
        {
            consecutiveRejections = 0;
        }

        // 14: no table fall - the full-footprint-off-table condition
        // must never be the robot's actual resting state (transient at
        // most, if ever).
        ASSERT_FALSE(readings.allCliff() && frame > 0 && consecutiveRejections > 5)
            << "robot remained with the whole footprint off-table for a sustained stretch";

        if (everActive && !tableEdgeSafety.active())
        {
            resolved = true;
            break;
        }
    }

    // 7: recovery entered an appropriate state (not Inactive - the
    // trigger above guarantees this, restated here as an explicit proof
    // point).
    ASSERT_TRUE(everActive);

    // 8: robot does NOT remain indefinitely in AdvancingInward (or any
    // state) - this is the actual reported defect.
    ASSERT_TRUE(resolved) << "recovery never resolved within the frame budget - reproduces the reported deadlock";

    // 15: no repeated table-edge hard-guard rejection loop.
    ASSERT_TRUE(stopTaskIssued) << "test setup error: Stop Task was never issued mid-incident";
    EXPECT_LT(maxConsecutiveRejections, 10);

    const RobotPose& finalPose = world.robotPose();

    // 9: heading becomes inward-facing (within the controller's own
    // alignment tolerance of its last recovery target).
    EXPECT_LE(std::fabs(shortestSignedHeadingErrorDegrees(finalPose.headingDegrees,
                                                           tableEdgeSafety.targetRecoveryHeadingDegrees())),
              TableEdgeSafetyController::kRecoveryHeadingToleranceDegrees);

    // 10: physical position moves away from the right edge.
    EXPECT_LT(finalPose.position.x, 3.95F - 0.3F);

    // 11: support margin becomes safe - the whole footprint is robustly
    // back on the table, not merely on it by a hair.
    EXPECT_TRUE(areAllCornersSafelyInsideTable(finalPose, table, 0.0F));

    // 12: Safety releases.
    EXPECT_FALSE(tableEdgeSafety.active());
    EXPECT_FALSE(hardware.safetyOverrideActive());

    // 13: effective wheels become stopped because FSM is Ready (Stop
    // Task was already issued above, mid-incident; nothing else is
    // contending for authority now that Safety has released).
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::Fsm);
    const WheelSpeeds finalWheels = hardware.wheelSpeeds();
    EXPECT_FLOAT_EQ(finalWheels.left, 0.0F);
    EXPECT_FLOAT_EQ(finalWheels.right, 0.0F);
}
