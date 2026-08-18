#include <gtest/gtest.h>

#include "robot/CompositePollingEventSource.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/DemoCommandSource.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::CompositePollingEventSource;
using robot::HardwareEventSource;
using robot::RobotController;
using robot::RobotRuntime;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::RuntimeStepResult;
using robot::visual::DemoCommandSource;
using robot::visual::Vec3;
using robot::visual::VirtualDriveCommand;
using robot::visual::VirtualRobotHardware;
using robot::visual::VirtualWorld;

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
    // Arrange
    VirtualWorld world; // default heading 0.0F, faces +Z
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
    // Arrange: default heading is 0.0F, which VisualRobot.cpp's
    // rlRotatef(headingDegrees, 0, 1, 0) convention (and VirtualWorld.hpp/
    // VisualRobot.hpp's own docs) defines as facing +Z.
    VirtualWorld world;
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

// --- Geometry-backed obstacleDetected()/obstacleDistance() (Phase 13O) ---
//
// The default VirtualWorld places its one enabled "blocking" demo obstacle
// (VirtualWorld::kBlockingObstacleIndex) directly ahead of the robot's
// start pose, far enough away that obstacleDetected() starts false (see
// VirtualWorld.cpp) - these tests reposition the robot via
// setRobotPosition() to exercise near/far/disabled/moved-away cases against
// that same real obstacle, rather than reimplementing sensor geometry here.

// 14: ObstacleDetectedFalseWhenFarFromBlockingObstacle
TEST(VirtualRobotHardwareTest, ObstacleDetectedFalseWhenFarFromBlockingObstacle)
{
    // Arrange / Act: default world - robot starts well outside detection
    // range of the blocking obstacle.
    VirtualWorld world;
    VirtualRobotHardware hardware(world);

    // Assert
    EXPECT_FALSE(hardware.obstacleDetected());
    ASSERT_TRUE(hardware.obstacleDistance().has_value());
    EXPECT_GT(*hardware.obstacleDistance(), 1.0F);
}

// 15: ObstacleDetectedTrueWhenWithinThreshold
TEST(VirtualRobotHardwareTest, ObstacleDetectedTrueWhenWithinThreshold)
{
    // Arrange: move the robot close enough to the blocking obstacle
    // (Z 4.3, near face Z 3.7) that the front-sensor distance is 0.5.
    VirtualWorld world;
    world.setRobotPosition(Vec3{-3.0F, 0.125F, 2.8F});
    VirtualRobotHardware hardware(world);

    // Act / Assert
    EXPECT_TRUE(hardware.obstacleDetected());
    ASSERT_TRUE(hardware.obstacleDistance().has_value());
    EXPECT_NEAR(*hardware.obstacleDistance(), 0.5F, 0.01F);
}

// 16: ObstacleDisabledMeansNotDetected
TEST(VirtualRobotHardwareTest, ObstacleDisabledMeansNotDetected)
{
    // Arrange: same close position as above, but the blocking obstacle is
    // disabled.
    VirtualWorld world;
    world.setRobotPosition(Vec3{-3.0F, 0.125F, 2.8F});
    world.setObstacleEnabled(VirtualWorld::kBlockingObstacleIndex, false);
    VirtualRobotHardware hardware(world);

    // Act / Assert
    EXPECT_FALSE(hardware.obstacleDetected());
    EXPECT_FALSE(hardware.obstacleDistance().has_value());
}

// 17: ObstacleMovedAwayMeansNotDetected
TEST(VirtualRobotHardwareTest, ObstacleMovedAwayMeansNotDetected)
{
    // Arrange: same close position as above, but the blocking obstacle has
    // been relocated far away.
    VirtualWorld world;
    world.setRobotPosition(Vec3{-3.0F, 0.125F, 2.8F});
    world.setObstaclePosition(VirtualWorld::kBlockingObstacleIndex, Vec3{100.0F, 0.4F, 100.0F});
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
    // yet), so it must still move exactly as before.
    VirtualWorld world;
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
    // Arrange
    VirtualWorld world;
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
    // Arrange: default world - the blocking obstacle
    // (VirtualWorld::kBlockingObstacleIndex) sits directly ahead on the
    // robot's real forward path.
    VirtualWorld world;
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
    hardware.update(1.0F); // z: 1.0 -> 2.0; sensor distance 1.3 (not yet detected)

    // Act / Assert: DemoCommandSource is now exhausted, so this step polls
    // HardwareEventSource - still no obstacle within detection range yet.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::NoEvent);
    ASSERT_EQ(stateMachine.currentState(), RobotState::Moving);
    hardware.update(1.0F); // z: 2.0 -> 3.0; sensor distance 0.3 (within threshold)

    // Act / Assert: the real HardwareEventSource now observes the
    // false -> true edge and emits ObstacleDetected; RobotStateMachine
    // accepts Moving -> WaitingForObstacleClear; RobotController stops the
    // hardware - all through the unmodified production chain.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(stateMachine.currentState(), RobotState::WaitingForObstacleClear);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::Stopped);
    const float zWhenStopped = world.robotPose().position.z;
    hardware.update(1.0F); // Stopped -> update() is a no-op.
    EXPECT_FLOAT_EQ(world.robotPose().position.z, zWhenStopped);

    // Act / Assert: obstacle condition persists (true -> true) - no repeated
    // ObstacleDetected, no state change.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::NoEvent);
    ASSERT_EQ(stateMachine.currentState(), RobotState::WaitingForObstacleClear);

    // Act: disable the blocking obstacle - identical world-only mutation to
    // RobotSimulator3D's "O" key; never a manual Event injection.
    world.setObstacleEnabled(VirtualWorld::kBlockingObstacleIndex, false);

    // Act / Assert: HardwareEventSource observes the true -> false edge and
    // emits ObstacleCleared; RobotStateMachine returns to Moving (the
    // remembered resume state); RobotController resumes MoveForward.
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(stateMachine.currentState(), RobotState::Moving);
    ASSERT_EQ(hardware.currentCommand(), VirtualDriveCommand::MoveForward);

    // Act / Assert: movement resumes.
    hardware.update(1.0F);
    EXPECT_GT(world.robotPose().position.z, zWhenStopped);
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
    // first hardware sample observes the false -> true edge.
    VirtualWorld world;
    world.setRobotPosition(Vec3{-3.0F, 0.125F, 2.8F}); // sensor distance 0.5
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
