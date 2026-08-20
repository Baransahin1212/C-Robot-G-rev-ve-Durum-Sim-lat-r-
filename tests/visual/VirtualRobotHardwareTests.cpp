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
    // Arrange: default heading 0.0F faces +Z.
    VirtualWorld world;
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
    // Arrange
    VirtualWorld world;
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
// underlying geometry query in isolation). All obstacles are heading
// straight along the default heading-0 (+Z) direction, with the one
// controlled obstacle at X 0 so the robot's collision circle is centered
// exactly on its footprint.

// 30: ManualForwardDriveStopsAtObstacleBoundary
TEST(VirtualRobotHardwareTest, ManualForwardDriveStopsAtObstacleBoundary)
{
    // Arrange: obstacle (0.8 cube) centered at Z 3.0 -> near face Z 2.6.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 3.0F});
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
    // face 2.6 minus collision radius ~0.5 -> boundary near Z 2.1).
    const float finalZ = world.robotPose().position.z;
    EXPECT_GT(finalZ, 1.0F);
    EXPECT_LT(finalZ, 2.15F);
}

// 31: ManualReverseAwayFromObstacleWorks
TEST(VirtualRobotHardwareTest, ManualReverseAwayFromObstacleWorks)
{
    // Arrange: drive up to the obstacle boundary first (same setup as
    // ManualForwardDriveStopsAtObstacleBoundary).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 3.0F});
    world.setObstacleEnabled(0, true);
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    for (int i = 0; i < 200; ++i)
    {
        hardware.update(0.05F);
    }
    const float zAtBoundary = world.robotPose().position.z;

    // Act: reverse.
    hardware.setManualWheelSpeeds(-1.0F, -1.0F);
    for (int i = 0; i < 20; ++i)
    {
        hardware.update(0.05F);
    }

    // Assert: moved back away from the obstacle - reverse is never
    // blocked by a collision guard that only ever rejects entering an
    // obstacle.
    EXPECT_LT(world.robotPose().position.z, zAtBoundary);
}

// 32: InPlaceRotationDoesNotTranslateIntoObstacle
TEST(VirtualRobotHardwareTest, InPlaceRotationDoesNotTranslateIntoObstacle)
{
    // Arrange: drive up to the obstacle boundary first.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 3.0F});
    world.setObstacleEnabled(0, true);
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    for (int i = 0; i < 200; ++i)
    {
        hardware.update(0.05F);
    }
    const float zAtBoundary = world.robotPose().position.z;
    const float headingBefore = world.robotPose().headingDegrees;

    // Act: in-place rotation right at the boundary.
    hardware.setManualWheelSpeeds(-0.5F, 0.5F);
    for (int i = 0; i < 20; ++i)
    {
        hardware.update(0.05F);
    }

    // Assert: position essentially unchanged (a circular footprint is
    // rotation-independent, so it is never blocked), heading did change.
    EXPECT_NEAR(world.robotPose().position.z, zAtBoundary, 0.01F);
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
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 3.0F});
    // Deliberately left disabled (disableAllObstacles() above).
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);

    // Act
    for (int i = 0; i < 200; ++i)
    {
        hardware.update(0.05F);
    }

    // Assert: drove straight through where an enabled obstacle would have
    // stopped it (~Z 2.1).
    EXPECT_GT(world.robotPose().position.z, 4.0F);
}

// 34: ReEnabledObstacleBlocksMovement
TEST(VirtualRobotHardwareTest, ReEnabledObstacleBlocksMovement)
{
    // Arrange: obstacle disabled - robot drives all the way through it and
    // out the far side (near face 2.6, far face 3.4).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 3.0F});
    world.setObstacleEnabled(0, false);
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);
    for (int i = 0; i < 160; ++i)
    {
        hardware.update(0.05F);
    }
    ASSERT_GT(world.robotPose().position.z, 4.0F);

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
    EXPECT_GT(world.robotPose().position.z, 3.4F);
}

// 35: WorldBoundsStillApplyWithCollisionGuardPresent
TEST(VirtualRobotHardwareTest, WorldBoundsStillApplyWithCollisionGuardPresent)
{
    // Arrange: no obstacles in the way - only the world-bounds clamp
    // should limit movement.
    VirtualWorld world;
    disableAllObstacles(world);
    VirtualRobotHardware hardware(world);
    hardware.setManualWheelSpeeds(1.0F, 1.0F);

    // Act: far more simulated time than needed to reach the ~10-unit
    // world half-extent at 1.0 unit/second.
    for (int i = 0; i < 100; ++i)
    {
        hardware.update(1.0F);
    }

    // Assert: clamped, not teleported off into infinity, and not stuck
    // short of the bound by a false collision.
    EXPECT_LE(world.robotPose().position.z, 10.0F);
    EXPECT_GE(world.robotPose().position.z, 9.0F);
}
