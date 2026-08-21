#include <cmath>

#include <gtest/gtest.h>

#include "robot/CompositePollingEventSource.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/DemoCommandSource.hpp"
#include "robot/visual/ForwardClearanceProbe.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/TableEdgeSafetyController.hpp"
#include "robot/visual/VirtualCliffSensor.hpp"
#include "robot/visual/VirtualObstacleSensorArray.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"
#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualMath.hpp"

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
using robot::visual::DriveAuthority;
using robot::visual::CliffSensorReadings;
using robot::visual::ForwardClearanceProbe;
using robot::visual::ReactiveObstacleAvoidance;
using robot::visual::robotPositionCollidesWithObstacles;
using robot::visual::shortestSignedHeadingErrorDegrees;
using robot::visual::TableEdgeSafetyController;
using robot::visual::TableSurface;
using robot::visual::VirtualCliffSensor;
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
    // should limit forward movement. Default world start position/heading
    // (X -3, Z 1.0, heading 0) - straight line toward the table's +Z edge
    // (tableSurface().maxZ, 6.0F by default).
    VirtualWorld world;
    disableAllObstacles(world);
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
    const WheelSpeeds turnSpeeds = avoidance.avoidanceWheelSpeeds();
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
    // engaged (as if the `A` toggle were OFF).
    VirtualWorld world;
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
        avoidance.update(/*enabled=*/true, triggerAvoidance, forwardCorridorClear);
        if (avoidance.active())
        {
            everActivated = true;
        }

        if (avoidance.active())
        {
            const WheelSpeeds turnSpeeds = avoidance.avoidanceWheelSpeeds();
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
    // Arrange: latch active, clearance still blocked - equivalent to a
    // mid-turn snapshot of the closed-loop test above, constructed
    // directly against the latch/hardware APIs rather than driving the
    // full FSM chain again.
    VirtualWorld world;
    VirtualRobotHardware hardware(world);
    ReactiveObstacleAvoidance avoidance;
    avoidance.update(/*enabled=*/true, /*triggerAvoidance=*/true, /*forwardCorridorClear=*/false);
    ASSERT_TRUE(avoidance.active());
    const WheelSpeeds turnSpeeds = avoidance.avoidanceWheelSpeeds();
    hardware.setAutonomousWheelSpeeds(turnSpeeds.left, turnSpeeds.right);
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    // Act: engage manual override mid-turn (main3d.cpp's `M` key).
    hardware.setManualWheelSpeeds(1.0F, -1.0F);

    // Assert: manual physically wins; the latch remains logically active
    // underneath (main3d.cpp keeps calling avoidance.update() every frame
    // regardless of manual mode - simulated here directly).
    ASSERT_EQ(hardware.driveAuthority(), DriveAuthority::Manual);
    avoidance.update(true, /*triggerAvoidance=*/false, /*forwardCorridorClear=*/false);
    EXPECT_TRUE(avoidance.active());

    // Act: leave manual mode while clearance is still blocked.
    hardware.clearManualWheelOverride();
    if (avoidance.active())
    {
        const WheelSpeeds resumedTurnSpeeds = avoidance.avoidanceWheelSpeeds();
        hardware.setAutonomousWheelSpeeds(resumedTurnSpeeds.left, resumedTurnSpeeds.right);
    }

    // Assert: authority returns to AUTONOMOUS, not FSM, since the latch
    // is still active.
    EXPECT_EQ(hardware.driveAuthority(), DriveAuthority::AutonomousAvoidance);

    // Act: clearance becomes safe while manual is no longer active.
    avoidance.update(true, false, /*forwardCorridorClear=*/true);
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
            if (framesSinceRelease >= 100)
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
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{0.0F, 0.125F, 5.0F}, 0.0F);

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
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{0.0F, 0.125F, -5.0F}, 180.0F);

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
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{5.0F, 0.125F, 0.0F}, 90.0F);

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
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{-5.0F, 0.125F, 0.0F}, 270.0F);

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
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{0.0F, 0.125F, 0.0F}, 45.0F);

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
    const EdgeRecoveryOutcome outcome = driveTowardEdgeAndRecover(Vec3{0.0F, 0.125F, 5.0F}, 20.0F);

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
    // Arrange
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 5.0F});
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
            if (framesSinceRelease >= 100)
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
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 5.6F}); // front corner exactly at the table edge
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
    avoidance.update(/*enabled=*/true, /*triggerAvoidance=*/true, /*forwardCorridorClear=*/false);
    ASSERT_TRUE(avoidance.active());
    const WheelSpeeds turnSpeeds = avoidance.avoidanceWheelSpeeds();
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
        avoidance.update(true, true, false);
        if (avoidance.active())
        {
            const WheelSpeeds speeds = avoidance.avoidanceWheelSpeeds();
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
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(3, Vec3{0.65F, 0.4F, 1.5F});
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
    // Arrange: obstacle index 3 offset per the regression test above;
    // robot approaches it from further back, straight along heading 0 (X
    // stays 0 the whole approach, so no ray - old or new - would EVER see
    // this obstacle; only the body corridor can).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, -2.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(3, Vec3{0.65F, 0.4F, 1.5F});
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
        avoidance.update(true, triggerAvoidance, forwardCorridorClear);
        if (avoidance.active())
        {
            everAvoidanceActive = true;
            const WheelSpeeds turnSpeeds = avoidance.avoidanceWheelSpeeds();
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
