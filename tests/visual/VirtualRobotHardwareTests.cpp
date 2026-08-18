#include <gtest/gtest.h>

#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/DemoCommandSource.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

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
