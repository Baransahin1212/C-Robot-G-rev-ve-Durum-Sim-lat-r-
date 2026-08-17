#include <gtest/gtest.h>

#include "robot/RobotController.hpp"
#include "robot/RobotState.hpp"
#include "robot/SimulatedRobotHardware.hpp"

namespace
{

using robot::RobotCommand;
using robot::RobotController;
using robot::RobotState;
using robot::SimulatedRobotHardware;

} // namespace

TEST(RobotControllerTest, IdleStopsHardware)
{
    // Arrange
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);

    // Act
    controller.applyState(RobotState::Idle);

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

TEST(RobotControllerTest, ReadyStopsHardware)
{
    // Arrange
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);

    // Act
    controller.applyState(RobotState::Ready);

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

TEST(RobotControllerTest, MovingMovesHardwareForward)
{
    // Arrange
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);

    // Act
    controller.applyState(RobotState::Moving);

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::MovingForward);
}

TEST(RobotControllerTest, WaitingForObstacleClearStopsHardware)
{
    // Arrange
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);

    // Act
    controller.applyState(RobotState::WaitingForObstacleClear);

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

TEST(RobotControllerTest, ReturningHomeReturnsHardwareToBase)
{
    // Arrange
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);

    // Act
    controller.applyState(RobotState::ReturningHome);

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::ReturningToBase);
}

TEST(RobotControllerTest, CompletedStopsHardware)
{
    // Arrange
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);

    // Act
    controller.applyState(RobotState::Completed);

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

TEST(RobotControllerTest, AbortedStopsHardware)
{
    // Arrange
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);

    // Act
    controller.applyState(RobotState::Aborted);

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

TEST(RobotControllerTest, EmergencyStoppedStopsHardware)
{
    // Arrange
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);

    // Act
    controller.applyState(RobotState::EmergencyStopped);

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

TEST(RobotControllerTest, ErrorStopsHardware)
{
    // Arrange
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);

    // Act
    controller.applyState(RobotState::Error);

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

TEST(RobotControllerTest, ControllerOverwritesPriorCommandAcrossStateChanges)
{
    // Arrange
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);

    // Act / Assert: Moving -> MovingForward
    controller.applyState(RobotState::Moving);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::MovingForward);

    // Act / Assert: EmergencyStopped -> Stopped
    controller.applyState(RobotState::EmergencyStopped);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);

    // Act / Assert: ReturningHome -> ReturningToBase
    controller.applyState(RobotState::ReturningHome);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::ReturningToBase);
}
