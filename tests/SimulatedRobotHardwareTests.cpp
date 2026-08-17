#include <gtest/gtest.h>

#include <stdexcept>

#include "robot/SimulatedRobotHardware.hpp"

namespace
{

using robot::RobotCommand;
using robot::SimulatedRobotHardware;

} // namespace

TEST(SimulatedRobotHardwareTest, DefaultsToFullBattery)
{
    // Arrange / Act
    SimulatedRobotHardware hardware;

    // Assert
    EXPECT_EQ(hardware.batteryLevelPercent(), 100);
}

TEST(SimulatedRobotHardwareTest, DefaultsToNoObstacle)
{
    // Arrange / Act
    SimulatedRobotHardware hardware;

    // Assert
    EXPECT_FALSE(hardware.obstacleDetected());
}

TEST(SimulatedRobotHardwareTest, DefaultsToEmergencyStopNotPressed)
{
    // Arrange / Act
    SimulatedRobotHardware hardware;

    // Assert
    EXPECT_FALSE(hardware.emergencyStopPressed());
}

TEST(SimulatedRobotHardwareTest, DefaultsToStoppedCommand)
{
    // Arrange / Act
    SimulatedRobotHardware hardware;

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

TEST(SimulatedRobotHardwareTest, BatteryLevelCanBeChanged)
{
    // Arrange
    SimulatedRobotHardware hardware;

    // Act
    hardware.setBatteryLevelPercent(42);

    // Assert
    EXPECT_EQ(hardware.batteryLevelPercent(), 42);
}

TEST(SimulatedRobotHardwareTest, ObstacleStateCanBeChanged)
{
    // Arrange
    SimulatedRobotHardware hardware;

    // Act
    hardware.setObstacleDetected(true);

    // Assert
    EXPECT_TRUE(hardware.obstacleDetected());
}

TEST(SimulatedRobotHardwareTest, EmergencyStopStateCanBeChanged)
{
    // Arrange
    SimulatedRobotHardware hardware;

    // Act
    hardware.setEmergencyStopPressed(true);

    // Assert
    EXPECT_TRUE(hardware.emergencyStopPressed());
}

TEST(SimulatedRobotHardwareTest, MoveForwardChangesCommand)
{
    // Arrange
    SimulatedRobotHardware hardware;

    // Act
    hardware.moveForward();

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::MovingForward);
}

TEST(SimulatedRobotHardwareTest, StopChangesCommand)
{
    // Arrange
    SimulatedRobotHardware hardware;
    hardware.moveForward();

    // Act
    hardware.stop();

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

TEST(SimulatedRobotHardwareTest, ReturnToBaseChangesCommand)
{
    // Arrange
    SimulatedRobotHardware hardware;

    // Act
    hardware.returnToBase();

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::ReturningToBase);
}

TEST(SimulatedRobotHardwareTest, BatteryBelowZeroIsRejected)
{
    // Arrange
    SimulatedRobotHardware hardware;
    hardware.setBatteryLevelPercent(50);

    // Act / Assert
    EXPECT_THROW(hardware.setBatteryLevelPercent(-1), std::invalid_argument);
    EXPECT_EQ(hardware.batteryLevelPercent(), 50);
}

TEST(SimulatedRobotHardwareTest, BatteryAboveHundredIsRejected)
{
    // Arrange
    SimulatedRobotHardware hardware;
    hardware.setBatteryLevelPercent(50);

    // Act / Assert
    EXPECT_THROW(hardware.setBatteryLevelPercent(101), std::invalid_argument);
    EXPECT_EQ(hardware.batteryLevelPercent(), 50);
}
