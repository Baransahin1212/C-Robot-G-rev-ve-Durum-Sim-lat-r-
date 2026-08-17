#include "robot/SimulatedRobotHardware.hpp"

#include <stdexcept>

namespace robot
{

SimulatedRobotHardware::SimulatedRobotHardware()
    : batteryLevelPercent_(100)
    , obstacleDetected_(false)
    , emergencyStopPressed_(false)
    , currentCommand_(RobotCommand::Stopped)
{
}

int SimulatedRobotHardware::batteryLevelPercent() const
{
    return batteryLevelPercent_;
}

bool SimulatedRobotHardware::obstacleDetected() const
{
    return obstacleDetected_;
}

bool SimulatedRobotHardware::emergencyStopPressed() const
{
    return emergencyStopPressed_;
}

void SimulatedRobotHardware::moveForward()
{
    currentCommand_ = RobotCommand::MovingForward;
}

void SimulatedRobotHardware::stop()
{
    currentCommand_ = RobotCommand::Stopped;
}

void SimulatedRobotHardware::returnToBase()
{
    currentCommand_ = RobotCommand::ReturningToBase;
}

void SimulatedRobotHardware::setBatteryLevelPercent(int percent)
{
    if (percent < 0 || percent > 100)
    {
        throw std::invalid_argument("Battery level percent must be within 0..100");
    }
    batteryLevelPercent_ = percent;
}

void SimulatedRobotHardware::setObstacleDetected(bool detected)
{
    obstacleDetected_ = detected;
}

void SimulatedRobotHardware::setEmergencyStopPressed(bool pressed)
{
    emergencyStopPressed_ = pressed;
}

RobotCommand SimulatedRobotHardware::currentCommand() const
{
    return currentCommand_;
}

} // namespace robot
