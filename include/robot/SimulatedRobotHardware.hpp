#pragma once

#include "robot/IRobotHardware.hpp"

namespace robot
{

// Last actuator command SimulatedRobotHardware received. This is
// simulation/test observability only, not part of IRobotHardware - a real
// hardware implementation would have no equivalent in-memory field to
// report.
enum class RobotCommand
{
    Stopped,
    MovingForward,
    ReturningToBase
};

// Deterministic in-memory implementation of IRobotHardware for desktop
// testing. Sensor state defaults to battery=100, no obstacle, emergency
// stop not pressed, and is mutated only through the setters below - never
// by any timer, thread, or external input - so tests can drive exact
// scenarios without sleeps or flakiness.
class SimulatedRobotHardware : public IRobotHardware
{
public:
    SimulatedRobotHardware();

    int batteryLevelPercent() const override;
    bool obstacleDetected() const override;
    bool emergencyStopPressed() const override;

    void moveForward() override;
    void stop() override;
    void returnToBase() override;

    // Simulation-only mutators, not part of IRobotHardware.
    // Throws std::invalid_argument if percent is outside 0..100.
    void setBatteryLevelPercent(int percent);
    void setObstacleDetected(bool detected);
    void setEmergencyStopPressed(bool pressed);

    // Simulation/test observability, not part of IRobotHardware.
    RobotCommand currentCommand() const;

private:
    int batteryLevelPercent_;
    bool obstacleDetected_;
    bool emergencyStopPressed_;
    RobotCommand currentCommand_;
};

} // namespace robot
