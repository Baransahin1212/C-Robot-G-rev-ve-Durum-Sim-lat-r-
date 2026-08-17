#pragma once

namespace robot
{

// Abstraction over anything that can report sensor-like state and accept
// actuator commands: simulated hardware today, potentially a real robot
// later. RobotStateMachine intentionally does not depend on this interface
// - it remains a pure (state, event) -> state function. This interface
// exists for a future orchestration layer that will connect FSM
// states/events to hardware commands.
class IRobotHardware
{
public:
    virtual ~IRobotHardware() = default;

    // Sensor-like state
    virtual int batteryLevelPercent() const = 0;
    virtual bool obstacleDetected() const = 0;
    virtual bool emergencyStopPressed() const = 0;

    // Actuator commands
    virtual void moveForward() = 0;
    virtual void stop() = 0;
    virtual void returnToBase() = 0;
};

} // namespace robot
