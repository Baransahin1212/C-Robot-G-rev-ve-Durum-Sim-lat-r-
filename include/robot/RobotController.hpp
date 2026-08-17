#pragma once

#include "robot/IRobotHardware.hpp"
#include "robot/RobotState.hpp"

namespace robot
{

// Translates a resulting RobotState into the corresponding actuator command
// on IRobotHardware. This is the only place FSM states and hardware
// commands meet - RobotStateMachine has no knowledge of IRobotHardware, and
// RobotController has no knowledge of Event/EventType or transition rules.
// It only reacts to a state it is handed; it does not decide FSM
// transitions, parse JSON, log, or report.
class RobotController
{
public:
    explicit RobotController(IRobotHardware& hardware);

    void applyState(RobotState state);

private:
    IRobotHardware& hardware_;
};

} // namespace robot
