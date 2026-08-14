#pragma once

#include <cstddef>

#include "robot/IEventSource.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"

namespace robot
{

// Summarizes what happened during one Simulator::run() call.
struct SimulationResult
{
    RobotState finalState;
    std::size_t eventsProcessed = 0;
    std::size_t successfulTransitions = 0;
    std::size_t rejectedTransitions = 0;
};

// Pulls events from an IEventSource and feeds them to a RobotStateMachine
// one at a time, tallying the outcome. Simulator owns neither the event
// source nor the state machine - both are supplied by the caller and must
// outlive the Simulator - so it can drive any IEventSource implementation
// against any already-constructed machine without taking on their
// lifetimes or allocating anything itself.
class Simulator
{
public:
    Simulator(IEventSource& eventSource, RobotStateMachine& stateMachine);

    SimulationResult run();

private:
    IEventSource& eventSource_;
    RobotStateMachine& stateMachine_;
};

} // namespace robot
