#pragma once

#include <cstddef>

#include "robot/IEventSource.hpp"
#include "robot/ISimulationLogger.hpp"
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
// source, the state machine, nor the logger - all are supplied by the
// caller and must outlive the Simulator - so it can drive any IEventSource
// implementation against any already-constructed machine without taking on
// their lifetimes or allocating anything itself.
//
// `logger` is an optional, non-owning pointer (default nullptr). Requiring
// every caller - including every Simulator unit test - to supply a real
// ISimulationLogger would be an awkward, unrelated burden on code that only
// cares about FSM orchestration. A null pointer means "don't log"; a
// non-null pointer is checked before each call, adding no heap allocation
// and no behavioral difference to SimulationResult either way.
class Simulator
{
public:
    Simulator(IEventSource& eventSource, RobotStateMachine& stateMachine, ISimulationLogger* logger = nullptr);

    SimulationResult run();

private:
    IEventSource& eventSource_;
    RobotStateMachine& stateMachine_;
    ISimulationLogger* logger_;
};

} // namespace robot
