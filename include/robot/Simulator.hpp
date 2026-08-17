#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "robot/IEventSource.hpp"
#include "robot/ISimulationLogger.hpp"
#include "robot/RobotController.hpp"
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
    std::optional<std::uint64_t> lastEventTimestampMs;
};

// Pulls events from an IEventSource and feeds them to a RobotStateMachine
// one at a time, tallying the outcome. Simulator owns neither the event
// source, the state machine, the logger, nor the controller - all are
// supplied by the caller and must outlive the Simulator - so it can drive
// any IEventSource implementation against any already-constructed machine
// without taking on their lifetimes or allocating anything itself.
//
// `logger` and `controller` are optional, non-owning pointers (default
// nullptr). Requiring every caller - including every Simulator unit test -
// to supply a real ISimulationLogger or RobotController would be an
// awkward, unrelated burden on code that only cares about FSM
// orchestration. A null pointer means "don't log" / "no hardware attached";
// a non-null pointer is checked before each call, adding no heap
// allocation and no behavioral difference to SimulationResult either way.
//
// When `controller` is supplied, Simulator synchronizes hardware to the
// state machine's current state once before processing any events, and
// again after every transition RobotStateMachine reports as successful.
// Rejected transitions never reach the controller - RobotStateMachine
// remains the sole authority on transition validity, and hardware state
// only ever reflects a state the FSM actually entered.
class Simulator
{
public:
    Simulator(IEventSource& eventSource,
              RobotStateMachine& stateMachine,
              ISimulationLogger* logger = nullptr,
              RobotController* controller = nullptr);

    SimulationResult run();

private:
    IEventSource& eventSource_;
    RobotStateMachine& stateMachine_;
    ISimulationLogger* logger_;
    RobotController* controller_;
};

} // namespace robot
