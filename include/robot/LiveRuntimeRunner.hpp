#pragma once

#include <cstddef>

#include "robot/RobotRuntime.hpp"

namespace robot
{

// Summarizes what happened during one LiveRuntimeRunner::runCycles() call.
struct RuntimeRunSummary
{
    std::size_t cyclesExecuted = 0;
    std::size_t noEventCycles = 0;
    std::size_t acceptedTransitions = 0;
    std::size_t rejectedTransitions = 0;
};

// Deterministic, finite scheduler on top of RobotRuntime::step(). Where
// RobotRuntime represents exactly one live polling cycle, LiveRuntimeRunner
// represents a controlled, bounded *number* of cycles: it calls step()
// exactly cycleCount times, classifying and tallying every
// RuntimeStepResult, and never stops early merely because a cycle returned
// NoEvent. NoEvent is an ordinary, expected outcome for a live polling
// source (see IPollingEventSource) - not a termination signal - and
// continuing past it is the central point of this phase.
//
// LiveRuntimeRunner owns nothing RobotRuntime depends on, and it knows
// only RobotRuntime::step() and RuntimeStepResult - it has no knowledge of
// IRobotHardware, HardwareEventSource, IPollingEventSource,
// RobotController, RobotStateMachine, or SimulatedRobotHardware. It
// orchestrates cycle counting only, keeping scheduling independent of
// robot internals.
//
// This is deliberately not yet a real-time scheduler: runCycles() contains
// no std::chrono, no sleeping, no threads, and no timers. It is a pure,
// synchronous loop over a fixed number of step() calls, useful for
// deterministic tests and bounded live runs. An actual timed scheduling
// policy is future work.
class LiveRuntimeRunner
{
public:
    explicit LiveRuntimeRunner(RobotRuntime& runtime);

    // Executes exactly cycleCount calls to RobotRuntime::step(),
    // classifying and accumulating every result. runCycles(0) makes no
    // step() calls at all and returns an all-zero summary.
    RuntimeRunSummary runCycles(std::size_t cycleCount);

private:
    RobotRuntime& runtime_;
};

} // namespace robot
