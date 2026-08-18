#pragma once

#include <cstddef>

#include "robot/LiveRuntimeRunner.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/SensorScript.hpp"
#include "robot/SimulatedRobotHardware.hpp"

namespace robot
{

// Simulator-only scripting adapter that applies a SensorScript's mutations
// to a SimulatedRobotHardware immediately before each RobotRuntime::step()
// call. This is deliberately a separate component from LiveRuntimeRunner,
// not a variant of it: LiveRuntimeRunner stays fully generic (it knows
// nothing about SimulatedRobotHardware, SensorScript, or cycle-based
// injection - see its own header), while this class owns exactly that
// simulator-specific responsibility. Reuses RuntimeRunSummary from
// LiveRuntimeRunner.hpp so callers get an identical summary shape either
// way. See docs/technical-decisions.md (Phase 13I) for the full pipeline:
//   SensorScript -> ScriptedLiveRuntimeRunner -> SimulatedRobotHardware
//     -> HardwareEventSource -> RobotRuntime -> RobotStateMachine
class ScriptedLiveRuntimeRunner
{
public:
    // runtime, hardware, and script must all outlive this object - the
    // same non-owning-reference pattern RobotRuntime/LiveRuntimeRunner
    // use for their own dependencies. `hardware` must be the same
    // instance the IPollingEventSource passed to `runtime` observes,
    // otherwise mutations here would never be seen by that source.
    ScriptedLiveRuntimeRunner(RobotRuntime& runtime, SimulatedRobotHardware& hardware, const SensorScript& script);

    // Cycles are zero-based: for cycle = 0 .. cycleCount - 1, every
    // SensorScript entry scheduled for that cycle is applied to hardware
    // (in file order, for entries sharing a cycle) via the existing
    // SimulatedRobotHardware setters, and then runtime.step() is called
    // exactly once. Entries scheduled at or beyond cycleCount are simply
    // never applied - not an error, just outside this run's range.
    RuntimeRunSummary runCycles(std::size_t cycleCount);

private:
    void applyEntriesForCycle(std::size_t cycle);

    RobotRuntime& runtime_;
    SimulatedRobotHardware& hardware_;
    const SensorScript& script_;
    std::size_t nextEntryIndex_ = 0;
};

} // namespace robot
