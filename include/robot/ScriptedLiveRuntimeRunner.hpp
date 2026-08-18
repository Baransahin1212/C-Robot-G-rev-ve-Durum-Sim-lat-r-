#pragma once

#include <cstddef>

#include "robot/LiveRuntimeRunner.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/ScriptedCommandEventSource.hpp"
#include "robot/SensorScript.hpp"
#include "robot/SimulatedRobotHardware.hpp"

namespace robot
{

// Simulator-only scripting adapter that, once per live runtime cycle,
// applies a SensorScript's mutations to a SimulatedRobotHardware and/or
// advances a ScriptedCommandEventSource's notion of "now" - both
// immediately before that cycle's RobotRuntime::step() call. This is
// deliberately a separate component from LiveRuntimeRunner, not a variant
// of it: LiveRuntimeRunner stays fully generic (it knows nothing about
// SimulatedRobotHardware, SensorScript, ScriptedCommandEventSource, or
// cycle-based injection - see its own header), while this class owns
// exactly that simulator-specific responsibility. Reuses RuntimeRunSummary
// from LiveRuntimeRunner.hpp so callers get an identical summary shape
// either way. See docs/technical-decisions.md (Phase 13I/13J) for the full
// pipelines:
//   SensorScript -> ScriptedLiveRuntimeRunner -> SimulatedRobotHardware
//     -> HardwareEventSource -> RobotRuntime -> RobotStateMachine
//   CommandScript -> ScriptedCommandEventSource -> CompositePollingEventSource
//     -> RobotRuntime -> RobotStateMachine
//
// The sensor script and command source are independent axes - either, both,
// or neither may be supplied (via the three constructors below), matching
// the CLI's independent --sensor-script/--command-script flags. This class
// does not construct or own the IPollingEventSource wiring `runtime` was
// built with (a plain HardwareEventSource, or a CompositePollingEventSource
// over a ScriptedCommandEventSource and a HardwareEventSource) - the
// caller (Application.cpp) composes that; this class only drives
// per-cycle mutation/notification ahead of runtime.step().
class ScriptedLiveRuntimeRunner
{
public:
    // Sensor-script-only (Phase 13I). runtime, hardware, and sensorScript
    // must all outlive this object - the same non-owning-reference pattern
    // RobotRuntime/LiveRuntimeRunner use for their own dependencies.
    // `hardware` must be the same instance the IPollingEventSource passed
    // to `runtime` observes, otherwise mutations here would never be seen
    // by that source.
    ScriptedLiveRuntimeRunner(RobotRuntime& runtime, SimulatedRobotHardware& hardware, const SensorScript& sensorScript);

    // Sensor-script and command-source together (Phase 13J). commandSource
    // must be the same instance (directly, or wrapped in a
    // CompositePollingEventSource) that the IPollingEventSource passed to
    // `runtime` polls, otherwise its eligible commands would never reach
    // the FSM.
    ScriptedLiveRuntimeRunner(RobotRuntime& runtime,
                               SimulatedRobotHardware& hardware,
                               const SensorScript& sensorScript,
                               ScriptedCommandEventSource& commandSource);

    // Command-source-only, no sensor script (Phase 13J).
    ScriptedLiveRuntimeRunner(RobotRuntime& runtime,
                               SimulatedRobotHardware& hardware,
                               ScriptedCommandEventSource& commandSource);

    // Cycles are zero-based: for cycle = 0 .. cycleCount - 1, (if a
    // sensor script was supplied) every SensorScript entry scheduled for
    // that cycle is applied to hardware, in file order for entries sharing
    // a cycle, via the existing SimulatedRobotHardware setters; (if a
    // command source was supplied) it is notified of the current cycle via
    // setCurrentCycle(); then runtime.step() is called exactly once.
    // Sensor entries scheduled at or beyond cycleCount are simply never
    // applied, and command entries scheduled at or beyond cycleCount are
    // simply never made eligible - neither is an error, just outside this
    // run's range.
    RuntimeRunSummary runCycles(std::size_t cycleCount);

private:
    void applyEntriesForCycle(std::size_t cycle);

    RobotRuntime& runtime_;
    SimulatedRobotHardware& hardware_;
    const SensorScript* sensorScript_ = nullptr;
    std::size_t nextEntryIndex_ = 0;
    ScriptedCommandEventSource* commandSource_ = nullptr;
};

} // namespace robot
