#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "robot/CommandScript.hpp"
#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"

namespace robot
{

// Turns a CommandScript's entries into an IPollingEventSource, the same
// role HardwareEventSource plays for sensor state - RobotRuntime consumes
// either (or, via CompositePollingEventSource, both) through the identical
// interface. Deliberately implements only IPollingEventSource, not
// IEventSource: a command script is scheduled against live runtime cycles
// (see setCurrentCycle()), which only makes sense under the "nullopt means
// nothing new THIS cycle, ask again later" polling contract - not
// IEventSource's "nullopt means exhausted for good".
//
// This class knows nothing about IRobotHardware, SimulatedRobotHardware,
// or sensor state - it only ever turns CommandScriptEntry values into
// Event values on the cycle schedule the caller drives it with.
class ScriptedCommandEventSource : public IPollingEventSource
{
public:
    // script must outlive this object - the same non-owning-reference
    // pattern used throughout this codebase (RobotRuntime, HardwareEventSource, etc.).
    explicit ScriptedCommandEventSource(const CommandScript& script);

    // Advances this source's notion of "now" to `cycle`. Must be called
    // with non-decreasing values by the caller (the live scripting
    // orchestrator owns cycle progression - see ScriptedLiveRuntimeRunner);
    // this class only compares against the most recently set cycle, it
    // does not enforce monotonicity itself.
    void setCurrentCycle(std::size_t cycle);

    // Returns the next unconsumed entry whose scheduled cycle is <= the
    // most recent setCurrentCycle() value ("eligible"), or std::nullopt if
    // none is eligible yet. An eligible-but-unconsumed entry remains
    // pending across calls (and across cycle advances) until returned by
    // this method exactly once - it is never dropped or replayed. Entries
    // become eligible, and are returned, in entries()' order (ascending
    // cycle, file order within a cycle), so only one event is ever
    // returned per call - this class never batches multiple entries into
    // a single pollEvent().
    std::optional<Event> pollEvent() override;

private:
    const CommandScript& script_;
    std::size_t currentCycle_ = 0;
    std::size_t nextEntryIndex_ = 0;
    std::uint64_t nextTimestampMs_ = 0;
};

} // namespace robot
