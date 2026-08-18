#include "robot/ScriptedCommandEventSource.hpp"

namespace robot
{

ScriptedCommandEventSource::ScriptedCommandEventSource(const CommandScript& script)
    : script_(script)
{
}

void ScriptedCommandEventSource::setCurrentCycle(std::size_t cycle)
{
    currentCycle_ = cycle;
}

std::optional<Event> ScriptedCommandEventSource::pollEvent()
{
    const std::vector<CommandScriptEntry>& entries = script_.entries();
    if (nextEntryIndex_ >= entries.size())
    {
        return std::nullopt;
    }

    const CommandScriptEntry& entry = entries[nextEntryIndex_];
    if (entry.cycle > currentCycle_)
    {
        return std::nullopt; // not yet eligible
    }

    ++nextEntryIndex_;
    return Event{entry.eventType, nextTimestampMs_++, std::nullopt};
}

} // namespace robot
