#pragma once

#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot/Event.hpp"

namespace robot
{

// Thrown when a command script file cannot be opened, or a line does not
// conform to the "<cycle> <command>" format. Mirrors
// SensorScript/SensorScriptParseError's philosophy: bad input fails before
// live execution begins, not partway through a run.
class CommandScriptParseError : public std::runtime_error
{
public:
    explicit CommandScriptParseError(const std::string& message)
        : std::runtime_error(message)
    {
    }
};

// One scripted mission/operator command: raise `eventType` immediately
// once the runtime cycle numbered `cycle` is reached (cycles are
// zero-based, and eligibility persists until the event is consumed - see
// ScriptedCommandEventSource). eventType is one of the *existing*
// EventType values from Event.hpp - CommandScript introduces no new event
// vocabulary, only a text-file way to schedule the ones that are not
// sensor-derived (see CommandScript's class comment for the exact set).
struct CommandScriptEntry
{
    std::size_t cycle;
    EventType eventType;
};

// Reads and validates a command script file eagerly, exactly like
// SensorScript does for sensor scripts and JsonScenarioSource does for
// scenario JSON: a successfully constructed CommandScript is guaranteed to
// contain only well-formed entries.
//
// Script format - one entry per non-empty, non-comment line:
//   <cycle> <command>
// where `cycle` is a non-negative integer and `command` is one of:
//   scenario_loaded    -> EventType::ScenarioLoaded
//   start_mission      -> EventType::StartMission
//   mission_completed  -> EventType::MissionCompleted
//   return_home        -> EventType::ReturnHomeRequested
//   home_reached       -> EventType::HomeReached
//   reset              -> EventType::Reset
// This is deliberately the set of existing EventType values that are
// mission-lifecycle/operator commands, not sensor readings - obstacle,
// battery, and emergency-stop events remain SensorScript/
// HardwareEventSource's exclusive responsibility and are rejected here as
// unknown commands. Blank lines and lines whose first non-whitespace
// character is '#' are ignored.
//
// entries() is sorted by cycle (ascending) using a stable sort, so entries
// sharing the same cycle retain their original file order - the ordering
// contract ScriptedCommandEventSource depends on to emit same-cycle
// commands deterministically.
class CommandScript
{
public:
    // Throws CommandScriptParseError if the file cannot be opened or any
    // line fails to parse; the message includes a 1-based line number.
    explicit CommandScript(const std::filesystem::path& path);

    const std::vector<CommandScriptEntry>& entries() const noexcept;

private:
    std::vector<CommandScriptEntry> entries_;
};

} // namespace robot
