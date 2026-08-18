#pragma once

#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>

namespace robot
{

// Thrown when a sensor script file cannot be opened, or a line does not
// conform to the "<cycle> <sensor> <value>" format. Mirrors
// JsonScenarioSource/ScenarioParseError's philosophy: bad input fails
// before live execution begins, not partway through a run.
class SensorScriptParseError : public std::runtime_error
{
public:
    explicit SensorScriptParseError(const std::string& message)
        : std::runtime_error(message)
    {
    }
};

enum class SensorKind
{
    Obstacle,
    Battery,
    EmergencyStop
};

// One scripted mutation: apply `sensor` = `value` immediately before the
// runtime cycle numbered `cycle` (cycles are zero-based - see
// ScriptedLiveRuntimeRunner). `value` is 0/1 for Obstacle/EmergencyStop
// (boolean) and 0..100 for Battery (percent) - a plain int rather than
// std::variant, since every field of this struct is already fully
// determined by `sensor` and there is no behavior attached to the value
// beyond "pass it to the matching SimulatedRobotHardware setter".
struct SensorScriptEntry
{
    std::size_t cycle;
    SensorKind sensor;
    int value;
};

// Reads and validates a sensor script file eagerly, exactly like
// JsonScenarioSource does for scenario JSON: a successfully constructed
// SensorScript is guaranteed to contain only well-formed entries.
//
// Script format - one entry per non-empty, non-comment line:
//   <cycle> <sensor> <value>
// where `cycle` is a non-negative integer, `sensor` is one of
// "obstacle"/"battery"/"emergency", and `value` is "true"/"false" for
// obstacle/emergency or an integer 0..100 for battery. Blank lines and
// lines whose first non-whitespace character is '#' are ignored.
//
// entries() is sorted by cycle (ascending) using a stable sort, so entries
// sharing the same cycle retain their original file order - the ordering
// contract ScriptedLiveRuntimeRunner depends on to apply same-cycle
// mutations deterministically.
class SensorScript
{
public:
    // Throws SensorScriptParseError if the file cannot be opened or any
    // line fails to parse; the message includes a 1-based line number.
    explicit SensorScript(const std::filesystem::path& path);

    const std::vector<SensorScriptEntry>& entries() const noexcept;

private:
    std::vector<SensorScriptEntry> entries_;
};

} // namespace robot
