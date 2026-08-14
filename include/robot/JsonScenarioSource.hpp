#pragma once

#include <cstddef>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot/Event.hpp"
#include "robot/IEventSource.hpp"

namespace robot
{

// Thrown when a scenario file cannot be opened, is not valid JSON, or does
// not follow the scenario schema. This represents an inability to construct
// a valid scenario, not an ordinary FSM transition outcome, so it uses
// exceptions rather than a result enum.
class ScenarioParseError : public std::runtime_error
{
public:
    explicit ScenarioParseError(const std::string& message)
        : std::runtime_error(message)
    {
    }
};

// Reads a JSON scenario file and exposes its events as an IEventSource.
// The entire file is parsed and validated eagerly in the constructor, so a
// successfully constructed JsonScenarioSource is guaranteed to yield only
// well-formed events - nextEvent() itself cannot fail partway through.
//
// This class only converts JSON -> Event objects. It does not evaluate FSM
// rules, change robot state, or log mission decisions.
class JsonScenarioSource : public IEventSource
{
public:
    // Throws ScenarioParseError if the file cannot be opened or does not
    // conform to the scenario schema.
    explicit JsonScenarioSource(const std::filesystem::path& filePath);

    std::optional<Event> nextEvent() override;

private:
    std::vector<Event> events_;
    std::size_t index_ = 0;
};

} // namespace robot
