#include "robot/CommandScript.hpp"

#include <algorithm>
#include <fstream>
#include <optional>
#include <sstream>
#include <unordered_map>

namespace robot
{

namespace
{

std::string LineError(std::size_t lineNumber, const std::string& message)
{
    return "command script line " + std::to_string(lineNumber) + ": " + message;
}

// Deliberately a separate, private helper from SensorScript.cpp's
// ParseCycle()/Application.cpp's parseCycleCount() - robot_command_script
// must not depend on robot_sensor_script or robot_app, and this is small
// enough that duplicating it keeps the libraries decoupled. Same rules: no
// leading '-', full-string consumption, and range/exception safety.
std::optional<std::size_t> ParseCycle(const std::string& token)
{
    if (token.empty() || token.front() == '-')
    {
        return std::nullopt;
    }

    std::size_t consumed = 0;
    unsigned long long value = 0;
    try
    {
        value = std::stoull(token, &consumed);
    }
    catch (const std::exception&)
    {
        return std::nullopt;
    }

    if (consumed != token.size())
    {
        return std::nullopt;
    }

    return static_cast<std::size_t>(value);
}

std::optional<EventType> CommandFromString(const std::string& name)
{
    static const std::unordered_map<std::string, EventType> kCommandsByName{
        {"scenario_loaded", EventType::ScenarioLoaded},
        {"start_mission", EventType::StartMission},
        {"mission_completed", EventType::MissionCompleted},
        {"home_reached", EventType::HomeReached},
        {"reset", EventType::Reset},
    };

    const auto it = kCommandsByName.find(name);
    if (it == kCommandsByName.end())
    {
        return std::nullopt;
    }
    return it->second;
}

std::vector<std::string> Tokenize(const std::string& line)
{
    std::vector<std::string> tokens;
    std::istringstream stream(line);
    std::string token;
    while (stream >> token)
    {
        tokens.push_back(token);
    }
    return tokens;
}

CommandScriptEntry ParseLine(const std::string& content, std::size_t lineNumber)
{
    const std::vector<std::string> tokens = Tokenize(content);

    if (tokens.size() < 2)
    {
        throw CommandScriptParseError(LineError(lineNumber, "expected \"<cycle> <command>\", got too few tokens"));
    }
    if (tokens.size() > 2)
    {
        throw CommandScriptParseError(
            LineError(lineNumber, "unexpected trailing tokens after \"<cycle> <command>\""));
    }

    const std::optional<std::size_t> cycle = ParseCycle(tokens[0]);
    if (!cycle.has_value())
    {
        throw CommandScriptParseError(LineError(lineNumber, "invalid cycle \"" + tokens[0] + "\""));
    }

    const std::optional<EventType> eventType = CommandFromString(tokens[1]);
    if (!eventType.has_value())
    {
        throw CommandScriptParseError(LineError(lineNumber, "unknown command \"" + tokens[1] + "\""));
    }

    return CommandScriptEntry{*cycle, *eventType};
}

} // namespace

CommandScript::CommandScript(const std::filesystem::path& path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        throw CommandScriptParseError("Could not open command script file: " + path.string());
    }

    std::vector<CommandScriptEntry> parsed;
    std::string line;
    std::size_t lineNumber = 0;
    while (std::getline(file, line))
    {
        ++lineNumber;

        const std::size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos)
        {
            continue; // blank line
        }
        if (line[start] == '#')
        {
            continue; // comment line
        }
        const std::size_t end = line.find_last_not_of(" \t\r\n");
        const std::string content = line.substr(start, end - start + 1);

        parsed.push_back(ParseLine(content, lineNumber));
    }

    // Stable sort by cycle only: entries already appear in file order, so
    // a stable sort preserves that order for entries sharing a cycle,
    // satisfying the "same-cycle entries emitted in file order" contract.
    std::stable_sort(parsed.begin(), parsed.end(), [](const CommandScriptEntry& a, const CommandScriptEntry& b)
                      { return a.cycle < b.cycle; });

    entries_ = std::move(parsed);
}

const std::vector<CommandScriptEntry>& CommandScript::entries() const noexcept
{
    return entries_;
}

} // namespace robot
