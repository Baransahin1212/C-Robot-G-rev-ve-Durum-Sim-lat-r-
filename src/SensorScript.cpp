#include "robot/SensorScript.hpp"

#include <algorithm>
#include <fstream>
#include <optional>
#include <sstream>

namespace robot
{

namespace
{

std::string LineError(std::size_t lineNumber, const std::string& message)
{
    return "sensor script line " + std::to_string(lineNumber) + ": " + message;
}

// Deliberately a separate, private helper from Application.cpp's
// parseCycleCount() - robot_sensor_script must not depend on robot_app,
// and this is small enough that duplicating it keeps the two libraries
// decoupled. Same rules: no leading '-', full-string consumption, and
// range/exception safety.
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

// Battery values may be negative in the raw token (e.g. "-5"), so a
// distinct "battery must be between 0 and 100" error can fire for
// below-range input rather than a generic "invalid value" one - unlike
// ParseCycle, a leading '-' is not rejected here.
std::optional<int> ParseInt(const std::string& token)
{
    if (token.empty())
    {
        return std::nullopt;
    }

    std::size_t consumed = 0;
    int value = 0;
    try
    {
        value = std::stoi(token, &consumed);
    }
    catch (const std::exception&)
    {
        return std::nullopt;
    }

    if (consumed != token.size())
    {
        return std::nullopt;
    }

    return value;
}

std::optional<bool> ParseBool(const std::string& token)
{
    if (token == "true")
    {
        return true;
    }
    if (token == "false")
    {
        return false;
    }
    return std::nullopt;
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

SensorScriptEntry ParseLine(const std::string& content, std::size_t lineNumber)
{
    const std::vector<std::string> tokens = Tokenize(content);

    if (tokens.size() < 3)
    {
        throw SensorScriptParseError(
            LineError(lineNumber, "expected \"<cycle> <sensor> <value>\", got too few tokens"));
    }
    if (tokens.size() > 3)
    {
        throw SensorScriptParseError(
            LineError(lineNumber, "unexpected trailing tokens after \"<cycle> <sensor> <value>\""));
    }

    const std::optional<std::size_t> cycle = ParseCycle(tokens[0]);
    if (!cycle.has_value())
    {
        throw SensorScriptParseError(LineError(lineNumber, "invalid cycle \"" + tokens[0] + "\""));
    }

    const std::string& sensorName = tokens[1];
    const std::string& valueToken = tokens[2];

    if (sensorName == "obstacle" || sensorName == "emergency")
    {
        const std::optional<bool> value = ParseBool(valueToken);
        if (!value.has_value())
        {
            throw SensorScriptParseError(
                LineError(lineNumber, sensorName + " value must be exactly \"true\" or \"false\""));
        }
        const SensorKind kind = (sensorName == "obstacle") ? SensorKind::Obstacle : SensorKind::EmergencyStop;
        return SensorScriptEntry{*cycle, kind, *value ? 1 : 0};
    }

    if (sensorName == "battery")
    {
        const std::optional<int> value = ParseInt(valueToken);
        if (!value.has_value())
        {
            throw SensorScriptParseError(LineError(lineNumber, "invalid battery value \"" + valueToken + "\""));
        }
        if (*value < 0 || *value > 100)
        {
            throw SensorScriptParseError(LineError(lineNumber, "battery must be between 0 and 100"));
        }
        return SensorScriptEntry{*cycle, SensorKind::Battery, *value};
    }

    throw SensorScriptParseError(LineError(lineNumber, "unknown sensor \"" + sensorName + "\""));
}

} // namespace

SensorScript::SensorScript(const std::filesystem::path& path)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        throw SensorScriptParseError("Could not open sensor script file: " + path.string());
    }

    std::vector<SensorScriptEntry> parsed;
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
    // satisfying the "same-cycle entries apply in file order" contract.
    std::stable_sort(parsed.begin(), parsed.end(), [](const SensorScriptEntry& a, const SensorScriptEntry& b)
                      { return a.cycle < b.cycle; });

    entries_ = std::move(parsed);
}

const std::vector<SensorScriptEntry>& SensorScript::entries() const noexcept
{
    return entries_;
}

} // namespace robot
