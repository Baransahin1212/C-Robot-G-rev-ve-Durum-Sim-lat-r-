#include "robot/JsonScenarioSource.hpp"

#include <fstream>
#include <unordered_map>

#include <nlohmann/json.hpp>

namespace robot
{

namespace
{

EventType eventTypeFromString(const std::string& name)
{
    static const std::unordered_map<std::string, EventType> kEventTypesByName{
        {"SCENARIO_LOADED", EventType::ScenarioLoaded},
        {"START_MISSION", EventType::StartMission},
        {"OBSTACLE_DETECTED", EventType::ObstacleDetected},
        {"OBSTACLE_CLEARED", EventType::ObstacleCleared},
        {"BATTERY_CRITICAL", EventType::BatteryCritical},
        {"MISSION_COMPLETED", EventType::MissionCompleted},
        {"HOME_REACHED", EventType::HomeReached},
        {"EMERGENCY_STOP", EventType::EmergencyStop},
        {"INVALID_SENSOR_DATA", EventType::InvalidSensorData},
        {"RESET", EventType::Reset},
    };

    const auto it = kEventTypesByName.find(name);
    if (it == kEventTypesByName.end())
    {
        throw ScenarioParseError("Unknown event type: \"" + name + "\"");
    }
    return it->second;
}

Event parseEvent(const nlohmann::json& node)
{
    if (!node.contains("type") || !node["type"].is_string())
    {
        throw ScenarioParseError("Event is missing a string \"type\" field");
    }
    if (!node.contains("timestamp_ms") || !node["timestamp_ms"].is_number_integer())
    {
        throw ScenarioParseError("Event is missing an integer \"timestamp_ms\" field");
    }

    const EventType type = eventTypeFromString(node["type"].get<std::string>());

    const long long timestampRaw = node["timestamp_ms"].get<long long>();
    if (timestampRaw < 0)
    {
        throw ScenarioParseError("\"timestamp_ms\" must not be negative");
    }
    const std::uint64_t timestampMs = static_cast<std::uint64_t>(timestampRaw);

    std::optional<double> value;
    if (node.contains("value"))
    {
        if (!node["value"].is_number())
        {
            throw ScenarioParseError("\"value\" must be numeric when present");
        }
        value = node["value"].get<double>();
    }

    return Event{type, timestampMs, value};
}

} // namespace

JsonScenarioSource::JsonScenarioSource(const std::filesystem::path& filePath)
{
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        throw ScenarioParseError("Could not open scenario file: " + filePath.string());
    }

    nlohmann::json root;
    try
    {
        file >> root;
    }
    catch (const nlohmann::json::parse_error& e)
    {
        throw ScenarioParseError(std::string("Malformed scenario JSON: ") + e.what());
    }

    if (!root.contains("events") || !root["events"].is_array())
    {
        throw ScenarioParseError("Scenario JSON must contain an \"events\" array");
    }

    events_.reserve(root["events"].size());
    for (const auto& eventNode : root["events"])
    {
        events_.push_back(parseEvent(eventNode));
    }
}

std::optional<Event> JsonScenarioSource::nextEvent()
{
    if (index_ >= events_.size())
    {
        return std::nullopt;
    }
    return events_[index_++];
}

} // namespace robot
