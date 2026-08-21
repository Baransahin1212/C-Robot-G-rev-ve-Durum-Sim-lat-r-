#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace robot
{

enum class EventType
{
    ScenarioLoaded,
    StartMission,
    ObstacleDetected,
    ObstacleCleared,
    BatteryCritical,
    MissionCompleted,
    ReturnHomeRequested,
    StopTaskRequested,
    HomeReached,
    EmergencyStop,
    InvalidSensorData,
    Reset
};

constexpr std::string_view toString(EventType type) noexcept
{
    switch (type)
    {
        case EventType::ScenarioLoaded: return "ScenarioLoaded";
        case EventType::StartMission: return "StartMission";
        case EventType::ObstacleDetected: return "ObstacleDetected";
        case EventType::ObstacleCleared: return "ObstacleCleared";
        case EventType::BatteryCritical: return "BatteryCritical";
        case EventType::MissionCompleted: return "MissionCompleted";
        case EventType::ReturnHomeRequested: return "ReturnHomeRequested";
        case EventType::StopTaskRequested: return "StopTaskRequested";
        case EventType::HomeReached: return "HomeReached";
        case EventType::EmergencyStop: return "EmergencyStop";
        case EventType::InvalidSensorData: return "InvalidSensorData";
        case EventType::Reset: return "Reset";
    }
    return "Unknown";
}

// A single simulated occurrence fed into the robot's state machine.
// `value` carries an optional numeric payload (e.g. battery percentage for
// BatteryCritical) without needing a separate field per event type.
struct Event
{
    EventType type;
    std::uint64_t timestampMs;
    std::optional<double> value;
};

} // namespace robot
