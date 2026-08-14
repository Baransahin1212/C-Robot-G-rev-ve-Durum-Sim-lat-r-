#pragma once

#include <cstdint>
#include <string_view>

#include "robot/Event.hpp"
#include "robot/RobotState.hpp"

namespace robot
{

enum class LogLevel
{
    Info,
    Warning,
    Error
};

constexpr std::string_view toString(LogLevel level) noexcept
{
    switch (level)
    {
        case LogLevel::Info: return "INFO";
        case LogLevel::Warning: return "WARNING";
        case LogLevel::Error: return "ERROR";
    }
    return "UNKNOWN";
}

// Logging abstraction the Simulator records through. Kept separate from
// RobotStateMachine so FSM decision logic never depends on how (or
// whether) a run is logged; RobotStateMachine remains the sole source of
// truth for which transitions are valid.
class ISimulationLogger
{
public:
    virtual ~ISimulationLogger() = default;

    virtual void logEventReceived(const Event& event) = 0;

    virtual void logTransitionSucceeded(std::uint64_t timestampMs, RobotState fromState, RobotState toState) = 0;

    virtual void logTransitionRejected(std::uint64_t timestampMs, RobotState state, EventType eventType) = 0;
};

} // namespace robot
