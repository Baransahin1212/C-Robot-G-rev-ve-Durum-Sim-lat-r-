#pragma once

#include <string_view>

namespace robot
{

enum class RobotState
{
    Idle,
    Ready,
    Moving,
    WaitingForObstacleClear,
    ReturningHome,
    Completed,
    Aborted,
    EmergencyStopped,
    Error
};

constexpr std::string_view toString(RobotState state) noexcept
{
    switch (state)
    {
        case RobotState::Idle: return "Idle";
        case RobotState::Ready: return "Ready";
        case RobotState::Moving: return "Moving";
        case RobotState::WaitingForObstacleClear: return "WaitingForObstacleClear";
        case RobotState::ReturningHome: return "ReturningHome";
        case RobotState::Completed: return "Completed";
        case RobotState::Aborted: return "Aborted";
        case RobotState::EmergencyStopped: return "EmergencyStopped";
        case RobotState::Error: return "Error";
    }
    return "Unknown";
}

} // namespace robot
