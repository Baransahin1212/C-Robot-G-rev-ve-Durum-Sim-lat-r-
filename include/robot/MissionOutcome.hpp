#pragma once

#include <string_view>

#include "robot/RobotState.hpp"

namespace robot
{

enum class MissionOutcome
{
    Completed,
    Aborted,
    EmergencyStopped,
    Error,
    Incomplete
};

constexpr std::string_view toString(MissionOutcome outcome) noexcept
{
    switch (outcome)
    {
        case MissionOutcome::Completed: return "Completed";
        case MissionOutcome::Aborted: return "Aborted";
        case MissionOutcome::EmergencyStopped: return "EmergencyStopped";
        case MissionOutcome::Error: return "Error";
        case MissionOutcome::Incomplete: return "Incomplete";
    }
    return "Unknown";
}

// Central mapping from the FSM's final RobotState to a reporting-level
// MissionOutcome. This is a categorization for human-facing reports, not
// FSM transition logic - RobotStateMachine remains the sole authority on
// which states and transitions are valid; this function only interprets an
// already-final state.
constexpr MissionOutcome missionOutcomeFromState(RobotState state) noexcept
{
    switch (state)
    {
        case RobotState::Completed: return MissionOutcome::Completed;
        case RobotState::Aborted: return MissionOutcome::Aborted;
        case RobotState::EmergencyStopped: return MissionOutcome::EmergencyStopped;
        case RobotState::Error: return MissionOutcome::Error;
        case RobotState::Idle:
        case RobotState::Ready:
        case RobotState::Moving:
        case RobotState::WaitingForObstacleClear:
        case RobotState::ReturningHome:
            return MissionOutcome::Incomplete;
    }
    return MissionOutcome::Incomplete;
}

} // namespace robot
