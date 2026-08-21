#pragma once

#include "robot/Event.hpp"
#include "robot/RobotState.hpp"

namespace robot
{

enum class TransitionResult
{
    Success,
    InvalidTransition
};

// Phase 13T manual-validation bugfix: WHY the machine entered
// ReturningHome - the physical ReturningHome state alone is not enough to
// decide where HomeReached should lead, because it represents two
// genuinely different mission outcomes (see RobotStateMachine::
// processEvent()'s ReturningHome + HomeReached case). Deliberately a
// plain robot-domain enum, not an EventType - it is derived FSM context,
// never something a caller sends in as an Event.
enum class ReturnHomeReason
{
    // Not currently returning home (the default outside ReturningHome).
    None,

    // Automatic mission-abort trigger (e.g. BatteryCritical) - the
    // original, pre-Phase-13T meaning of "the robot is heading home,"
    // where arriving genuinely represents the mission ending abnormally.
    MissionAbort,

    // Explicit operator/user intent (ReturnHomeRequested) - an
    // interactive "come home" command, where arriving should leave the
    // robot in a reusable, non-terminal state, not a mission-failure
    // state.
    UserRequest
};

constexpr std::string_view toString(ReturnHomeReason reason) noexcept
{
    switch (reason)
    {
        case ReturnHomeReason::None: return "None";
        case ReturnHomeReason::MissionAbort: return "MissionAbort";
        case ReturnHomeReason::UserRequest: return "UserRequest";
    }
    return "Unknown";
}

class RobotStateMachine
{
public:
    RobotStateMachine();

    RobotState currentState() const noexcept;

    // Valid only while currentState() == ReturningHome; None otherwise
    // (including immediately after HomeReached is consumed - see
    // processEvent()). Exposed for telemetry/tests, not consulted by
    // RobotController/IRobotHardware - see ReturnHomeReason's own docs.
    ReturnHomeReason returnHomeReason() const noexcept;

    TransitionResult processEvent(const Event& event);

private:
    RobotState state_;

    // Valid only while state_ == WaitingForObstacleClear; remembers whether
    // the obstacle was hit while Moving or while ReturningHome so
    // ObstacleCleared can resume the correct state.
    RobotState resumeState_;

    // See ReturnHomeReason's own docs above.
    ReturnHomeReason returnHomeReason_;
};

} // namespace robot
