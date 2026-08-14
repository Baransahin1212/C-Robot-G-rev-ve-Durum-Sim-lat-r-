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

class RobotStateMachine
{
public:
    RobotStateMachine();

    RobotState currentState() const noexcept;

    TransitionResult processEvent(const Event& event);

private:
    RobotState state_;

    // Valid only while state_ == WaitingForObstacleClear; remembers whether
    // the obstacle was hit while Moving or while ReturningHome so
    // ObstacleCleared can resume the correct state.
    RobotState resumeState_;
};

} // namespace robot
