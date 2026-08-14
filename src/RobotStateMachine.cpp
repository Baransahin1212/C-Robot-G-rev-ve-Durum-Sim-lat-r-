#include "robot/RobotStateMachine.hpp"

namespace robot
{

RobotStateMachine::RobotStateMachine()
    : state_(RobotState::Idle)
    , resumeState_(RobotState::Idle)
{
}

RobotState RobotStateMachine::currentState() const noexcept
{
    return state_;
}

TransitionResult RobotStateMachine::processEvent(const Event& event)
{
    switch (state_)
    {
        case RobotState::Idle:
            if (event.type == EventType::ScenarioLoaded)
            {
                state_ = RobotState::Ready;
                return TransitionResult::Success;
            }
            break;

        case RobotState::Ready:
            if (event.type == EventType::StartMission)
            {
                state_ = RobotState::Moving;
                return TransitionResult::Success;
            }
            break;

        case RobotState::Moving:
            switch (event.type)
            {
                case EventType::ObstacleDetected:
                    resumeState_ = RobotState::Moving;
                    state_ = RobotState::WaitingForObstacleClear;
                    return TransitionResult::Success;
                case EventType::BatteryCritical:
                    state_ = RobotState::ReturningHome;
                    return TransitionResult::Success;
                case EventType::MissionCompleted:
                    state_ = RobotState::Completed;
                    return TransitionResult::Success;
                case EventType::EmergencyStop:
                    state_ = RobotState::EmergencyStopped;
                    return TransitionResult::Success;
                case EventType::InvalidSensorData:
                    state_ = RobotState::Error;
                    return TransitionResult::Success;
                default:
                    break;
            }
            break;

        case RobotState::WaitingForObstacleClear:
            switch (event.type)
            {
                case EventType::ObstacleCleared:
                    state_ = resumeState_;
                    return TransitionResult::Success;
                case EventType::EmergencyStop:
                    state_ = RobotState::EmergencyStopped;
                    return TransitionResult::Success;
                case EventType::InvalidSensorData:
                    state_ = RobotState::Error;
                    return TransitionResult::Success;
                default:
                    break;
            }
            break;

        case RobotState::ReturningHome:
            switch (event.type)
            {
                case EventType::HomeReached:
                    state_ = RobotState::Aborted;
                    return TransitionResult::Success;
                case EventType::ObstacleDetected:
                    resumeState_ = RobotState::ReturningHome;
                    state_ = RobotState::WaitingForObstacleClear;
                    return TransitionResult::Success;
                case EventType::EmergencyStop:
                    state_ = RobotState::EmergencyStopped;
                    return TransitionResult::Success;
                case EventType::InvalidSensorData:
                    state_ = RobotState::Error;
                    return TransitionResult::Success;
                default:
                    break;
            }
            break;

        case RobotState::EmergencyStopped:
            if (event.type == EventType::Reset)
            {
                state_ = RobotState::Idle;
                return TransitionResult::Success;
            }
            break;

        case RobotState::Error:
            if (event.type == EventType::Reset)
            {
                state_ = RobotState::Idle;
                return TransitionResult::Success;
            }
            break;

        case RobotState::Completed:
        case RobotState::Aborted:
            // Terminal states for now; no transitions defined out of them.
            break;
    }

    return TransitionResult::InvalidTransition;
}

} // namespace robot
