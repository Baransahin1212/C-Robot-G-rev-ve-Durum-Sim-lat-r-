#include "robot/RobotRuntime.hpp"

namespace robot
{

RobotRuntime::RobotRuntime(IPollingEventSource& source, RobotStateMachine& stateMachine, RobotController& controller)
    : source_(source)
    , stateMachine_(stateMachine)
    , controller_(controller)
{
}

RuntimeStepResult RobotRuntime::step()
{
    if (!initialized_)
    {
        controller_.applyState(stateMachine_.currentState());
        initialized_ = true;
    }

    const std::optional<Event> event = source_.pollEvent();
    if (!event.has_value())
    {
        return RuntimeStepResult::NoEvent;
    }

    if (stateMachine_.processEvent(*event) == TransitionResult::Success)
    {
        controller_.applyState(stateMachine_.currentState());
        return RuntimeStepResult::TransitionAccepted;
    }

    return RuntimeStepResult::TransitionRejected;
}

} // namespace robot
