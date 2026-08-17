#include "robot/Simulator.hpp"

namespace robot
{

Simulator::Simulator(IEventSource& eventSource,
                      RobotStateMachine& stateMachine,
                      ISimulationLogger* logger,
                      RobotController* controller)
    : eventSource_(eventSource)
    , stateMachine_(stateMachine)
    , logger_(logger)
    , controller_(controller)
{
}

SimulationResult Simulator::run()
{
    SimulationResult result;

    if (controller_ != nullptr)
    {
        controller_->applyState(stateMachine_.currentState());
    }

    while (const std::optional<Event> event = eventSource_.nextEvent())
    {
        const RobotState stateBeforeEvent = stateMachine_.currentState();

        if (logger_ != nullptr)
        {
            logger_->logEventReceived(*event);
        }

        const TransitionResult transitionResult = stateMachine_.processEvent(*event);
        ++result.eventsProcessed;
        result.lastEventTimestampMs = event->timestampMs;

        if (transitionResult == TransitionResult::Success)
        {
            ++result.successfulTransitions;
            if (logger_ != nullptr)
            {
                logger_->logTransitionSucceeded(event->timestampMs, stateBeforeEvent, stateMachine_.currentState());
            }
            if (controller_ != nullptr)
            {
                controller_->applyState(stateMachine_.currentState());
            }
        }
        else
        {
            ++result.rejectedTransitions;
            if (logger_ != nullptr)
            {
                logger_->logTransitionRejected(event->timestampMs, stateBeforeEvent, event->type);
            }
        }
    }

    result.finalState = stateMachine_.currentState();
    return result;
}

} // namespace robot
