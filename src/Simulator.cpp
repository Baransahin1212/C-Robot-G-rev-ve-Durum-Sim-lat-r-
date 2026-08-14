#include "robot/Simulator.hpp"

namespace robot
{

Simulator::Simulator(IEventSource& eventSource, RobotStateMachine& stateMachine)
    : eventSource_(eventSource)
    , stateMachine_(stateMachine)
{
}

SimulationResult Simulator::run()
{
    SimulationResult result;

    while (const std::optional<Event> event = eventSource_.nextEvent())
    {
        const TransitionResult transitionResult = stateMachine_.processEvent(*event);
        ++result.eventsProcessed;

        if (transitionResult == TransitionResult::Success)
        {
            ++result.successfulTransitions;
        }
        else
        {
            ++result.rejectedTransitions;
        }
    }

    result.finalState = stateMachine_.currentState();
    return result;
}

} // namespace robot
