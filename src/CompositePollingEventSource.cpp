#include "robot/CompositePollingEventSource.hpp"

namespace robot
{

CompositePollingEventSource::CompositePollingEventSource(IPollingEventSource& commandSource,
                                                           IPollingEventSource& hardwareSource)
    : commandSource_(commandSource)
    , hardwareSource_(hardwareSource)
{
}

std::optional<Event> CompositePollingEventSource::pollEvent()
{
    if (const std::optional<Event> commandEvent = commandSource_.pollEvent())
    {
        return commandEvent;
    }
    return hardwareSource_.pollEvent();
}

} // namespace robot
