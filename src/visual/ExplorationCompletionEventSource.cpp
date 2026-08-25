#include "robot/visual/ExplorationCompletionEventSource.hpp"

namespace robot::visual
{

ExplorationCompletionEventSource::ExplorationCompletionEventSource(const ExplorationCompletionSignal& signal) noexcept
    : signal_(signal)
{
}

std::optional<Event> ExplorationCompletionEventSource::pollEvent()
{
    const bool completeNow = signal_.complete;

    std::optional<Event> result;
    if (completeNow && !wasComplete_)
    {
        result = Event{EventType::ReturnHomeRequested, 0, std::nullopt};
    }

    wasComplete_ = completeNow;
    return result;
}

} // namespace robot::visual
