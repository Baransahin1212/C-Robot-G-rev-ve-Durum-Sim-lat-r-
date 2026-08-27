#include "robot/visual/DockApproachArrivalEventSource.hpp"

namespace robot::visual
{

DockApproachArrivalEventSource::DockApproachArrivalEventSource(const DockApproachController& controller) noexcept
    : controller_(controller)
{
}

std::optional<Event> DockApproachArrivalEventSource::pollEvent()
{
    const bool dockedNow = controller_.state() == DockApproachState::Docked;

    std::optional<Event> result;
    if (dockedNow && !wasArrived_)
    {
        result = Event{EventType::HomeReached, 0, std::nullopt};
    }

    wasArrived_ = dockedNow;
    return result;
}

} // namespace robot::visual
