#include "robot/visual/DockApproachArrivalEventSource.hpp"

namespace robot::visual
{

DockApproachArrivalEventSource::DockApproachArrivalEventSource(const DockApproachController& controller) noexcept
    : controller_(controller)
{
}

std::optional<Event> DockApproachArrivalEventSource::pollEvent()
{
    const bool arrivedNow = controller_.state() == DockApproachState::Arrived;

    std::optional<Event> result;
    if (arrivedNow && !wasArrived_)
    {
        result = Event{EventType::HomeReached, 0, std::nullopt};
    }

    wasArrived_ = arrivedNow;
    return result;
}

} // namespace robot::visual
