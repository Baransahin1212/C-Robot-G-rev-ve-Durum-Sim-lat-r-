#include "robot/visual/WaypointArrivalEventSource.hpp"

namespace robot::visual
{

WaypointArrivalEventSource::WaypointArrivalEventSource(const WaypointNavigator& navigator) noexcept
    : navigator_(navigator)
{
}

std::optional<Event> WaypointArrivalEventSource::pollEvent()
{
    const bool arrivedNow = navigator_.state() == WaypointNavigatorState::Arrived;

    std::optional<Event> result;
    if (arrivedNow && !wasArrived_)
    {
        result = Event{EventType::HomeReached, 0, std::nullopt};
    }

    wasArrived_ = arrivedNow;
    return result;
}

} // namespace robot::visual
