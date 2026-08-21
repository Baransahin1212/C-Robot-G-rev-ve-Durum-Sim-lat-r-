#include "robot/visual/HomeArrivalEventSource.hpp"

namespace robot::visual
{

HomeArrivalEventSource::HomeArrivalEventSource(const HomeNavigator& navigator) noexcept
    : navigator_(navigator)
{
}

std::optional<Event> HomeArrivalEventSource::pollEvent()
{
    const bool arrivedNow = navigator_.state() == HomeNavigationState::Arrived;

    std::optional<Event> result;
    if (arrivedNow && !wasArrived_)
    {
        result = Event{EventType::HomeReached, 0, std::nullopt};
    }

    wasArrived_ = arrivedNow;
    return result;
}

} // namespace robot::visual
