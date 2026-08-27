#include "robot/visual/DockLaneObstacleFilter.hpp"

namespace robot::visual
{

DockLaneObstacleFilter::DockLaneObstacleFilter(IPollingEventSource& inner) noexcept
    : inner_(inner)
{
}

void DockLaneObstacleFilter::setSuppressed(bool suppressed) noexcept
{
    suppressed_ = suppressed;
}

std::optional<Event> DockLaneObstacleFilter::pollEvent()
{
    std::optional<Event> event = inner_.pollEvent();
    if (suppressed_ && event.has_value() && event->type == EventType::ObstacleDetected)
    {
        return std::nullopt;
    }
    return event;
}

} // namespace robot::visual
