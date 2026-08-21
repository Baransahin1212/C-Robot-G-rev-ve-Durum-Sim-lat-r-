#include "robot/visual/ReactiveObstacleAvoidance.hpp"

namespace robot::visual
{

void ReactiveObstacleAvoidance::update(bool enabled, bool triggerAvoidance, bool forwardCorridorClear) noexcept
{
    if (!enabled)
    {
        active_ = false;
        return;
    }

    if (triggerAvoidance)
    {
        active_ = true;
    }

    if (active_ && forwardCorridorClear)
    {
        active_ = false;
    }
}

bool ReactiveObstacleAvoidance::active() const noexcept
{
    return active_;
}

WheelSpeeds ReactiveObstacleAvoidance::avoidanceWheelSpeeds() const noexcept
{
    return WheelSpeeds{-kTurnWheelSpeed, kTurnWheelSpeed};
}

} // namespace robot::visual
