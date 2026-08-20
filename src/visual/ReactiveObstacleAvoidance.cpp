#include "robot/visual/ReactiveObstacleAvoidance.hpp"

namespace robot::visual
{

WheelSpeeds ReactiveObstacleAvoidance::avoidanceWheelSpeeds() const noexcept
{
    return WheelSpeeds{-kTurnWheelSpeed, kTurnWheelSpeed};
}

} // namespace robot::visual
