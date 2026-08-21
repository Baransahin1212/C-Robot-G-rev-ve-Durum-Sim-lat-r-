#include "robot/visual/HomeNavigator.hpp"

#include <cmath>

#include "robot/visual/VisualMath.hpp"

namespace robot::visual
{

HomeNavigationOutput HomeNavigator::update(const RobotPose& pose, const BasePlatform& base, bool enabled) noexcept
{
    if (!enabled)
    {
        state_ = HomeNavigationState::Inactive;
        return HomeNavigationOutput{WheelSpeeds{0.0F, 0.0F}, state_, 0.0F, 0.0F, 0.0F, false};
    }

    // Continuously recomputed every call from the CURRENT pose - never a
    // cached target (see this class's header docs for why).
    const float dx = base.position.x - pose.position.x;
    const float dz = base.position.z - pose.position.z;
    const float distance = std::sqrt((dx * dx) + (dz * dz));
    const float targetHeading = normalizeHeadingDegrees(headingDegreesFromDirection(dx, dz));
    const float headingError = shortestSignedHeadingErrorDegrees(pose.headingDegrees, targetHeading);
    const float absHeadingError = std::fabs(headingError);

    if (state_ == HomeNavigationState::Inactive)
    {
        // Just (re-)enabled this call - default to Aligning; the arrival
        // check immediately below upgrades this to Arrived instead if the
        // robot is already close enough, so a fresh/resumed navigation
        // session never rotates or drives away first ("base start
        // condition").
        state_ = HomeNavigationState::Aligning;
    }

    if (distance <= kHomeArrivalRadius)
    {
        state_ = HomeNavigationState::Arrived;
    }
    else if (state_ == HomeNavigationState::Aligning && absHeadingError <= kStartDrivingHeadingToleranceDegrees)
    {
        state_ = HomeNavigationState::Driving;
    }
    else if (state_ == HomeNavigationState::Driving && absHeadingError >= kStopDrivingHeadingToleranceDegrees)
    {
        state_ = HomeNavigationState::Aligning;
    }

    WheelSpeeds speeds{0.0F, 0.0F};
    switch (state_)
    {
        case HomeNavigationState::Aligning:
        {
            // Shortest-path turn direction - identical convention to
            // TableEdgeSafetyController::recoveryWheelSpeeds()'s own
            // Turning case: positive error turns the same direction
            // ReactiveObstacleAvoidance always uses (omega > 0); negative
            // error turns the opposite way.
            const float directionSign = (headingError >= 0.0F) ? 1.0F : -1.0F;
            speeds = WheelSpeeds{-kNavigationTurnSpeed * directionSign, kNavigationTurnSpeed * directionSign};
            break;
        }
        case HomeNavigationState::Driving:
            speeds = WheelSpeeds{kNavigationForwardSpeed, kNavigationForwardSpeed};
            break;
        case HomeNavigationState::Arrived:
        case HomeNavigationState::Inactive:
            speeds = WheelSpeeds{0.0F, 0.0F};
            break;
    }

    return HomeNavigationOutput{speeds, state_, distance, targetHeading, headingError,
                                 state_ == HomeNavigationState::Arrived};
}

void HomeNavigator::reset() noexcept
{
    state_ = HomeNavigationState::Inactive;
}

HomeNavigationState HomeNavigator::state() const noexcept
{
    return state_;
}

} // namespace robot::visual
