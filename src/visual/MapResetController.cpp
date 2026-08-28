#include "robot/visual/MapResetController.hpp"

namespace robot::visual
{

namespace
{
bool resetAllowedForState(RobotState state) noexcept
{
    return state == RobotState::Idle || state == RobotState::Ready;
}
} // namespace

void MapResetController::update(float deltaSeconds) noexcept
{
    if (!confirmationPending_)
    {
        return;
    }
    elapsedSinceFirstPressSeconds_ += deltaSeconds;
    if (elapsedSinceFirstPressSeconds_ >= kConfirmationWindowSeconds)
    {
        confirmationPending_ = false;
    }
}

MapResetRequestOutcome MapResetController::requestKeyPress(RobotState state) noexcept
{
    if (!resetAllowedForState(state))
    {
        confirmationPending_ = false;
        elapsedSinceFirstPressSeconds_ = 0.0F;
        return MapResetRequestOutcome::RejectedActiveMission;
    }

    if (confirmationPending_)
    {
        confirmationPending_ = false;
        elapsedSinceFirstPressSeconds_ = 0.0F;
        return MapResetRequestOutcome::Confirmed;
    }

    confirmationPending_ = true;
    elapsedSinceFirstPressSeconds_ = 0.0F;
    return MapResetRequestOutcome::ConfirmationRequested;
}

bool MapResetController::confirmationPending() const noexcept
{
    return confirmationPending_;
}

} // namespace robot::visual
