#include "robot/visual/ManualDriveInput.hpp"

namespace robot::visual
{

WheelSpeeds computeManualWheelSpeeds(bool upHeld, bool downHeld, bool leftHeld, bool rightHeld, bool xHeld,
                                      float wheelSpeed) noexcept
{
    if (xHeld)
    {
        // Highest priority - do not evaluate any directional key this
        // frame, so a previous command can never remain latched.
        return WheelSpeeds{0.0F, 0.0F};
    }

    float left = 0.0F;
    float right = 0.0F;

    if (upHeld)
    {
        left += wheelSpeed;
        right += wheelSpeed;
    }
    if (downHeld)
    {
        left -= wheelSpeed;
        right -= wheelSpeed;
    }
    if (leftHeld)
    {
        left -= wheelSpeed;
        right += wheelSpeed;
    }
    if (rightHeld)
    {
        left += wheelSpeed;
        right -= wheelSpeed;
    }

    return WheelSpeeds{left, right};
}

} // namespace robot::visual
