#pragma once

#include <cmath>

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Shared heading -> world-space forward-direction conversion, so movement
// (VirtualRobotHardware::update()), the forward distance sensor
// (VirtualDistanceSensor), and its renderer visualization all agree on
// exactly one convention - never independently guessed or duplicated. Matches
// VisualRobot.cpp's actual rlRotatef(headingDegrees, 0, 1, 0) call (right-
// hand rotation around +Y): heading 0 faces +Z, and increasing heading
// rotates the front marker from +Z toward +X. See
// docs/technical-decisions.md (Phase 13N/13O). Header-only and raylib-free -
// safe for VirtualRobotHardware/VirtualDistanceSensor to include.
inline Vec3 forwardDirection(const RobotPose& pose) noexcept
{
    constexpr float kPi = 3.14159265358979323846F;
    const float headingRadians = pose.headingDegrees * (kPi / 180.0F);
    return Vec3{std::sin(headingRadians), 0.0F, std::cos(headingRadians)};
}

} // namespace robot::visual
