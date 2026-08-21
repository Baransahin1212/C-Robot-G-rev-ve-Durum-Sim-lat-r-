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

// Shared heading -> world-space RIGHT-direction conversion (Phase 13S) -
// perpendicular to forwardDirection() above, sharing the same one heading
// convention: at heading 0 (forward +Z), right is +X, matching the
// intuition that facing +Z with +X to the right (this project's X/Z-on-
// the-ground, Y-up convention) puts the robot's right side toward +X.
// Derived as forwardDirection() evaluated at (headingDegrees + 90):
// sin(theta+90) = cos(theta), cos(theta+90) = -sin(theta). Used by
// VirtualCliffSensor to place the four corner sensors relative to the
// robot's rotated footprint - never independently reimplemented
// elsewhere.
inline Vec3 rightDirection(const RobotPose& pose) noexcept
{
    constexpr float kPi = 3.14159265358979323846F;
    const float headingRadians = pose.headingDegrees * (kPi / 180.0F);
    return Vec3{std::cos(headingRadians), 0.0F, -std::sin(headingRadians)};
}

// Normalizes a heading into [0, 360) - fmod alone can return a negative
// result for a negative input, so a single conditional correction follows
// it (Phase 13S bugfix). Matches DifferentialDrive.cpp's own private
// wrapHeadingDegrees() exactly; duplicated here (not shared across that
// visual-math/kinematics boundary) because DifferentialDrive.cpp keeps
// its copy file-local - a third caller needing this from outside either
// file is why this one lives in VisualMath.hpp instead.
inline float normalizeHeadingDegrees(float degrees) noexcept
{
    float wrapped = std::fmod(degrees, 360.0F);
    if (wrapped < 0.0F)
    {
        wrapped += 360.0F;
    }
    return wrapped;
}

// Inverse of forwardDirection(): converts a world-space X/Z direction
// into the heading in degrees that would make forwardDirection() return
// (a normalized version of) it, using this project's one convention (0 =
// +Z, 90 = +X). Since forwardDirection(theta) = (sin(theta), cos(theta)),
// the inverse is theta = atan2(dirX, dirZ) - note the argument order
// (X first, then Z), the opposite of the usual atan2(y, x) reading, since
// this project's "sin component" is X and "cos component" is Z, not the
// other way around. A near-zero-length direction (both components below
// a small epsilon - e.g. a target exactly at the robot's own position)
// has no meaningful heading; rather than let atan2(0, 0) silently return
// a well-defined-but-meaningless 0.0F that looks identical to a real
// heading of 0, this still returns 0.0F but callers with a degenerate
// direction should treat the result as "no preferred heading," not "face
// +Z." Never returns NaN/Inf.
inline float headingDegreesFromDirection(float dirX, float dirZ) noexcept
{
    constexpr float kPi = 3.14159265358979323846F;
    constexpr float kDirectionEpsilon = 1.0e-6F;
    if (std::fabs(dirX) < kDirectionEpsilon && std::fabs(dirZ) < kDirectionEpsilon)
    {
        return 0.0F;
    }
    return std::atan2(dirX, dirZ) * (180.0F / kPi);
}

// Shortest signed angular error from `fromDegrees` to `toDegrees`, in
// degrees, in the range (-180, 180]. Positive means `toDegrees` is
// reached by INCREASING heading - matching this project's one turn-
// direction convention (omega > 0 increases headingDegrees, per
// DifferentialDrive's equations and ReactiveObstacleAvoidance/
// TableEdgeSafetyController's own turn wheel-speed signs), so a caller
// can directly use the sign of this result to pick which way to turn.
// Never affected by 0/360 wraparound: e.g.
// shortestSignedHeadingErrorDegrees(350, 10) is +20, not -340.
inline float shortestSignedHeadingErrorDegrees(float fromDegrees, float toDegrees) noexcept
{
    float error = normalizeHeadingDegrees(toDegrees - fromDegrees);
    if (error > 180.0F)
    {
        error -= 360.0F;
    }
    return error;
}

} // namespace robot::visual
