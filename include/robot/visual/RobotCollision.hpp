#pragma once

#include <cmath>
#include <vector>

#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

// Small named safety margin added on top of the body's own half-diagonal
// below (Phase 13W final workspace redesign) - the enclosing circle
// alone is already conservative for ROTATION (it contains the rectangle
// at every heading), but adds none of its own margin for approach/sensor
// noise; this keeps kRobotCollisionRadius "half-diagonal plus a little,"
// per the brief's own derivation, rather than the bare geometric minimum.
inline constexpr float kCollisionSafetyMargin = 0.03F;

// Conservative circular collision footprint for the robot body (Phase
// 13P), derived from RobotDimensions::kBodyWidth/kBodyLength
// (VisualRobot.hpp) - the smallest circle that fully encloses the
// rectangular body regardless of heading (so this collision test never
// needs to know the robot's orientation - rotation-independent by
// construction), plus kCollisionSafetyMargin above. Phase 13W final
// workspace redesign: ~0.35F for this project's rescaled 0.40x0.50 robot
// (half-diagonal sqrt(0.20^2 + 0.25^2) =~ 0.3202F, plus the 0.03F
// margin) - was 0.5F exactly (sqrt(0.3^2 + 0.4^2), no margin) for the
// original 0.60x0.80 body. Not constexpr only because std::sqrt is not
// constexpr in C++17; it is still computed once, from RobotDimensions,
// never hand-duplicated.
inline const float kRobotCollisionRadius =
    std::sqrt(((RobotDimensions::kBodyWidth / 2.0F) * (RobotDimensions::kBodyWidth / 2.0F)) +
              ((RobotDimensions::kBodyLength / 2.0F) * (RobotDimensions::kBodyLength / 2.0F))) +
    kCollisionSafetyMargin;

// True when a robot centered at `position` (only X/Z are read - this is a
// ground-plane check) overlaps any ENABLED obstacle in `obstacles`, via a
// circle-vs-AABB test in the X/Z plane (circle: kRobotCollisionRadius
// around `position`; AABB: each obstacle's X/Z footprint). Disabled
// obstacles never collide, matching VirtualDistanceSensor's own
// enabled-only convention.
//
// Raylib-free; has no VirtualRobotHardware/DifferentialDrive/FSM/Event/
// IRobotHardware knowledge of its own beyond the plain Vec3/BoxObstacle
// data passed in - DifferentialDrive itself stays entirely unaware this
// exists. Only VirtualRobotHardware::update() calls it, to validate a
// proposed pose before committing it to VirtualWorld. See
// docs/technical-decisions.md (Phase 13P).
bool robotPositionCollidesWithObstacles(const Vec3& position, const std::vector<BoxObstacle>& obstacles) noexcept;

} // namespace robot::visual
