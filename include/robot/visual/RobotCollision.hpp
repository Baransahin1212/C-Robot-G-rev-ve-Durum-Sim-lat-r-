#pragma once

#include <cmath>
#include <vector>

#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

// Conservative circular collision footprint for the robot body (Phase
// 13P), derived from RobotDimensions::kBodyWidth/kBodyLength
// (VisualRobot.hpp) - the smallest circle that fully encloses the
// rectangular body regardless of heading, so this collision test never
// needs to know the robot's orientation (rotation-independent by
// construction). Equals 0.5F exactly for this project's robot dimensions
// (sqrt(0.3^2 + 0.4^2)). Not constexpr only because std::sqrt is not
// constexpr in C++17; it is still computed once, from RobotDimensions,
// never hand-duplicated.
inline const float kRobotCollisionRadius =
    std::sqrt(((RobotDimensions::kBodyWidth / 2.0F) * (RobotDimensions::kBodyWidth / 2.0F)) +
              ((RobotDimensions::kBodyLength / 2.0F) * (RobotDimensions::kBodyLength / 2.0F)));

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
