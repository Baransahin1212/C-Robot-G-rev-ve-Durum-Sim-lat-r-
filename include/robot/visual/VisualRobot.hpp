#pragma once

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Fixed world-unit dimensions of the small two-wheel mobile robot model,
// per Phase 13M's brief. Named constants so other code (Renderer3D, and
// a future movement phase) has one source of truth instead of scattered
// magic numbers.
namespace RobotDimensions
{
inline constexpr float kBodyWidth = 0.60F;
inline constexpr float kBodyLength = 0.80F;
inline constexpr float kBodyHeight = 0.25F;
inline constexpr float kWheelRadius = 0.15F;
inline constexpr float kWheelThickness = 0.08F;
} // namespace RobotDimensions

// Draws the two-wheel mobile robot model (body, left/right wheels, and a
// small front heading marker) at `pose`, oriented by
// `pose.headingDegrees` as a rotation around the world Y axis (0 = facing
// +Z). Must be called between BeginMode3D()/EndMode3D() - it only draws,
// it owns no window/camera state of its own, and it has no knowledge of
// RobotStateMachine or any FSM concept: `pose` is plain position +
// heading, nothing more.
void drawVisualRobot(const RobotPose& pose);

} // namespace robot::visual
