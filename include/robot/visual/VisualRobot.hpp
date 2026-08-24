#pragma once

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Fixed world-unit dimensions of the small two-wheel mobile robot model,
// per Phase 13M's brief. Named constants so other code (Renderer3D, and
// a future movement phase) has one source of truth instead of scattered
// magic numbers.
//
// Phase 13W final workspace redesign: rescaled from the original
// 0.60x0.80 down to ~0.40x0.50 - human validation of the first 8x4-desk
// pass found the robot still read as oversized next to the (also
// rescaled) monitor/keyboard/mouse, and its collision footprint left too
// little clearance in the desk's own corridors. At this project's
// ~20cm-per-world-unit design scale (docs/technical-decisions.md, Phase
// 13W), 0.40x0.50 corresponds to an ~8x10cm miniature body - every other
// dimension below is scaled by the same ~2/3 factor from its own
// original value, kept proportionate rather than picked independently.
namespace RobotDimensions
{
inline constexpr float kBodyWidth = 0.40F;
inline constexpr float kBodyLength = 0.50F;
inline constexpr float kBodyHeight = 0.16F;
inline constexpr float kWheelRadius = 0.10F;
inline constexpr float kWheelThickness = 0.05F;
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
