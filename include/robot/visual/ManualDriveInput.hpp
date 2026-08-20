#pragma once

#include "robot/visual/DifferentialDrive.hpp"

namespace robot::visual
{

// Pure, raylib-free decision function behind RobotSimulator3D's manual
// drive-mode keyboard controls (Phase 13P): takes plain key-held booleans
// (main3d.cpp is the only caller, feeding it IsKeyDown() results every
// frame) and returns the WheelSpeeds those keys command. Kept separate
// from main3d.cpp's raylib-dependent input polling specifically so this
// decision logic - X's highest priority, additive UP/DOWN/LEFT/RIGHT,
// opposite-key cancellation - can be unit-tested deterministically without
// a window (see ManualDriveInputTests.cpp), instead of only being provable
// interactively.
//
// xHeld has the highest priority: while true, both wheel speeds are
// always exactly zero regardless of every other argument, and no
// directional argument is evaluated at all - matching the deterministic
// behavior table in docs/technical-decisions.md (Phase 13P). With xHeld
// false, each held directional key additively contributes +-wheelSpeed to
// left/right, so opposite pairs (UP+DOWN, LEFT+RIGHT) cancel to zero and
// UP+LEFT/UP+RIGHT/etc. combine into an arc.
WheelSpeeds computeManualWheelSpeeds(bool upHeld, bool downHeld, bool leftHeld, bool rightHeld, bool xHeld,
                                      float wheelSpeed) noexcept;

} // namespace robot::visual
