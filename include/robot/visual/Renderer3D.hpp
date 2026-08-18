#pragma once

#include <string_view>

#include "raylib.h"

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Owns the Camera3D and draws one complete frame - ground, grid,
// obstacles, base platform, robot, and a 2D HUD overlay - for a given
// VirtualWorld. Does not create or destroy the raylib window itself; that
// is main3d's responsibility (see docs/technical-decisions.md, Phase
// 13M), so a Renderer3D must only be constructed/used while a window is
// already open. Has no knowledge of RobotStateMachine, RobotRuntime,
// RobotController, or IRobotHardware - it only ever reads VirtualWorld's
// plain pose/obstacle/base data, plus the two already-formatted display
// strings passed into renderFrame() (Phase 13N), never a FSM/hardware
// type directly. This keeps robot_visual free of any dependency on
// robot_visual_simulation - the two are sibling targets under
// RobotSimulator3D, not layered on each other.
class Renderer3D
{
public:
    Renderer3D();

    // Draws one complete frame for `world`. When `updateCamera` is true,
    // the camera is first advanced per raylib's built-in CAMERA_FREE
    // controls (mouse look, WASD, scroll zoom); when false, the camera is
    // left exactly as it was on the previous frame - main3d decides
    // `updateCamera` based on whether mouse capture (DisableCursor()) is
    // currently active, so the camera never drifts while the cursor has
    // been released for normal desktop use. `stateText`/`commandText` are
    // shown verbatim in the HUD (e.g. "Moving"/"MoveForward") - the
    // caller (main3d) is responsible for converting the real
    // RobotState/VirtualDriveCommand to text, so this class never needs
    // to know either type. Call exactly once per iteration of the main
    // render loop, between InitWindow() and CloseWindow().
    void renderFrame(const VirtualWorld& world, bool updateCamera, std::string_view stateText,
                      std::string_view commandText);

private:
    void drawScene(const VirtualWorld& world) const;
    void drawHud(const VirtualWorld& world, std::string_view stateText, std::string_view commandText) const;

    Camera3D camera_;
};

} // namespace robot::visual
