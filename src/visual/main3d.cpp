#include "raylib.h"

#include "robot/visual/Renderer3D.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr int kTargetFps = 60;
} // namespace

// Thin application-lifecycle composition root for the 3D visual
// simulator: owns the raylib window, fullscreen/mouse-capture state, and
// the render loop only. Phase 13M is visualization foundation - it
// constructs no RobotStateMachine, RobotRuntime, or RobotController;
// VirtualWorld's demo scene is stationary and Renderer3D draws it every
// frame.
int main()
{
    InitWindow(kWindowWidth, kWindowHeight, "Robot Simulator 3D");
    SetTargetFPS(kTargetFps);

    // Borderless fullscreen on the current monitor - preferred over
    // exclusive ToggleFullscreen() for this desktop simulator, since it
    // keeps normal window management (alt-tab, other monitors) working.
    ToggleBorderlessWindowed();

    robot::visual::VirtualWorld world;
    robot::visual::Renderer3D renderer;

    // Camera mouse capture starts enabled, so the mouse immediately
    // drives the camera without an extra keypress; DisableCursor() also
    // keeps the OS cursor from escaping the window while captured.
    bool cameraCaptured = true;
    DisableCursor();

    while (!WindowShouldClose())
    {
        if (IsKeyPressed(KEY_TAB))
        {
            cameraCaptured = !cameraCaptured;
            if (cameraCaptured)
            {
                DisableCursor();
            }
            else
            {
                EnableCursor();
            }
        }

        if (IsKeyPressed(KEY_F11))
        {
            ToggleBorderlessWindowed();
        }

        renderer.renderFrame(world, cameraCaptured);
    }

    CloseWindow();
    return 0;
}
