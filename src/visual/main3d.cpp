#include "raylib.h"

#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/DemoCommandSource.hpp"
#include "robot/visual/Renderer3D.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr int kTargetFps = 60;
} // namespace

// Thin application-lifecycle composition root for the 3D visual
// simulator: owns the raylib window, fullscreen/mouse-capture state, and
// the render loop only. As of Phase 13N, the on-screen robot is driven by
// the real, unmodified RobotStateMachine/RobotController/RobotRuntime -
// see docs/technical-decisions.md. DemoCommandSource delivers exactly
// ScenarioLoaded then StartMission once each, so the FSM reaches Moving
// through its real transition rules, not by main3d forcing state
// directly; RobotController then drives VirtualRobotHardware exactly as
// it would drive SimulatedRobotHardware/RealRobotHardware.
int main()
{
    InitWindow(kWindowWidth, kWindowHeight, "Robot Simulator 3D");
    SetTargetFPS(kTargetFps);

    // Borderless fullscreen on the current monitor - preferred over
    // exclusive ToggleFullscreen() for this desktop simulator, since it
    // keeps normal window management (alt-tab, other monitors) working.
    ToggleBorderlessWindowed();

    robot::visual::VirtualWorld world;
    robot::visual::VirtualRobotHardware hardware(world);
    robot::RobotStateMachine stateMachine;
    robot::RobotController controller(hardware);
    robot::visual::DemoCommandSource commandSource;
    robot::RobotRuntime runtime(commandSource, stateMachine, controller);
    robot::visual::Renderer3D renderer;

    // Camera mouse capture starts enabled, so the mouse immediately
    // drives the camera without an extra keypress; DisableCursor() also
    // keeps the OS cursor from escaping the window while captured.
    bool cameraCaptured = true;
    DisableCursor();

    // Optional (Phase 13N): pausing only ever skips the world-movement
    // tick below - it never skips runtime.step(), and it does not affect
    // TAB/F11 behavior at all.
    bool worldPaused = false;

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

        if (IsKeyPressed(KEY_SPACE))
        {
            worldPaused = !worldPaused;
        }

        // Exactly one RobotRuntime::step() per rendered frame - the
        // render loop itself is the scheduler (see RobotRuntime's own
        // docs); once DemoCommandSource is exhausted, NoEvent every frame
        // is the expected, correct outcome.
        runtime.step();

        if (!worldPaused)
        {
            hardware.update(GetFrameTime());
        }

        renderer.renderFrame(world, cameraCaptured, robot::toString(stateMachine.currentState()),
                              robot::visual::toString(hardware.currentCommand()));
    }

    CloseWindow();
    return 0;
}
