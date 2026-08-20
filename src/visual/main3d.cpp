#include "raylib.h"

#include "robot/CompositePollingEventSource.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/DemoCommandSource.hpp"
#include "robot/visual/ManualDriveInput.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/Renderer3D.hpp"
#include "robot/visual/VirtualDistanceSensor.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{
constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr int kTargetFps = 60;

// Manual-drive-mode wheel speed (Phase 13P) - visual-simulator-only debug
// control, not related to VirtualRobotHardware's own kForwardWheelSpeed.
// One shared magnitude for forward/reverse/turn, matching the
// deterministic UP/DOWN/LEFT/RIGHT behavior table in
// docs/technical-decisions.md (Phase 13P) exactly.
constexpr float kManualWheelSpeed = 1.0F;
} // namespace

// Thin application-lifecycle composition root for the 3D visual simulator:
// owns the raylib window, fullscreen/mouse-capture state, and the render
// loop only. As of Phase 13N, the on-screen robot is driven by the real,
// unmodified RobotStateMachine/RobotController/RobotRuntime; as of Phase
// 13O, obstacle stop/resume is driven by the real, unmodified
// HardwareEventSource reading VirtualRobotHardware's geometry-backed
// obstacleDetected() - see docs/technical-decisions.md. DemoCommandSource
// delivers exactly ScenarioLoaded then StartMission once each, so the FSM
// reaches Moving through its real transition rules; CompositePollingEventSource
// then combines that finite command source with the always-live
// HardwareEventSource (command-before-sensor priority, unmodified from Phase
// 13J), exactly the same composition Application::runLiveSimulation() uses
// for the CLI's --live mode. main3d never injects ObstacleDetected/
// ObstacleCleared directly - both only ever come from HardwareEventSource
// reading VirtualRobotHardware's real sensor state. As of Phase 13P,
// movement is differential-drive kinematics (VirtualRobotHardware now owns
// a DifferentialDrive), and this file adds an M-toggled manual wheel
// override purely for interactively proving turning - see the
// manualDriveMode block below and docs/technical-decisions.md. As of
// Phase 13Q, an A-toggled reactive obstacle-avoidance policy sits between
// manual and the FSM in drive authority - see the avoidanceEnabled block
// below.
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
    robot::HardwareEventSource hardwareEventSource(hardware);
    robot::visual::DemoCommandSource commandSource;
    robot::CompositePollingEventSource compositeSource(commandSource, hardwareEventSource);
    robot::RobotStateMachine stateMachine;
    robot::RobotController controller(hardware);
    robot::RobotRuntime runtime(compositeSource, stateMachine, controller);
    robot::visual::Renderer3D renderer;

    // Reactive obstacle-avoidance policy (Phase 13Q) - stateless/
    // deterministic, only ever answers "what wheel speeds does an
    // avoidance turn use." The decision of *whether* to apply it each
    // frame lives entirely here in main3d.cpp (see avoidanceEnabled
    // below) - ReactiveObstacleAvoidance itself has no FSM/sensor
    // knowledge.
    robot::visual::ReactiveObstacleAvoidance avoidance;

    // Display-only sensor handle (Phase 13O): reads the exact same
    // VirtualWorld state VirtualRobotHardware's own internal sensor does, so
    // the HUD/ray-visualization telemetry it produces is always identical to
    // what actually drove obstacleDetected() this frame. It never influences
    // FSM/hardware behavior - it only feeds Renderer3D's VisualTelemetry.
    robot::visual::VirtualDistanceSensor sensor(world);

    // Camera mouse capture starts enabled, so the mouse immediately
    // drives the camera without an extra keypress; DisableCursor() also
    // keeps the OS cursor from escaping the window while captured.
    bool cameraCaptured = true;
    DisableCursor();

    // SPACE pauses/resumes world movement only (VirtualRobotHardware::update()
    // below) - it never skips runtime.step(), so event polling (including
    // hardware obstacle sensing) continues normally while paused, and it
    // never fakes an FSM transition.
    bool worldPaused = false;

    // Manual drive mode (Phase 13P) - a visual-simulator-only debug
    // override, entirely local to main3d.cpp. IRobotHardware,
    // RobotController, RobotRuntime, and RobotStateMachine have no
    // knowledge this exists; runtime.step() keeps polling FSM events every
    // frame regardless, but while manualDriveMode is true, the arrow-key
    // wheel speeds set below win for physical movement (see
    // VirtualRobotHardware::setManualWheelSpeeds()/driveAuthority()).
    //
    // Arrow keys were chosen over WASD to avoid CAMERA_FREE's WASD
    // panning - but raylib's UpdateCamera() (rcamera.h) ALSO reads
    // KEY_UP/KEY_DOWN/KEY_LEFT/KEY_RIGHT for camera pitch/yaw in every
    // non-custom/orbital mode, including CAMERA_FREE, on top of its
    // continuous unbounded mouse-look while the cursor is captured. Left
    // enabled, every manual-drive keypress (and any stray mouse motion)
    // would simultaneously rotate the camera view out from under the
    // robot - the wheel speeds/pose would still be correct (see
    // DifferentialDriveTests/VirtualRobotHardwareTests), but the robot
    // could appear not to move at all. So camera updates are suppressed
    // entirely while manual drive mode is active (see updateCamera below)
    // - the camera holds still, and arrow keys/mouse only ever drive the
    // robot until `M` is pressed again.
    bool manualDriveMode = false;

    // Reactive obstacle-avoidance enable/disable (Phase 13Q) - ON by
    // default so RobotSimulator3D demonstrates autonomous behavior
    // immediately; `A` toggles it. When OFF, WaitingForObstacleClear
    // behaves exactly like Phase 13O/13P: the robot stays stopped until
    // the obstacle is cleared some other way (e.g. `O`). This flag only
    // decides whether the policy below is ever engaged - it has no effect
    // on manual mode, which always takes priority regardless of this
    // setting (see VirtualRobotHardware::driveAuthority()).
    bool avoidanceEnabled = true;

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

        if (IsKeyPressed(KEY_O))
        {
            // O only ever changes world geometry - it never emits
            // ObstacleCleared/ObstacleDetected itself. The next runtime.step()
            // below has HardwareEventSource sample this new geometry through
            // VirtualRobotHardware::obstacleDetected() and emit the real
            // edge-triggered event, exactly like any other sensor change.
            const bool currentlyEnabled =
                world.obstacleEnabled(robot::visual::VirtualWorld::kBlockingObstacleIndex);
            world.setObstacleEnabled(robot::visual::VirtualWorld::kBlockingObstacleIndex, !currentlyEnabled);
        }

        if (IsKeyPressed(KEY_M))
        {
            manualDriveMode = !manualDriveMode;
            if (!manualDriveMode)
            {
                // Immediately restore the wheel speeds corresponding to
                // whatever drive authority ranks highest once manual ends
                // - the still-active autonomous-avoidance override if one
                // is pending, otherwise the latest VirtualDriveCommand -
                // see VirtualRobotHardware::clearManualWheelOverride().
                hardware.clearManualWheelOverride();
            }
        }

        if (IsKeyPressed(KEY_A))
        {
            // Only toggles whether the policy below is ever engaged; it
            // never itself starts/stops a turn - see the avoidanceEnabled
            // declaration above.
            avoidanceEnabled = !avoidanceEnabled;
        }

        // Exactly one RobotRuntime::step() per rendered frame - the render
        // loop itself is the scheduler (see RobotRuntime's own docs). If
        // VirtualRobotHardware::obstacleDetected() has newly become true,
        // this is where HardwareEventSource surfaces ObstacleDetected and
        // RobotController stops the hardware - before update() below ever
        // runs this frame, so the robot never moves an extra frame past
        // detection.
        runtime.step();

        // Determine and apply drive authority (Phase 13Q) - manual first,
        // autonomous avoidance second, FSM otherwise (see
        // VirtualRobotHardware::driveAuthority()). This runs AFTER
        // runtime.step() specifically so the avoidance decision below sees
        // this frame's up-to-date FSM state (e.g. a WaitingForObstacleClear
        // transition that just happened this same frame) - see
        // docs/technical-decisions.md (Phase 13Q, frame order).
        if (manualDriveMode)
        {
            // Recomputed from scratch every frame from currently-held
            // keys via computeManualWheelSpeeds() (ManualDriveInput.hpp) -
            // so releasing every key naturally yields zero wheel speeds
            // with no separate "key up" handling, and a previous manual
            // command can never remain latched. X uses IsKeyDown() (held,
            // not just the press edge) and has the highest priority inside
            // that pure function - it is evaluated first and, while true,
            // fully overrides every directional key that frame, instead of
            // only zeroing wheel speeds for the single frame X transitions
            // down (that one-frame-only behavior was the bug: any
            // directional key still held on the very next frame would
            // silently override X's zero and the robot would resume
            // moving). Combining UP/DOWN with LEFT/RIGHT produces an arc
            // turn; holding only LEFT/RIGHT rotates in place.
            //
            // While manual mode is active, any pending autonomous-
            // avoidance override is deliberately left untouched below (the
            // policy is simply not evaluated) - it automatically resumes
            // the instant clearManualWheelOverride() runs above, without
            // needing to be re-requested.
            const robot::visual::WheelSpeeds manualSpeeds = robot::visual::computeManualWheelSpeeds(
                IsKeyDown(KEY_UP), IsKeyDown(KEY_DOWN), IsKeyDown(KEY_LEFT), IsKeyDown(KEY_RIGHT), IsKeyDown(KEY_X),
                kManualWheelSpeed);

            // This is main3d.cpp's manual-drive-mode input feeding
            // VirtualRobotHardware's visual-only override API - never a
            // fake RobotController/IRobotHardware call, and never
            // presented as if it came from the FSM.
            hardware.setManualWheelSpeeds(manualSpeeds.left, manualSpeeds.right);
        }
        else
        {
            // Simple V1 reactive-avoidance policy (Phase 13Q, section 7):
            // engage only while avoidance is enabled, the FSM is actually
            // WaitingForObstacleClear, and the forward sensor still
            // reports the obstacle. The moment any of those stops holding
            // true - including the real ObstacleCleared edge returning the
            // FSM to Moving above - the override is released and control
            // falls back to whatever RobotController/the FSM currently
            // wants. This never calls stateMachine.processEvent()/
            // handleEvent() or injects ObstacleDetected/ObstacleCleared
            // itself - HardwareEventSource observes the sensor's real
            // true -> false edge naturally, once the turn below has
            // rotated the sensor ray far enough away from the obstacle.
            const bool shouldAvoidObstacle = avoidanceEnabled &&
                                              stateMachine.currentState() == robot::RobotState::WaitingForObstacleClear &&
                                              hardware.obstacleDetected();

            if (shouldAvoidObstacle)
            {
                const robot::visual::WheelSpeeds turnSpeeds = avoidance.avoidanceWheelSpeeds();
                hardware.setAutonomousWheelSpeeds(turnSpeeds.left, turnSpeeds.right);
            }
            else if (hardware.autonomousOverrideActive())
            {
                hardware.clearAutonomousWheelOverride();
            }
        }

        if (!worldPaused)
        {
            hardware.update(GetFrameTime());
        }

        robot::visual::VisualTelemetry telemetry{};
        telemetry.stateText = robot::toString(stateMachine.currentState());
        telemetry.commandText = robot::visual::toString(hardware.currentCommand());
        telemetry.sensorOrigin = sensor.sensorOrigin();
        telemetry.sensorDirection = sensor.sensorDirection();
        telemetry.obstacleDistance = sensor.distanceToNearestObstacle();
        telemetry.obstacleDetected = sensor.obstacleDetected();
        telemetry.sensorMaximumRange = robot::visual::VirtualDistanceSensor::kMaximumRange;
        const robot::visual::WheelSpeeds wheelSpeeds = hardware.wheelSpeeds();
        telemetry.leftWheelSpeed = wheelSpeeds.left;
        telemetry.rightWheelSpeed = wheelSpeeds.right;
        telemetry.driveAuthorityText = robot::visual::toString(hardware.driveAuthority());
        telemetry.collidedLastUpdate = hardware.collidedLastUpdate();
        telemetry.avoidanceEnabled = avoidanceEnabled;

        // Camera updates (mouse-look and CAMERA_FREE's own arrow-key
        // pitch/yaw) are suppressed while manual drive mode is active -
        // see the manualDriveMode comment above. cameraCaptured still
        // governs cursor capture/release via TAB independently of this.
        const bool updateCamera = cameraCaptured && !manualDriveMode;
        renderer.renderFrame(world, updateCamera, telemetry);
    }

    CloseWindow();
    return 0;
}
