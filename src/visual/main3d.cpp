#include <cmath>

#include "raylib.h"

#include "robot/CompositePollingEventSource.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/ForwardClearanceProbe.hpp"
#include "robot/visual/HomeArrivalEventSource.hpp"
#include "robot/visual/HomeNavigator.hpp"
#include "robot/visual/HomeZoneMonitor.hpp"
#include "robot/visual/ManualDriveInput.hpp"
#include "robot/visual/MissionControlEventSource.hpp"
#include "robot/visual/MissionTask.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/Renderer3D.hpp"
#include "robot/visual/TableEdgeSafetyController.hpp"
#include "robot/visual/VirtualCliffSensor.hpp"
#include "robot/visual/VirtualDistanceSensor.hpp"
#include "robot/visual/VirtualObstacleSensorArray.hpp"
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
// obstacleDetected() - see docs/technical-decisions.md. main3d never
// injects ObstacleDetected/ObstacleCleared directly - both only ever come
// from HardwareEventSource reading VirtualRobotHardware's real sensor
// state. As of Phase 13P, movement is differential-drive kinematics
// (VirtualRobotHardware now owns a DifferentialDrive), and this file adds
// an M-toggled manual wheel override purely for interactively proving
// turning - see the manualDriveMode block below and
// docs/technical-decisions.md. As of Phase 13Q, an A-toggled reactive
// obstacle-avoidance policy sits between manual and the FSM in drive
// authority - see the avoidanceEnabled block below. As of Phase 13R, that
// policy is body-clearance-aware: the avoidance latch
// (ReactiveObstacleAvoidance::active()) can legitimately stay engaged for
// several frames after the FSM has already returned to Moving, if
// ForwardClearanceProbe still reports the robot's physical body corridor
// as blocked - see the frame-order block below and
// docs/technical-decisions.md (Phase 13R). As of Phase 13S, a fourth,
// HIGHEST-priority authority - Safety - sits above even manual driving:
// TableEdgeSafetyController, driven by VirtualCliffSensor's four corner
// readings, takes the wheels the instant the robot's footprint nears the
// table's edge, regardless of what manual/autonomous/FSM currently want -
// see docs/technical-decisions.md (Phase 13S). As of Phase 13T,
// HomeNavigator steers the robot toward VirtualWorld's BasePlatform via
// the Navigation drive-authority tier (Safety > Manual >
// AutonomousAvoidance > Navigation > Fsm), and HomeArrivalEventSource
// turns HomeNavigator's own Arrived state into a HomeReached Event,
// consumed through the existing ReturningHome + HomeReached transition.
//
// As of Phase 13U, mission assignment is fully explicit - the simulator no
// longer auto-starts. MissionControlEventSource is the single event source
// behind every Mission Control command (`1`/`2`/`3`/`R`), turning each
// into ScenarioLoaded/StartMission/ReturnHomeRequested/StopTaskRequested
// Events - never a direct FSM mutation, and never two competing Return
// Home implementations (`2` and `R` are literally the same call). Task
// status (`MissionTask` - None/Roam/ReturnHome) is derived fresh every
// frame from RobotStateMachine's own public state
// (MissionTask.hpp::deriveMissionTask()) - it is presentation/Home-Zone-
// activation context only, never a second authority over the robot.
// HomeZoneMonitor is an automatic convenience trigger: while the task is
// Roam, it watches distance from base with hysteresis (exit/rearm radii)
// and emits its own ReturnHomeRequested the instant the robot wanders too
// far, through the exact same IPollingEventSource/FSM path as every
// manual request - see docs/technical-decisions.md (Phase 13U) for the
// full audit, the nested event-source priority
// (MissionControl > HardwareEventSource > HomeArrivalEventSource >
// HomeZoneMonitor), and a real V1 limitation this phase's own testing
// discovered (reactive avoidance can resonate against HomeNavigator's
// continuous re-aiming for a centered obstacle sitting exactly on the
// return path).
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

    // Phase 13U: the single event source behind every explicit Mission
    // Control command (`1`/`2`/`3`, and `R` as the Return Home alias - see
    // the keyboard-input block below) - never a direct FSM mutation, and
    // never a separate/competing implementation from `R`'s own Return Home
    // path. Replaces DemoCommandSource's automatic ScenarioLoaded/
    // StartMission for this interactive executable - the simulator no
    // longer auto-starts; mission assignment is fully explicit. (Nothing
    // deletes DemoCommandSource itself - it remains in the codebase, still
    // used by tests/examples that want a fixed automatic two-event
    // sequence with no keyboard involved.)
    robot::visual::MissionControlEventSource missionControl;

    // HomeNavigator is raylib-free navigation logic (Aligning/Driving/
    // Arrived toward world.basePlatform()); HomeArrivalEventSource
    // observes ITS OWN Arrived state (edge-triggered) to produce
    // HomeReached, exactly like HardwareEventSource observes
    // VirtualRobotHardware's sensors - see HomeNavigator.hpp/
    // HomeArrivalEventSource.hpp for why neither ever decides FSM
    // transitions itself.
    robot::visual::HomeNavigator homeNavigator;
    robot::visual::HomeArrivalEventSource homeArrivalEventSource(homeNavigator);

    // HomeZoneMonitor (Phase 13U) - the automatic "come home" trigger for
    // Roam: watches distance from world.basePlatform() with hysteresis
    // while the current task is Roam, and produces its own
    // ReturnHomeRequested the instant the robot wanders outside the exit
    // radius - through this same IPollingEventSource path, never a direct
    // Navigation call or FSM mutation (see HomeZoneMonitor.hpp).
    robot::visual::HomeZoneMonitor homeZone;

    // CompositePollingEventSource only combines two sources at a time
    // (Phase 13J, unmodified), so the four effective sources this phase
    // needs are composed via NESTING rather than rewriting that class.
    // Priority (Phase 13U, highest first): explicit Mission Control
    // commands > hardware obstacle/sensor events > HomeReached arrival >
    // automatic Home Zone return request. Rationale: explicit user
    // commands must never be starved by an automatic trigger; safety-
    // relevant hardware perception must never be lost underneath a
    // same-frame HomeReached/Home-Zone readiness; HomeReached completes an
    // already-active Return Home, which outranks a brand new automatic
    // request from the same or a different mission. See
    // docs/technical-decisions.md (Phase 13U) for the full audit.
    robot::CompositePollingEventSource innerHardwareGroup(hardwareEventSource, homeArrivalEventSource);
    robot::CompositePollingEventSource innerAutoGroup(innerHardwareGroup, homeZone);
    robot::CompositePollingEventSource compositeSource(missionControl, innerAutoGroup);
    robot::RobotStateMachine stateMachine;
    robot::RobotController controller(hardware);
    robot::RobotRuntime runtime(compositeSource, stateMachine, controller);
    robot::visual::Renderer3D renderer;

    // Reactive obstacle-avoidance policy (Phase 13Q; stateful latch as of
    // Phase 13R) - only ever answers "what wheel speeds does an avoidance
    // turn use" and "is the latch currently engaged." The decision of
    // *whether/when* to trigger or release it each frame lives entirely
    // here in main3d.cpp (see the update() call below) -
    // ReactiveObstacleAvoidance itself has no FSM/sensor/clearance
    // knowledge of its own.
    robot::visual::ReactiveObstacleAvoidance avoidance;

    // Forward BODY-clearance probe (Phase 13R) - answers a different
    // question than the point-ray VirtualDistanceSensor below: whether
    // the robot's circular collision footprint has a physically safe
    // corridor to move forward along its current heading, not merely
    // whether a single ray is unobstructed. Drives avoidance's release
    // condition (see the update() call below) - never
    // RobotStateMachine/RobotController/RobotRuntime/IRobotHardware
    // directly, and never a replacement for RobotCollision's own
    // unconditional last-resort guard inside VirtualRobotHardware::update().
    robot::visual::ForwardClearanceProbe clearanceProbe(world);

    // Table-edge safety (Phase 13S) - VirtualCliffSensor reads the same
    // live VirtualWorld pose/table data every frame; TableEdgeSafetyController
    // is the small stateful recovery latch that converts its readings
    // into emergency wheel speeds. Both are raylib-free and have no FSM/
    // IRobotHardware knowledge of their own - the decision of *whether*
    // safety authority applies each frame lives entirely here in
    // main3d.cpp, exactly like avoidance's own caller contract. Unlike
    // avoidance, there is no enable/disable toggle - table-edge safety is
    // a physical safety layer, always active.
    robot::visual::VirtualCliffSensor cliffSensor(world);
    robot::visual::TableEdgeSafetyController tableEdgeSafety;

    // Display-only sensor handle (Phase 13O): reads the exact same
    // VirtualWorld state VirtualRobotHardware's own internal sensor does, so
    // the HUD/ray-visualization telemetry it produces is always identical to
    // what actually drove obstacleDetected() this frame. It never influences
    // FSM/hardware behavior - it only feeds Renderer3D's VisualTelemetry.
    robot::visual::VirtualDistanceSensor sensor(world);

    // Display-only three-ray perception array handle (manual-validation
    // bugfix): reads the exact same VirtualWorld state
    // VirtualRobotHardware's own internal array does, so the HUD/ray-
    // visualization telemetry is always identical to what actually feeds
    // obstacleDetected() this frame. It never influences FSM/hardware
    // behavior on its own - it only feeds Renderer3D's VisualTelemetry,
    // exactly like `sensor` above.
    robot::visual::VirtualObstacleSensorArray obstacleSensorArray(world);

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

    // HUD detail level (UX polish) - `H` toggles Full <-> Compact.
    // Presentation-only: read by Renderer3D's drawHud() alone, never
    // consulted by any FSM/hardware/safety/avoidance decision below.
    // Starts Full so RobotSimulator3D shows full engineering telemetry
    // immediately, matching every other toggle's own documented default.
    robot::visual::HudMode hudMode = robot::visual::HudMode::Full;

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

        if (IsKeyPressed(KEY_ONE))
        {
            // Start Roam - edge-triggered, state-aware (see
            // MissionControlEventSource::requestStartRoam()): queues
            // ScenarioLoaded+StartMission from Idle, only StartMission
            // from Ready, or nothing at all if already Moving/
            // ReturningHome/etc. Reads stateMachine.currentState() as of
            // the END of the previous frame's runtime.step() - the
            // correct up-to-date snapshot, since input is handled before
            // this frame's own step() below.
            missionControl.requestStartRoam(stateMachine.currentState());
        }

        if (IsKeyPressed(KEY_TWO) || IsKeyPressed(KEY_R))
        {
            // Return Home - `2` and `R` (preserved from Phase 13T) are
            // deliberately the exact same call: one Return Home
            // implementation, never two competing ones. Edge-triggered,
            // never a direct RobotStateMachine mutation or a direct
            // hardware.returnToBase() call - the request is only ever
            // consumed through the real Moving/Ready + ReturnHomeRequested
            // -> ReturningHome transition on a later runtime.step().
            missionControl.requestReturnHome();
        }

        if (IsKeyPressed(KEY_THREE))
        {
            // Stop Task - cancels whatever task is currently active
            // (Roam, Return Home, or a task paused in
            // WaitingForObstacleClear), landing in the reusable Ready
            // state - never a direct RobotStateMachine mutation (Phase
            // 13U).
            missionControl.requestStopTask();
        }

        if (IsKeyPressed(KEY_H))
        {
            // Edge-triggered (IsKeyPressed, not IsKeyDown) so holding H
            // toggles exactly once, not every frame - presentation-only,
            // see the hudMode declaration above.
            hudMode =
                (hudMode == robot::visual::HudMode::Full) ? robot::visual::HudMode::Compact : robot::visual::HudMode::Full;
        }

        // Exactly one RobotRuntime::step() per rendered frame - the render
        // loop itself is the scheduler (see RobotRuntime's own docs). If
        // VirtualRobotHardware::obstacleDetected() has newly become true,
        // this is where HardwareEventSource surfaces ObstacleDetected and
        // RobotController stops the hardware - before update() below ever
        // runs this frame, so the robot never moves an extra frame past
        // detection.
        runtime.step();

        // Compute this frame's forward BODY-clearance telemetry (Phase
        // 13R) - independent of, and complementary to, the point-ray
        // sensor below. Read once here so the trigger/release decision
        // and the HUD telemetry always agree on the exact same value.
        const bool forwardCorridorClear = clearanceProbe.isForwardCorridorClear();

        // Advance the avoidance latch (Phase 13R) - runs every frame,
        // regardless of manual drive mode, so it stays in sync with the
        // real FSM/sensor/clearance state and is correctly restored the
        // instant manual mode ends (see docs/technical-decisions.md,
        // Phase 13R, "manual priority while latched"). The trigger
        // condition is unchanged from Phase 13Q (section 7): avoidance
        // enabled, the FSM is actually WaitingForObstacleClear, and the
        // forward sensor still reports the obstacle. This never calls
        // stateMachine.processEvent()/handleEvent() or injects
        // ObstacleDetected/ObstacleCleared itself - HardwareEventSource
        // observes the sensor's real true -> false edge naturally, once
        // the turn below has rotated the sensor ray far enough away from
        // the obstacle. What changed from Phase 13Q: the FSM's real
        // return to Moving on that edge no longer by itself releases the
        // override - the latch stays active until forwardCorridorClear is
        // also true.
        const bool triggerAvoidance = avoidanceEnabled &&
                                       stateMachine.currentState() == robot::RobotState::WaitingForObstacleClear &&
                                       hardware.obstacleDetected();
        avoidance.update(avoidanceEnabled, triggerAvoidance, forwardCorridorClear);

        // Compute this frame's cliff-sensor readings and advance the
        // table-edge safety recovery latch (Phase 13S) - runs
        // unconditionally every frame, exactly like avoidance's own
        // update() above, so it stays in sync with the real robot pose
        // regardless of manual/autonomous/FSM state.
        const robot::visual::CliffSensorReadings cliffReadings = cliffSensor.readings();
        tableEdgeSafety.update(cliffReadings, world.robotPose(), world.tableSurface());

        // Advance HomeNavigator (Phase 13T) - runs every frame,
        // unconditionally, enabled exactly when RobotController/the FSM's
        // current intent is ReturnToBase (VirtualRobotHardware::
        // currentCommand()), never decided by HomeNavigator itself. This
        // single condition naturally covers every interruption/resumption
        // case with no extra bookkeeping: it goes false the moment
        // WaitingForObstacleClear's stop() runs (HomeNavigator resets to
        // Inactive), and true again the instant ReturningHome resumes
        // (HomeNavigator recomputes a fresh target from the new pose) -
        // while it stays true throughout a Manual/Safety interruption
        // (RobotController's command tracking is independent of
        // DriveAuthority override state), so a temporarily-overridden
        // Return Home request is never cancelled, only outranked. Reads
        // world.robotPose()/world.basePlatform() directly - HomeNavigator
        // never duplicates base coordinates of its own. This runs AFTER
        // runtime.step() above, so a same-frame arrival is naturally
        // consumed by HardwareEventSource's polling composite on the NEXT
        // frame - an accepted, deterministic one-frame latency (see
        // docs/technical-decisions.md, Phase 13T), never worked around by
        // calling runtime.step() twice.
        const bool navigationEnabled =
            hardware.currentCommand() == robot::visual::VirtualDriveCommand::ReturnToBase;
        const robot::visual::HomeNavigationOutput homeNavigation =
            homeNavigator.update(world.robotPose(), world.basePlatform(), navigationEnabled);

        // Phase 13U: this frame's user-facing task status - derived
        // fresh from RobotStateMachine's own public state, never a second
        // authority over the robot (see MissionTask.hpp). Drives both the
        // Mission Control HUD panel and HomeZoneMonitor's activation
        // condition below.
        const robot::visual::MissionTask missionTask =
            robot::visual::deriveMissionTask(stateMachine.currentState(), stateMachine.returnHomeReason());

        // Advance HomeZoneMonitor (Phase 13U) - runs every frame,
        // unconditionally, exactly like HomeNavigator's own update()
        // above; only actually evaluates distance while missionTask is
        // Roam (see HomeZoneMonitor::update()'s own docs for why it must
        // freeze - not merely suppress its trigger - the rest of the
        // time). Its own pollEvent() (polled by runtime.step() next
        // frame, via the composite chain) is the ONLY way a Home-Zone
        // excursion ever reaches the FSM - never a direct Navigation call
        // or RobotState mutation.
        homeZone.update(world.robotPose(), world.basePlatform(), missionTask == robot::visual::MissionTask::Roam);

        // Apply drive authority (Safety > Manual > AutonomousAvoidance >
        // Navigation > Fsm - see VirtualRobotHardware::driveAuthority()).
        // Safety is synced first and unconditionally, exactly like the
        // avoidance/navigation overrides below: even while manual mode is
        // active, this keeps firing so a table edge reached under manual
        // control is caught immediately -
        // VirtualRobotHardware::applyEffectiveWheelSpeeds() is the one
        // place final priority is actually resolved, so this code never
        // needs to reason about ordering itself.
        if (tableEdgeSafety.active())
        {
            const robot::visual::WheelSpeeds recoverySpeeds = tableEdgeSafety.recoveryWheelSpeeds();
            hardware.setSafetyWheelSpeeds(recoverySpeeds.left, recoverySpeeds.right);
        }
        else if (hardware.safetyOverrideActive())
        {
            hardware.clearSafetyWheelOverride();
        }

        // The autonomous override is kept in sync with the avoidance
        // latch unconditionally, even while manual mode is active: manual
        // still physically wins over it (driveAuthority() always prefers
        // Manual over AutonomousAvoidance), but the avoidance request
        // stays logically latched underneath, exactly as it did in Phase
        // 13Q, and is restored automatically the instant manual mode ends
        // without needing to be re-triggered.
        if (avoidance.active())
        {
            const robot::visual::WheelSpeeds turnSpeeds = avoidance.avoidanceWheelSpeeds();
            hardware.setAutonomousWheelSpeeds(turnSpeeds.left, turnSpeeds.right);
        }
        else if (hardware.autonomousOverrideActive())
        {
            hardware.clearAutonomousWheelOverride();
        }

        // The navigation override is kept in sync with HomeNavigator
        // unconditionally, even while manual/autonomous/safety currently
        // wins physically - exactly the same always-latched-underneath
        // pattern the safety/autonomous overrides above already use, so
        // Return Home resumes automatically the instant a higher
        // authority releases, with no need to be re-triggered. Only set
        // while Aligning/Driving: Arrived/Inactive need no override,
        // since currentCommand() == ReturnToBase's own FSM-mapped wheel
        // speeds are already zero (see
        // VirtualRobotHardware::wheelSpeedsForCommand()) - avoiding
        // unnecessary override churn for an identical physical result.
        const bool navigationDriving = homeNavigation.state == robot::visual::HomeNavigationState::Aligning ||
                                        homeNavigation.state == robot::visual::HomeNavigationState::Driving;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(homeNavigation.wheelSpeeds.left, homeNavigation.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }

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
            const robot::visual::WheelSpeeds manualSpeeds = robot::visual::computeManualWheelSpeeds(
                IsKeyDown(KEY_UP), IsKeyDown(KEY_DOWN), IsKeyDown(KEY_LEFT), IsKeyDown(KEY_RIGHT), IsKeyDown(KEY_X),
                kManualWheelSpeed);

            // This is main3d.cpp's manual-drive-mode input feeding
            // VirtualRobotHardware's visual-only override API - never a
            // fake RobotController/IRobotHardware call, and never
            // presented as if it came from the FSM.
            hardware.setManualWheelSpeeds(manualSpeeds.left, manualSpeeds.right);
        }

        if (!worldPaused)
        {
            hardware.update(GetFrameTime());
        }

        robot::visual::VisualTelemetry telemetry{};
        telemetry.stateText = robot::toString(stateMachine.currentState());
        telemetry.commandText = robot::visual::toString(hardware.currentCommand());
        // Manual-validation bugfix: optional Full-HUD-only diagnostic -
        // Renderer3D never uses this to decide anything, it only displays
        // it.
        telemetry.returnHomeReasonText = robot::toString(stateMachine.returnHomeReason());
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
        telemetry.avoidanceActive = avoidance.active();
        telemetry.forwardClearanceClear = forwardCorridorClear;
        telemetry.clearanceLookahead = robot::visual::ForwardClearanceProbe::kLookaheadDistance;
        telemetry.cliffFrontLeft = cliffReadings.frontLeft;
        telemetry.cliffFrontRight = cliffReadings.frontRight;
        telemetry.cliffRearLeft = cliffReadings.rearLeft;
        telemetry.cliffRearRight = cliffReadings.rearRight;
        telemetry.edgeSafetyActive = tableEdgeSafety.active();
        telemetry.edgeRecoveryStateText = robot::visual::toString(tableEdgeSafety.state());
        telemetry.edgeTargetHeadingDegrees = tableEdgeSafety.targetRecoveryHeadingDegrees();
        telemetry.edgeHeadingErrorDegrees = tableEdgeSafety.currentHeadingErrorDegrees();

        // Manual-validation bugfix: left/right ray telemetry - center
        // reuses telemetry.sensorOrigin/obstacleDistance/obstacleDetected
        // above (geometrically identical to the array's FrontCenter ray,
        // never recomputed twice). bodyCorridorObstacleHazard mirrors the
        // exact same hazard signal hardware.obstacleDetected() itself ORs
        // in - see VirtualRobotHardware::bodyCorridorObstacleHazard().
        const robot::visual::ObstacleSensorArrayReadings obstacleRays = obstacleSensorArray.readings();
        telemetry.obstacleRayLeftOrigin =
            obstacleSensorArray.rayOrigin(robot::visual::ObstacleRayPosition::FrontLeft);
        telemetry.obstacleRayRightOrigin =
            obstacleSensorArray.rayOrigin(robot::visual::ObstacleRayPosition::FrontRight);
        telemetry.obstacleRayLeftDistance = obstacleRays.frontLeftDistance;
        telemetry.obstacleRayRightDistance = obstacleRays.frontRightDistance;
        telemetry.obstacleRayLeftDetected = obstacleRays.frontLeftDetected;
        telemetry.obstacleRayRightDetected = obstacleRays.frontRightDetected;
        telemetry.bodyCorridorObstacleHazard = hardware.bodyCorridorObstacleHazard();
        telemetry.obstacleHazard = hardware.obstacleDetected();
        telemetry.hudMode = hudMode;

        // Phase 13T: already-computed HomeNavigator telemetry - Renderer3D
        // never has any notion of the Aligning/Driving/Arrived policy
        // itself.
        telemetry.homeNavigationStateText = robot::visual::toString(homeNavigation.state);
        telemetry.homeNavigationDistance = homeNavigation.distanceToHome;
        telemetry.homeNavigationTargetHeadingDegrees = homeNavigation.targetHeadingDegrees;
        telemetry.homeNavigationHeadingErrorDegrees = homeNavigation.headingErrorDegrees;
        telemetry.homeNavigationGuideVisible = navigationDriving;

        // Phase 13U: Mission Control panel telemetry - main3d computes
        // task status and base distance directly (the same simple
        // dx/dz-from-pose formula HomeNavigator/HomeZoneMonitor each
        // independently use internally for their own decisions); this is
        // a third, presentation-only computation purely for display,
        // exactly like "Position: X/Z" above already reads
        // world.robotPose() directly rather than through another
        // component.
        telemetry.missionTaskText = robot::visual::toString(missionTask);
        {
            const robot::visual::Vec3& robotPosition = world.robotPose().position;
            const robot::visual::Vec3& basePosition = world.basePlatform().position;
            const float dx = basePosition.x - robotPosition.x;
            const float dz = basePosition.z - robotPosition.z;
            telemetry.baseDistance = std::sqrt((dx * dx) + (dz * dz));
        }
        telemetry.homeZoneInside = telemetry.baseDistance <= robot::visual::HomeZoneMonitor::kHomeZoneExitRadius;

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
