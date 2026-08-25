#include <array>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "raylib.h"

#include "robot/CompositePollingEventSource.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/CoverageTrail.hpp"
#include "robot/visual/ExecutableDirectory.hpp"
#include "robot/visual/ExplorationCompletionEventSource.hpp"
#include "robot/visual/ExplorationMap.hpp"
#include "robot/visual/ExplorationMapStorage.hpp"
#include "robot/visual/ExplorationMapper.hpp"
#include "robot/visual/ForwardClearanceProbe.hpp"
#include "robot/visual/FrontierExplorer.hpp"
#include "robot/visual/GridPathPlanner.hpp"
#include "robot/visual/ManualDriveInput.hpp"
#include "robot/visual/MissionControlEventSource.hpp"
#include "robot/visual/MissionTask.hpp"
#include "robot/visual/RangeObservation.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/Renderer3D.hpp"
#include "robot/visual/TableEdgeSafetyController.hpp"
#include "robot/visual/TurkishText.hpp"
#include "robot/visual/VirtualCliffSensor.hpp"
#include "robot/visual/VirtualDistanceSensor.hpp"
#include "robot/visual/VirtualObstacleSensorArray.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"
#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualMath.hpp"
#include "robot/visual/WaypointArrivalEventSource.hpp"
#include "robot/visual/WaypointNavigator.hpp"

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

// Phase 13V: how often the exploration map/trail are saved to disk while
// dirty - a periodic timer, not once per frame (this phase's own brief,
// "keep filesystem writes low"). A final unconditional save also runs on
// clean shutdown (see the end of main() below), so this interval only
// bounds how much of an in-progress session could be lost to a non-clean
// exit (window-manager kill, crash, power loss) - 5 seconds is a
// reasonable bound for a desktop simulator without being a noticeable
// per-write cost.
constexpr float kMapSaveIntervalSeconds = 5.0F;

// Phase 13V: where the persisted exploration map/trail live, relative to
// RobotSimulator3D.exe's own directory (see ExecutableDirectory.hpp) -
// never the source tree, so a checkout built on another machine still
// finds/creates its own map correctly. "runtime/" (not "assets/", which
// is read-only shipped content) signals this is generated data - see
// .gitignore.
constexpr const char* kMapStorageRelativePath = "\\runtime\\maps\\exploration_map.json";

// Phase 13X: frontier-selection retry throttle - see the frontier-target-
// selection block's own docs below for the "not merely waiting for a
// temporary map update" reasoning this exists for.
constexpr int kFrontierRetryIntervalFrames = 20;
constexpr int kRequiredConsecutiveNoTargetAttempts = 5;

// Phase 13X blocker fix: minimum heading change (degrees) the robot must
// exhibit, since the moment ReactiveObstacleAvoidance last reported
// localRouteBlockedThisUpdate(), before `triggerAvoidance` is allowed to
// arm a brand new incident again - see the avoidance-wiring block's own
// docs below for why this is required (without it, avoidance would
// re-claim AutonomousAvoidance authority on the very next frame, before
// Navigation's freshly-replanned command ever gets a chance to actually
// turn the robot, since obstacleDetected() is a real-time reading that
// has not yet had a chance to change). Deterministic and physically
// grounded (tied to genuine rotation the robot has performed, never a
// frame-count/wall-clock timer) - comparable in spirit to
// HomeNavigator::kStartDrivingHeadingToleranceDegrees (8.0F), but
// deliberately larger: this only needs to prove the robot has started
// executing a MEANINGFULLY different command, not that it has finished
// aligning to one.
constexpr float kAvoidanceResumeHeadingChangeDegrees = 30.0F;

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
// (MissionTask.hpp::deriveMissionTask()) - it is presentation context
// only, never a second authority over the robot.
//
// Phase 13V human-validation fix: `HomeZoneMonitor` is deliberately NOT
// wired into this executable's event-source chain (it was, in Phase
// 13U). Human validation of Phase 13V's exploration map found that
// automatic Home-Zone-triggered Return Home was cutting normal
// exploration off at roughly 20-30% mapped - the user clarified that
// distance-from-base alone is not a meaningful reason to abandon an
// in-progress exploration/mapping session (unlike `BatteryCritical`, an
// explicit `2`/`R` request, or a future explicit mapping-complete
// signal). `HomeZoneMonitor` itself is unmodified and still compiled/
// tested (`HomeZoneMonitorTests.cpp`) as a small, self-contained,
// correctly-behaving geometry component - only its production wiring
// here was removed, since emitting `ReturnHomeRequested` was its one and
// only responsibility (see docs/technical-decisions.md, "Phase 13V
// human-validation fix," for the full audit and rationale). The event-
// source chain below is correspondingly one level flatter than Phase
// 13U's own nested nested (MissionControl, (Hardware, HomeArrival),
// HomeZone) - this is a wiring simplification, not a redesign of
// `CompositePollingEventSource` itself, which is unmodified.
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

    // Phase 13X: WaypointNavigator is the map-aware waypoint-following
    // orchestrator that now DRIVES Return Home (and, while Haritalama is
    // actively seeking a frontier target, exploration navigation too) -
    // it owns its own internal HomeNavigator for per-waypoint local
    // steering (HomeNavigator itself is UNCHANGED - still exists, still
    // does exactly the Aligning/Driving/Arrived job it always has - see
    // HomeNavigator.hpp), asked each frame to steer toward the CURRENT
    // waypoint of a GridPathPlanner-produced route rather than blindly
    // straight at a possibly-obstructed final goal. WaypointArrivalEventSource
    // observes WaypointNavigator's OWN Arrived state (edge-triggered, once
    // the FULL route completes) to produce HomeReached, exactly like
    // HardwareEventSource observes VirtualRobotHardware's sensors - see
    // WaypointNavigator.hpp/WaypointArrivalEventSource.hpp for why neither
    // ever decides FSM transitions itself. The old plain HomeNavigator +
    // HomeArrivalEventSource production wiring is gone from this file
    // (both classes remain fully compiled/tested elsewhere - see
    // docs/technical-decisions.md, Phase 13X, and HomeZoneMonitor's own
    // earlier precedent for a component staying compiled/tested after its
    // production wiring changes).
    robot::visual::WaypointNavigator mapNavigator;
    robot::visual::WaypointArrivalEventSource waypointArrivalEventSource(mapNavigator);

    // Phase 13X: edge-triggered ReturnHomeRequested once autonomous
    // Haritalama has no reachable frontier left - see
    // ExplorationCompletionEventSource.hpp for the full semantics
    // (deliberately reuses ReturnHomeRequested/ReturnHomeReason::
    // UserRequest, never a new Event/reason - zero RobotStateMachine
    // changes). `completionSignal.complete` is set below, in the
    // frontier-selection block, at the same low-frequency cadence
    // frontier target selection itself runs at - never recomputed by a
    // dedicated per-frame check.
    robot::visual::ExplorationCompletionSignal completionSignal;
    robot::visual::ExplorationCompletionEventSource explorationCompletionEventSource(completionSignal);

    // CompositePollingEventSource only combines two sources at a time
    // (Phase 13J, unmodified), so the four effective sources this phase
    // needs are composed via nesting - the same "nest one level deeper"
    // extension mechanism this file's own comments have documented since
    // Phase 13U/13V (HomeZoneMonitor was an earlier occupant of this exact
    // extension point - see docs/technical-decisions.md). Priority
    // (highest first): explicit Mission Control commands > hardware
    // obstacle/sensor events > HomeReached arrival > mapping-complete
    // auto-return. Rationale: explicit user commands must never be
    // starved by an automatic event; safety-relevant hardware perception
    // must never be lost underneath a same-frame HomeReached/completion
    // readiness; a genuine arrival is reported before a same-frame
    // completion signal in the rare case both are pending at once.
    robot::CompositePollingEventSource innerCompletionGroup(waypointArrivalEventSource,
                                                              explorationCompletionEventSource);
    robot::CompositePollingEventSource innerHardwareGroup(hardwareEventSource, innerCompletionGroup);
    robot::CompositePollingEventSource compositeSource(missionControl, innerHardwareGroup);
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

    // Phase 13V: progressive robot-vacuum-style exploration map. Grid
    // covers world.tableSurface() exactly (ExplorationMap's own
    // constructor derives width/height/resolution from it - never
    // hardcoded here). ExplorationMapper is the ONLY thing allowed to
    // mutate explorationMap's cells, and it is only ever fed real sensor
    // observations (obstacleSensorArray.observations() below) - never
    // world.obstacles() directly; see ExplorationMapper.hpp's own docs.
    // CoverageTrail independently records the actual travelled path,
    // regardless of which task/authority is currently driving the
    // wheels.
    robot::visual::ExplorationMap explorationMap(world.tableSurface());
    robot::visual::ExplorationMapper explorationMapper(explorationMap);
    robot::visual::CoverageTrail coverageTrail;

    // Phase 13X: frontier-based autonomous exploration target selection -
    // raylib-free, reads ONLY explorationMap (see FrontierExplorer.hpp).
    // `currentFrontierTarget`/`frontierBlacklist` are this file's own
    // exploration-loop state (mirrors avoidanceEnabled/mapSaveTimer's own
    // plain-local-variable style below) - never persisted (Phase 13X
    // brief: "DO NOT persist temporary path/waypoint/frontier
    // blacklists"), and cleared whenever Haritalama is not the active
    // task (see the per-frame block below), so a fresh Start Haritalama
    // always begins target selection clean.
    robot::visual::FrontierExplorer frontierExplorer(explorationMap);
    std::optional<robot::visual::FrontierTarget> currentFrontierTarget;
    std::vector<robot::visual::GridCoord> frontierBlacklist;
    int framesSinceLastFrontierAttempt = 0;
    int consecutiveNoTargetFound = 0;

    // Phase 13X: edge-detection state for "avoidance/Safety just
    // released" (see the map-aware-navigation block below, `forceReplan`)
    // - a robot displaced from its planned route by a reactive maneuver
    // must replan from where it actually ended up, never keep chasing an
    // obsolete waypoint behind it.
    bool previousAvoidanceActive = false;
    bool previousSafetyActive = false;

    // Phase 13X blocker fix: deferred-by-one-frame consumption of
    // avoidance's localRouteBlockedThisUpdate() signal - see the
    // avoidance-wiring block's own docs below for why the forced replan
    // it triggers must wait one frame (so this same incident's final
    // sensor observations have already reached explorationMapper before
    // GridPathPlanner reads the map), and `avoidanceSuppressedAfterBlock`/
    // `headingAtLastLocalRouteBlocked` for why `triggerAvoidance` must not
    // immediately re-arm before Navigation's replanned command gets a
    // genuine chance to actually turn the robot.
    bool localRouteBlockedPendingReplan = false;
    bool avoidanceSuppressedAfterBlock = false;
    float headingAtLastLocalRouteBlocked = 0.0F;

    // Runtime-relative persistence path (never a source-tree path - see
    // ExecutableDirectory.hpp/kMapStorageRelativePath's own docs above).
    // An empty executableDirectory() (the OS call failed for some reason)
    // degrades to "no persistence this session" rather than writing to a
    // malformed relative-looking-but-actually-drive-root path - the map
    // still works in memory for the session, it just is not saved/loaded.
    const std::string exeDirectory = robot::visual::executableDirectory();
    const bool mapPersistenceAvailable = !exeDirectory.empty();
    const std::string mapStoragePath = exeDirectory + kMapStorageRelativePath;

    // First run (no compatible file yet) vs. subsequent run (a
    // compatible map was found) - captured once at startup, drives the
    // map panel's "Oluşturuluyor"/"Yüklendi" status line for the whole
    // session (Phase 13V brief). The map is never read-only after
    // loading: explorationMapper keeps updating explorationMap normally
    // regardless of how this came out.
    bool mapWasLoadedAtStartup = false;
    if (mapPersistenceAvailable)
    {
        const robot::visual::MapLoadResult loadResult =
            robot::visual::ExplorationMapStorage::load(mapStoragePath, explorationMap, &coverageTrail);
        mapWasLoadedAtStartup = (loadResult == robot::visual::MapLoadResult::Loaded);
    }
    float mapSaveTimer = 0.0F;

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

    // HUD detail level (UX polish) - `H` toggles Full <-> Compact, shown
    // to the user as Ayrıntılı (detailed) <-> Sade (simple). Presentation-
    // only: read by Renderer3D's drawHud() alone, never consulted by any
    // FSM/hardware/safety/avoidance decision below. Final UI/HUD polish:
    // starts Compact (Sade) so RobotSimulator3D shows the small,
    // easy-to-read operational summary immediately - the new default per
    // this phase's brief - with the full engineering telemetry panel one
    // `H` press away.
    robot::visual::HudMode hudMode = robot::visual::HudMode::Compact;

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
            // Phase 13W human-visual-redesign v2: toggles the Keyboard's
            // registered obstacle (a named, semantic lookup via
            // deskObjectObstacleIndex() - never a raw magic index) now that
            // the old standalone "legacy blocking cube" is gone entirely.
            const std::size_t keyboardIndex =
                world.deskObjectObstacleIndex(robot::visual::DeskObjectType::Keyboard);
            const bool currentlyEnabled = world.obstacleEnabled(keyboardIndex);
            world.setObstacleEnabled(keyboardIndex, !currentlyEnabled);
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

        // Phase 13X blocker fix: reads the STICKY flag exactly as left by
        // the END of the PREVIOUS frame (before this frame's own
        // avoidance.update() call below can overwrite it for the frame
        // AFTER this one) - by now, last frame's own
        // explorationMapper.update() call has already run (it is the
        // last thing driveFrame()/main3d's loop does each frame), so the
        // map this value's consumer reads from below is guaranteed to
        // include the observations gathered right up through the exact
        // frame the block was reported on. See the avoidance-wiring
        // block's own docs for the full "map update before replan"
        // reasoning.
        const bool consumeLocalRouteBlockedReplan = localRouteBlockedPendingReplan;

        // Compute this frame's forward BODY-clearance telemetry (Phase
        // 13R) - independent of, and complementary to, the point-ray
        // sensor below. Read once here so the trigger/release decision
        // and the HUD telemetry always agree on the exact same value.
        const bool forwardCorridorClear = clearanceProbe.isForwardCorridorClear();

        // Phase 13V human-validation fix: read the three-ray obstacle
        // perception array once, here, BEFORE avoidance.update() below -
        // it now needs per-side hazard distances (to choose a stable turn
        // direction), not just the aggregate obstacleDetected() bool.
        // Reused verbatim for the HUD ray telemetry further below (never
        // recomputed a second time this frame), matching this loop's own
        // established "compute once, reuse" convention (e.g. cliffReadings).
        const robot::visual::ObstacleSensorArrayReadings obstacleRays = obstacleSensorArray.readings();

        // Advance the avoidance state machine (Phase 13R latch; Phase 13V
        // human-validation fix upgraded it to a three-phase
        // TurnAway/AdvanceClear/Inactive incident lifecycle - see
        // ReactiveObstacleAvoidance's own class docs for the full "why").
        // Runs every frame, regardless of manual drive mode, so it stays
        // in sync with the real FSM/sensor/clearance state and is
        // correctly restored the instant manual mode ends (see
        // docs/technical-decisions.md, Phase 13R, "manual priority while
        // latched"). The trigger condition is unchanged from Phase 13Q
        // (section 7): avoidance enabled, the FSM is actually
        // WaitingForObstacleClear, and the forward sensor still reports
        // the obstacle - this only ever starts a NEW incident from
        // Inactive; once TurnAway/AdvanceClear are in progress, the
        // incident's own forwardCorridorClear/displacement bookkeeping
        // drives every further transition (see update()'s own docs).
        // This never calls stateMachine.processEvent()/handleEvent() or
        // injects ObstacleDetected/ObstacleCleared itself -
        // HardwareEventSource observes the sensor's real true -> false
        // edge naturally, once TurnAway has rotated the sensor ray far
        // enough away from the obstacle.
        //
        // Phase 13X blocker fix: `avoidanceSuppressedAfterBlock` briefly
        // withholds re-arming a NEW incident right after
        // ReactiveObstacleAvoidance itself reported
        // localRouteBlockedThisUpdate() (see below) - without this,
        // avoidance would reclaim AutonomousAvoidance authority on the
        // very next frame (obstacleDetected() is a real-time reading that
        // has had no chance to change yet, since the robot has not moved
        // at all the instant after release), before Navigation's freshly-
        // replanned command ever gets a chance to actually turn the
        // robot - each individual TurnAway incident would be bounded, but
        // the AGGREGATE behavior would still be an unbroken sequence of
        // full-sweep incidents, never resolving. The suppression lifts
        // the moment EITHER: the robot's heading has genuinely changed by
        // kAvoidanceResumeHeadingChangeDegrees since the block (by then
        // Navigation's own Aligning phase has had a real chance to steer
        // the robot toward a genuinely different heading); OR the most
        // recent hardware.update() rejected the proposed position due to
        // collision (hardware.collidedLastUpdate(), read one frame
        // naturally-lagging, same as every other "last update" telemetry
        // in this file) - a second, independent deterministic release
        // condition needed because Navigation's own Driving phase (unlike
        // Aligning) translates in a straight line without rotating at
        // all: if the CURRENT waypoint's straight-line direction from
        // wherever the robot actually ended up happens to clip a
        // different obstacle than the originally-planned route did (a
        // real, reproduced scenario - see docs/technical-decisions.md,
        // Phase 13X blocker fix), heading alone would never change and
        // the suppression would never lift, even though the robot is
        // just as genuinely stuck as the original TurnAway incident was.
        // Both conditions are deterministic and physically grounded -
        // never a frame-count/wall-clock timer. Safety/Manual/
        // RobotCollision's own hard guard remain fully active and
        // unaffected throughout - this only withholds the LOCAL reactive
        // layer's own re-arming, never any of those.
        if (avoidanceSuppressedAfterBlock)
        {
            const float headingChangeSinceBlock = std::fabs(robot::visual::shortestSignedHeadingErrorDegrees(
                headingAtLastLocalRouteBlocked, world.robotPose().headingDegrees));
            if (headingChangeSinceBlock >= kAvoidanceResumeHeadingChangeDegrees || hardware.collidedLastUpdate())
            {
                avoidanceSuppressedAfterBlock = false;
            }
        }

        const bool triggerAvoidance = avoidanceEnabled && !avoidanceSuppressedAfterBlock &&
                                       stateMachine.currentState() == robot::RobotState::WaitingForObstacleClear &&
                                       hardware.obstacleDetected();
        const robot::visual::ObstacleHazardSample avoidanceHazard{
            obstacleRays.frontLeftDistance, obstacleRays.frontCenterDistance, obstacleRays.frontRightDistance};
        avoidance.update(avoidanceEnabled, triggerAvoidance, forwardCorridorClear, world.robotPose(),
                          avoidanceHazard);

        // Phase 13X blocker fix: capture the block signal immediately
        // after the update() call that can produce it - `localRouteBlockedPendingReplan`
        // is consumed one frame later (see the map-aware-navigation
        // block's own `forceReplan` computation below) so this incident's
        // final sensor observations have already reached
        // explorationMapper (which runs at the END of this same frame,
        // AFTER navigation) before GridPathPlanner ever reads the map for
        // the forced replan - never solved by adding a second
        // RobotRuntime::step() call.
        if (avoidance.localRouteBlockedThisUpdate())
        {
            avoidanceSuppressedAfterBlock = true;
            headingAtLastLocalRouteBlocked = world.robotPose().headingDegrees;
        }
        localRouteBlockedPendingReplan = avoidance.localRouteBlockedThisUpdate();

        // Compute this frame's cliff-sensor readings and advance the
        // table-edge safety recovery latch (Phase 13S) - runs
        // unconditionally every frame, exactly like avoidance's own
        // update() above, so it stays in sync with the real robot pose
        // regardless of manual/autonomous/FSM state.
        const robot::visual::CliffSensorReadings cliffReadings = cliffSensor.readings();
        tableEdgeSafety.update(cliffReadings, world.robotPose(), world.tableSurface());

        // Phase 13U: this frame's user-facing task status - derived
        // fresh from RobotStateMachine's own public state, never a second
        // authority over the robot (see MissionTask.hpp). Drives the
        // Mission Control HUD panel below, and (Phase 13X) decides which
        // goal map-aware navigation is currently working toward.
        const robot::visual::MissionTask missionTask =
            robot::visual::deriveMissionTask(stateMachine.currentState(), stateMachine.returnHomeReason());

        // Phase 13X: frontier target selection - dirty-driven (a fresh
        // FrontierExplorer::selectTarget() call, itself internally
        // constructing a fresh GridPathPlanner, only when actually
        // needed), never re-run every frame (this phase's own brief). A
        // new target is needed when: Haritalama just became the active
        // task and none is held yet; the previously-held target's cell no
        // longer qualifies as a frontier (its region has been explored);
        // or mapNavigator reports Failed/Arrived for the current target
        // (Failed -> blacklist it and pick another; Arrived -> this
        // frontier is reached, seek the next one). Outside Haritalama,
        // any held target/blacklist is dropped, so a fresh Start
        // Haritalama session always begins target selection clean (this
        // phase's own brief on map-reset/new-session semantics).
        const bool roaming = missionTask == robot::visual::MissionTask::Roam;
        if (!roaming)
        {
            currentFrontierTarget.reset();
            frontierBlacklist.clear();
            framesSinceLastFrontierAttempt = 0;
            consecutiveNoTargetFound = 0;
        }
        else
        {
            bool needNewTarget = !currentFrontierTarget.has_value();
            if (currentFrontierTarget.has_value())
            {
                int targetCol = -1;
                int targetRow = -1;
                const bool stillFree = explorationMap.worldToCell(currentFrontierTarget->worldPosition, targetCol,
                                                                    targetRow) &&
                                        explorationMap.cellAt(targetCol, targetRow) == robot::visual::MapCell::Free;
                if (!stillFree)
                {
                    needNewTarget = true;
                }
            }
            if (mapNavigator.state() == robot::visual::WaypointNavigatorState::Failed)
            {
                if (currentFrontierTarget.has_value())
                {
                    frontierBlacklist.push_back(currentFrontierTarget->cell);
                }
                needNewTarget = true;
            }
            if (mapNavigator.state() == robot::visual::WaypointNavigatorState::Arrived)
            {
                needNewTarget = true;
            }

            // Phase 13X "not merely waiting for a temporary map update"
            // debounce: once no target is currently held at all (as
            // opposed to a fresh Failed/Arrived/stale transition, which
            // always retries immediately below), further attempts are
            // throttled to once every kFrontierRetryIntervalFrames -
            // while no target is held, plain Fsm MoveForward keeps
            // driving the robot (Navigation authority is not engaged),
            // so the map keeps growing between attempts. A just-formed
            // explored blob's border frequently has not yet grown a
            // cluster past kMinimumFrontierClusterSize on the very first
            // check - treating that transient state as genuine
            // completion would send the robot home before it has
            // explored anywhere near the dock, and Return Home would then
            // correctly (per this phase's own Unknown-blocked policy)
            // fail to find a route through the still-Unknown space
            // between it and the dock, deadlocking. completionSignal is
            // only ever latched true once kRequiredConsecutiveNoTargetAttempts
            // consecutive THROTTLED attempts all agree nothing is
            // reachable.
            if (needNewTarget && !currentFrontierTarget.has_value() &&
                framesSinceLastFrontierAttempt < kFrontierRetryIntervalFrames)
            {
                needNewTarget = false;
            }

            if (needNewTarget)
            {
                framesSinceLastFrontierAttempt = 0;

                // Phase 13X blocker fix: if NO frontier cell exists
                // anywhere on the map at all (ExplorationCompletion::
                // Complete - see FrontierExplorer.hpp), that is an
                // immediate, non-transient fact - it can never improve by
                // waiting, since the map cannot change without the robot
                // moving, and the robot is not moving while no target is
                // held. Confirmed on the spot, bypassing the
                // kRequiredConsecutiveNoTargetAttempts debounce below
                // (which exists ONLY for the different, genuinely
                // transient ExplorationCompletion::NoReachableFrontier
                // case - frontier cells exist, but reachability keeps
                // failing, which CAN legitimately improve as the map
                // grows). This closes a real, reproduced risk: the
                // original debounce let the robot drive blindly forward
                // (plain Fsm MoveForward, zero map awareness) for up to
                // kFrontierRetryIntervalFrames * kRequiredConsecutiveNoTargetAttempts
                // frames even when the map was ALREADY fully known (e.g.
                // a resumed/loaded already-complete map) - see
                // docs/technical-decisions.md, Phase 13X blocker fix, for
                // the full reproduced incident this caused. No extra map
                // scan cost: selectTarget() below would call the exact
                // same frontierCells() detection internally regardless.
                if (frontierExplorer.frontierCells().empty())
                {
                    currentFrontierTarget.reset();
                    completionSignal.complete = explorationMap.exploredCellCount() > 0;
                    consecutiveNoTargetFound = 0;
                }
                else
                {
                    const robot::visual::FrontierTarget target =
                        frontierExplorer.selectTarget(world.robotPose().position, frontierBlacklist);
                    if (target.found)
                    {
                        currentFrontierTarget = target;
                        completionSignal.complete = false;
                        consecutiveNoTargetFound = 0;
                    }
                    else
                    {
                        currentFrontierTarget.reset();
                        ++consecutiveNoTargetFound;
                        // Never "complete" before any exploration has
                        // actually happened yet (e.g. the very first frame
                        // of a fresh Haritalama session, before the
                        // robot's own footprint has produced a single Free
                        // cell) - see docs/technical-decisions.md (Phase
                        // 13X).
                        completionSignal.complete =
                            explorationMap.exploredCellCount() > 0 &&
                            consecutiveNoTargetFound >= kRequiredConsecutiveNoTargetAttempts;
                    }
                }
            }
            else
            {
                ++framesSinceLastFrontierAttempt;
            }
        }

        // Phase 13X: map-aware waypoint navigation - now drives BOTH
        // Return Home (goal = world.basePlatform().position) and, while
        // Haritalama holds a frontier target, exploration navigation
        // (goal = the current frontier target). `forceReplan` fires on
        // the exact frame avoidance or Safety just released, so the
        // robot replans from wherever it ACTUALLY ended up rather than
        // continuing to chase a waypoint that may now be behind/beside it
        // (this phase's own brief, "replanning + avoidance" - the core
        // fix for the human-observed Return Home spin problem). Runs
        // AFTER runtime.step() above, so a same-frame arrival is
        // naturally consumed by the polling composite on the NEXT frame -
        // the same accepted, deterministic one-frame latency Phase 13T
        // already established for plain HomeNavigator.
        // Phase 13X blocker fix: derived from `missionTask` (already
        // resume-aware via MissionTask::deriveMissionTask()'s own
        // resumeState_ handling - see MissionTask.hpp), NOT
        // hardware.currentCommand() (which collapses to Stopped the
        // moment WaitingForObstacleClear's stop() runs - RobotController
        // is unchanged and still does exactly that). Without this,
        // WaypointNavigator itself would be force-disabled for the ENTIRE
        // WaitingForObstacleClear pause (mirroring
        // navigationEnabled=false -> reset()), so once
        // ReactiveObstacleAvoidance suppresses its own re-arming after a
        // LocalRouteBlocked handoff (see the avoidance-wiring block
        // above), NOTHING would be left able to drive the robot at
        // all - Fsm's own Stopped intent is the lowest authority and
        // Navigation was disabled too, a silent total deadlock (the
        // robot never moves, the suppression never lifts since heading
        // never changes, obstacleDetected() never clears since the robot
        // never moves away - reproduced and confirmed during this
        // phase's own investigation). This is an orchestration-layer
        // decision only (which input feeds `navigationEnabled` here in
        // main3d.cpp) - RobotController/RobotStateMachine/DriveAuthority
        // ordering are all unchanged; Navigation still only ever outranks
        // Fsm, exactly as before.
        const bool returningHome = missionTask == robot::visual::MissionTask::ReturnHome;
        const bool navigationEnabled = returningHome || (roaming && currentFrontierTarget.has_value());
        const robot::visual::Vec3 navigationGoal =
            returningHome ? world.basePlatform().position
                          : (currentFrontierTarget.has_value() ? currentFrontierTarget->worldPosition
                                                                 : robot::visual::Vec3{});
        // Phase 13X blocker fix: `consumeLocalRouteBlockedReplan` (captured
        // at the TOP of this frame, before this frame's own avoidance
        // update could overwrite the pending flag - see that capture's
        // own docs) is the deferred, map-fresh trigger for the
        // ReactiveObstacleAvoidance handoff: "Navigation stops using the
        // stale route... WaypointNavigator forces GLOBAL replanning from
        // CURRENT pose." If the replanned route is STILL unreachable,
        // WaypointNavigator reports Failed, which the frontier-selection
        // block above already blacklists on (see its own `needNewTarget`
        // Failed-state handling) - never a second, duplicate blacklist
        // mechanism here.
        const bool forceReplan = consumeLocalRouteBlockedReplan ||
                                  (previousAvoidanceActive && !avoidance.active()) ||
                                  (previousSafetyActive && !tableEdgeSafety.active());
        const robot::visual::WaypointNavigatorOutput navOutput =
            mapNavigator.update(world.robotPose(), explorationMap, navigationGoal, navigationEnabled, forceReplan);

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
            const robot::visual::WheelSpeeds turnSpeeds = avoidance.wheelSpeeds();
            hardware.setAutonomousWheelSpeeds(turnSpeeds.left, turnSpeeds.right);
        }
        else if (hardware.autonomousOverrideActive())
        {
            hardware.clearAutonomousWheelOverride();
        }

        // The navigation override is kept in sync with WaypointNavigator
        // unconditionally, even while manual/autonomous/safety currently
        // wins physically - exactly the same always-latched-underneath
        // pattern the safety/autonomous overrides above already use, so
        // navigation resumes automatically the instant a higher authority
        // releases, with no need to be re-triggered. Only set while
        // Following: Arrived/Inactive/Failed need no override (Failed
        // reports zero wheel speeds itself; the FSM-mapped speeds for
        // ReturnToBase are already zero, and plain Fsm MoveForward simply
        // takes back over for Roam once no frontier target is held).
        const bool navigationDriving = navOutput.state == robot::visual::WaypointNavigatorState::Following;
        if (navigationDriving)
        {
            hardware.setNavigationWheelSpeeds(navOutput.wheelSpeeds.left, navOutput.wheelSpeeds.right);
        }
        else if (hardware.navigationOverrideActive())
        {
            hardware.clearNavigationWheelOverride();
        }

        // Edge-detection anchors for next frame's forceReplan check (see
        // above) - updated last, after both avoidance/safety have already
        // been read for every decision this frame.
        previousAvoidanceActive = avoidance.active();
        previousSafetyActive = tableEdgeSafety.active();

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

        // Phase 13V: the robot's physical pose and this frame's real
        // sensor observations are both final for this frame now (after
        // hardware.update() above, before rendering below) - exactly
        // where the phase brief says to update the map. Reuses
        // VirtualObstacleSensorArray's own existing ray/AABB math
        // (obstacleSensorArray.observations(), Phase 13V) - never a
        // second, duplicated implementation inside ExplorationMapper.
        // Runs unconditionally every frame (even while worldPaused, or
        // regardless of which task/authority currently drives the
        // wheels) - both explorationMapper.update() and
        // coverageTrail.update() are idempotent/distance-sampled
        // respectively, so re-observing an unchanged pose is harmless,
        // and this is the one call site that keeps the map/trail
        // genuinely representing "everywhere the robot has physically
        // been," matching the Explore/Return Home/Safety-recovery/
        // Avoidance/manual-drive trail-lifecycle requirement (Phase 13V
        // brief) with no per-task branching needed.
        {
            const std::array<robot::visual::RangeObservation, 3> rayObservations = obstacleSensorArray.observations();
            const std::vector<robot::visual::RangeObservation> observationList(rayObservations.begin(),
                                                                                 rayObservations.end());
            explorationMapper.update(world.robotPose(), observationList);
        }
        coverageTrail.update(world.robotPose());

        // Periodic save-when-dirty (Phase 13V brief, "do NOT write every
        // frame") - both consumeDirty() calls always run (never short-
        // circuited), so a dirty trail is never missed merely because
        // the map happened to be clean this interval, or vice versa.
        if (mapPersistenceAvailable)
        {
            mapSaveTimer += GetFrameTime();
            if (mapSaveTimer >= kMapSaveIntervalSeconds)
            {
                mapSaveTimer = 0.0F;
                const bool mapDirty = explorationMap.consumeDirty();
                const bool trailDirty = coverageTrail.consumeDirty();
                if (mapDirty || trailDirty)
                {
                    robot::visual::ExplorationMapStorage::save(mapStoragePath, explorationMap, &coverageTrail);
                }
            }
        }

        robot::visual::VisualTelemetry telemetry{};
        // Final UI/HUD polish: these six fields are now populated through
        // TurkishText.hpp's turkishText() overloads instead of each type's
        // own English toString() - main3d.cpp is the one place in this
        // codebase allowed to know both the real domain/visual-simulation
        // enums AND the Turkish presentation mapping (see TurkishText.hpp's
        // own docs for why this must not move into Renderer3D). The
        // English toString() overloads themselves are untouched and still
        // used everywhere else (CLI report output, existing tests).
        telemetry.stateText = robot::visual::turkishText(stateMachine.currentState());
        telemetry.commandText = robot::visual::turkishText(hardware.currentCommand());
        // Manual-validation bugfix: optional Ayrıntılı(detailed)-HUD-only
        // diagnostic - Renderer3D never uses this to decide anything, it
        // only displays it.
        telemetry.returnHomeReasonText = robot::visual::turkishText(stateMachine.returnHomeReason());
        telemetry.sensorOrigin = sensor.sensorOrigin();
        telemetry.sensorDirection = sensor.sensorDirection();
        telemetry.obstacleDistance = sensor.distanceToNearestObstacle();
        telemetry.obstacleDetected = sensor.obstacleDetected();
        telemetry.sensorMaximumRange = robot::visual::VirtualDistanceSensor::kMaximumRange;
        const robot::visual::WheelSpeeds wheelSpeeds = hardware.wheelSpeeds();
        telemetry.leftWheelSpeed = wheelSpeeds.left;
        telemetry.rightWheelSpeed = wheelSpeeds.right;
        telemetry.driveAuthorityText = robot::visual::turkishText(hardware.driveAuthority());
        telemetry.collidedLastUpdate = hardware.collidedLastUpdate();
        telemetry.batteryPercent = hardware.batteryLevelPercent();
        telemetry.mapWasLoaded = mapWasLoadedAtStartup;
        telemetry.avoidanceEnabled = avoidanceEnabled;
        telemetry.avoidanceActive = avoidance.active();
        telemetry.avoidanceStateText = robot::visual::turkishText(avoidance.state());
        telemetry.forwardClearanceClear = forwardCorridorClear;
        telemetry.clearanceLookahead = robot::visual::ForwardClearanceProbe::kLookaheadDistance;
        telemetry.cliffFrontLeft = cliffReadings.frontLeft;
        telemetry.cliffFrontRight = cliffReadings.frontRight;
        telemetry.cliffRearLeft = cliffReadings.rearLeft;
        telemetry.cliffRearRight = cliffReadings.rearRight;
        telemetry.edgeSafetyActive = tableEdgeSafety.active();
        telemetry.edgeRecoveryStateText = robot::visual::turkishText(tableEdgeSafety.state());
        telemetry.edgeTargetHeadingDegrees = tableEdgeSafety.targetRecoveryHeadingDegrees();
        telemetry.edgeHeadingErrorDegrees = tableEdgeSafety.currentHeadingErrorDegrees();

        // Manual-validation bugfix: left/right ray telemetry - center
        // reuses telemetry.sensorOrigin/obstacleDistance/obstacleDetected
        // above (geometrically identical to the array's FrontCenter ray,
        // never recomputed twice). bodyCorridorObstacleHazard mirrors the
        // exact same hazard signal hardware.obstacleDetected() itself ORs
        // in - see VirtualRobotHardware::bodyCorridorObstacleHazard().
        // obstacleRays itself was already computed once, earlier this
        // frame, before avoidance.update() (Phase 13V human-validation
        // fix) - reused here, never recomputed twice.
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

        // Phase 13X: already-computed WaypointNavigator telemetry -
        // Renderer3D never has any notion of the Inactive/Following/
        // Arrived/Failed policy itself. The guide line (a straight visual
        // aid toward world.basePlatform()) is only meaningful while
        // actually returning home - during frontier exploration the
        // planned-route polyline (passed to renderFrame() below) is the
        // correct visualization instead, never this straight-line guide.
        telemetry.homeNavigationStateText = robot::visual::turkishText(navOutput.state);
        telemetry.homeNavigationGuideVisible = returningHome && navigationDriving;
        if (navOutput.currentWaypointIndex < navOutput.route.size())
        {
            const robot::visual::Vec3& currentWaypoint = navOutput.route[navOutput.currentWaypointIndex];
            const robot::visual::Vec3& robotPosition = world.robotPose().position;
            const float dx = currentWaypoint.x - robotPosition.x;
            const float dz = currentWaypoint.z - robotPosition.z;
            telemetry.homeNavigationDistance = std::sqrt((dx * dx) + (dz * dz));
            telemetry.homeNavigationTargetHeadingDegrees =
                robot::visual::normalizeHeadingDegrees(robot::visual::headingDegreesFromDirection(dx, dz));
            telemetry.homeNavigationHeadingErrorDegrees = robot::visual::shortestSignedHeadingErrorDegrees(
                world.robotPose().headingDegrees, telemetry.homeNavigationTargetHeadingDegrees);
        }
        else
        {
            telemetry.homeNavigationDistance = 0.0F;
            telemetry.homeNavigationTargetHeadingDegrees = 0.0F;
            telemetry.homeNavigationHeadingErrorDegrees = 0.0F;
        }

        // Phase 13X: the current frontier exploration target (if any) -
        // drawn as a small marker on the HARİTA panel; never visible
        // during Return Home or once no target is held.
        telemetry.frontierTargetVisible = roaming && currentFrontierTarget.has_value();
        if (telemetry.frontierTargetVisible)
        {
            telemetry.frontierTargetPosition = currentFrontierTarget->worldPosition;
        }

        // Phase 13U: Mission Control panel telemetry - main3d computes
        // task status and base distance directly (the same simple
        // dx/dz-from-pose formula HomeNavigator itself independently uses
        // internally for its own decisions); this is a second,
        // presentation-only computation purely for display, exactly like
        // "Position: X/Z" above already reads world.robotPose() directly
        // rather than through another component. Phase 13V human-
        // validation fix: no longer also computes a Home-Zone inside/
        // outside boolean - that telemetry field and its Mission Control
        // panel line were removed along with HomeZoneMonitor's production
        // wiring, since a distance-only fact that no longer affects
        // behavior would only be presentation clutter (see this file's
        // own docs above).
        telemetry.missionTaskText = robot::visual::turkishText(missionTask);
        telemetry.returningHomeTask = missionTask == robot::visual::MissionTask::ReturnHome;
        {
            const robot::visual::Vec3& robotPosition = world.robotPose().position;
            const robot::visual::Vec3& basePosition = world.basePlatform().position;
            const float dx = basePosition.x - robotPosition.x;
            const float dz = basePosition.z - robotPosition.z;
            telemetry.baseDistance = std::sqrt((dx * dx) + (dz * dz));
        }

        // Camera updates (mouse-look and CAMERA_FREE's own arrow-key
        // pitch/yaw) are suppressed while manual drive mode is active -
        // see the manualDriveMode comment above. cameraCaptured still
        // governs cursor capture/release via TAB independently of this.
        const bool updateCamera = cameraCaptured && !manualDriveMode;
        renderer.renderFrame(world, updateCamera, telemetry, explorationMap, coverageTrail, navOutput.route);
    }

    // Phase 13V: one final unconditional save on clean shutdown (this
    // phase's own brief, "save... on clean application shutdown") -
    // regardless of the periodic dirty-check timer above, so a session
    // that ends less than kMapSaveIntervalSeconds after its last change
    // is never silently lost.
    if (mapPersistenceAvailable)
    {
        robot::visual::ExplorationMapStorage::save(mapStoragePath, explorationMap, &coverageTrail);
    }

    CloseWindow();
    return 0;
}
