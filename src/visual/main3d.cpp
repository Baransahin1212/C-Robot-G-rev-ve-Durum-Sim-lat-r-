#include <array>
#include <cmath>
#include <limits>
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
#include "robot/visual/DockApproachArrivalEventSource.hpp"
#include "robot/visual/DockApproachController.hpp"
#include "robot/visual/DockCaptureRegion.hpp"
#include "robot/visual/DockChargingContacts.hpp"
#include "robot/visual/DockLaneObstacleFilter.hpp"
#include "robot/visual/ExecutableDirectory.hpp"
#include "robot/visual/ExplorationCompletionEventSource.hpp"
#include "robot/visual/ExplorationMap.hpp"
#include "robot/visual/ExplorationMapStorage.hpp"
#include "robot/visual/ExplorationMapper.hpp"
#include "robot/visual/ExplorationSessionReset.hpp"
#include "robot/visual/ForwardClearanceProbe.hpp"
#include "robot/visual/FrontierExplorer.hpp"
#include "robot/visual/GridPathPlanner.hpp"
#include "robot/visual/ManualDriveInput.hpp"
#include "robot/visual/MapResetController.hpp"
#include "robot/visual/MissionControlEventSource.hpp"
#include "robot/visual/MissionTask.hpp"
#include "robot/visual/RangeObservation.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/Renderer3D.hpp"
#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/TableEdgeSafetyController.hpp"
#include "robot/visual/TurkishText.hpp"
#include "robot/visual/VirtualCliffSensor.hpp"
#include "robot/visual/VirtualDistanceSensor.hpp"
#include "robot/visual/VirtualObstacleSensorArray.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"
#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualMath.hpp"
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

// Phase 13Z: how long the New Map (`N`) "stop the mission first" and "new
// map created" HUD notices stay visible - both transient, presentation-
// only, and a distinct concern from MapResetController's own
// kConfirmationWindowSeconds (whether a SECOND press still counts as
// confirming, not how long a one-shot notice stays on screen).
constexpr float kMapResetNoticeDisplayDurationSeconds = 3.0F;

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

// Phase 13X blocker fix (deadlock repair): the ORIGINAL
// hardware.collidedLastUpdate() release condition (see
// avoidanceSuppressedAfterBlock's own docs) fires on a SINGLE collided
// frame - the instant Navigation's freshly-replanned Driving phase clips
// an obstacle even once, avoidance immediately re-arms and steals the
// wheels back for a brand new bounded TurnAway sweep, before Navigation
// (or its own route-invalidation/NavigationProgressTracker machinery) ever
// gets more than exactly one frame to actually attempt the escape - a
// second, slower-motion ping-pong between the two authorities (each
// individually bounded - a ~44-frame TurnAway sweep, then one Navigation
// frame - but the AGGREGATE never converging), reproduced by a full
// autonomous Haritalama+Return Home session once the first (Safety-tier)
// deadlock above no longer masks it. Requiring
// kAvoidanceResumeCollisionStallFrames CONSECUTIVE collided frames (never
// merely one) before honoring the collision-based release gives Navigation
// a genuine multi-frame window each time, without weakening the heading-
// change release condition (an unambiguously positive progress signal,
// never debounced). Small (a quarter-second at this project's ~0.05s/frame
// convention) - just enough to distinguish "Navigation is persistently
// wedged against this exact obstacle" from "a single frame's proposed step
// happened to graze one," never large enough to reintroduce a long stall.
constexpr int kAvoidanceResumeCollisionStallFrames = 5;

// Obstacle-deadlock fix (real-GUI-reproduced): a real GUI trace proved
// BOTH release conditions above share one unstated assumption - SOME
// authority is currently able to move the robot (Navigation replanning
// around the blocked cell, or the robot colliding while attempting to).
// When nothing can (e.g. Haritalama with no reachable frontier target -
// map already logically Complete - so `navigationEnabled` is false, Safety
// is inactive, and Manual is off), neither the heading-change nor the
// collision-streak condition can ever become true: the robot sits at
// exactly zero wheel speed forever, `avoidanceSuppressedAfterBlock` never
// releases, and ReactiveObstacleAvoidance never gets to attempt a fresh
// incident - a genuine, reproduced PERMANENT WaitingForObstacleClear
// deadlock, distinct from (and not fixed by) either existing condition.
// This third, unconditional, bounded-frame-count valve is the fix -
// mirrors TableEdgeSafetyController's own kSafetyResumeMaxSuppressedFrames
// last-resort valve exactly (same magnitude/reasoning: generous enough for
// a genuine bounded avoidance/replan attempt to run its own course when
// one IS possible, but never allowed to withhold avoidance indefinitely
// when none is). Deterministic - a plain frame count, never randomized,
// never reset by anything other than a fresh block or a genuine release -
// so re-arming is fully reproducible for every run. Once this valve fires,
// avoidance is simply un-suppressed again: ReactiveObstacleAvoidance's own
// existing Inactive-state entry condition (triggerAvoidance true) is what
// actually starts the next incident, choosing a fresh deterministic turn
// direction from that frame's real hazard sample - never a special-cased
// "forced" incident of its own.
constexpr int kAvoidanceSuppressionMaxFrames = 120;

// Phase 13X blocker fix (deadlock repair): minimum positional change
// (world units), since TableEdgeSafetyController last reported
// recoveryBlockedThisUpdate(), before its own re-arming is allowed again -
// see safetyRecoverySuppressedAfterBlock's own docs below for the full
// "why." Uses RobotCollision::kRobotCollisionRadius as its natural scale
// (the same "one full body-radius of real displacement proves genuine
// progress" reasoning ReactiveObstacleAvoidance::kMinimumBypassDistanceWorldUnits
// already uses for its own AdvanceClear release condition) rather than an
// arbitrary literal.
const float kSafetyResumeMinDisplacementWorldUnits = robot::visual::kRobotCollisionRadius;

// Phase 13X blocker fix: unconditional last-resort valve on how long
// TableEdgeSafetyController's own re-arming may be withheld - never an
// indefinite suppression regardless of what avoidance/Navigation manage to
// do with the window. 120 frames (6s of simulated time at this project's
// ~0.05s/frame convention) is generous enough for a genuine bounded
// avoidance/replan attempt to run its own course, small relative to the
// thousands of frames a full autonomous session already budgets.
constexpr int kSafetyResumeMaxSuppressedFrames = 120;

// Phase 13X final blocker fix ("BLOCKED-CELL REPLAN AUDIT" -
// docs/technical-decisions.md): excluded neighborhood radius (in grid
// cells) around a waypoint cell an avoidance incident just proved
// troublesome, not merely that ONE exact cell. Proven necessary by a
// traced reproduction: blacklisting only the single exact cell let
// GridPathPlanner's own A* immediately re-select the immediately-ADJACENT,
// functionally-equivalent cell (same clearance, same connectivity to the
// rest of the route), producing a near-identical route that failed again
// one grid cell over - a deterministic single-cell "walk" that never
// converged. 0 (the exact single cell only, no extra ring) was proven
// (same traced reproduction/regression) to already be the right size once
// combined with kMaxReturnHomeBlockedCells' own bounded-eviction below -
// a 3x3 ring around EVERY incident, accumulated over an extended Return
// Home session with the trigger below (any avoidance release, not just a
// full-sweep exhaustion), was empirically proven to over-exclude the
// dock's own already-tight goal-snapped approach area entirely
// (WaypointNavigatorState::Failed - a real regression this file's own
// test caught), which the brief's own "never arbitrarily large regions"
// constraint explicitly warns against.
constexpr int kLocalReplanExclusionRadiusCells = 0;

// Phase 13X final blocker fix: hard cap on how many cells
// returnHomeBlockedCells may ever hold at once - once exceeded, the
// OLDEST entries are evicted first (see the FIFO eviction below). This is
// what keeps the exclusion genuinely TEMPORARY/bounded (never an
// unbounded accumulation across a long Return Home session that could
// eventually wall off the goal's own neighborhood) rather than relying on
// eviction never being needed. Small relative to the ~100x100 exploration
// grid - comfortably enough live exclusions to break a short repeating
// cycle, never enough to meaningfully disconnect a real desk-scale
// region.
constexpr std::size_t kMaxReturnHomeBlockedCells = 12;

// Phase 13X final blocker fix ("NO OSCILLATION INVARIANT" /
// "BLOCKED-CELL REPLAN AUDIT"): two regression-tuned constants for the
// WHOLE-SESSION distance-to-home progress tracker (see the tracking
// block's own docs, below in main() itself, for the full reasoning). Two
// EARLIER, per-incident heuristics were tried and both regression-tested
// away by this project's own pre-existing Return Home test suite
// (unconditional blacklist-on-every-ordinary-release, and a
// "did-this-one-incident-net-improve-distance-to-home" heuristic): a
// single avoidance incident is frequently perfectly benign and fully
// recoverable (this codebase's whole reactive-avoidance layer exists
// precisely to handle that case), and even a genuinely converging route
// routinely includes individual incidents/legs that temporarily move the
// robot sideways or away from home as a completely normal, NECESSARY part
// of getting around an obstacle - judging any SINGLE incident in
// isolation cannot reliably tell that apart from a genuine non-converging
// attractor. A WHOLE-SESSION window can: kOscillationProgressEpsilonWorldUnits
// (world units) is how much distance-to-home must improve to count as
// genuine progress (small - just enough to ignore floating-point/near-
// zero noise); kOscillationStuckWindowFrames is how many consecutive
// frames may pass without that much improvement before the current
// waypoint is treated as the responsible attractor and blacklisted (see
// the tracking block's own docs for the exact accounting) - large enough
// that any genuinely converging route (including one with several normal,
// temporary detour-related non-improving stretches) never spuriously
// crosses it within this project's own regression suite, small enough
// relative to the 4000-frame Return Home budget that a real attractor is
// caught and corrected with many frames still available to actually reach
// home afterward. `localRouteBlockedThisUpdate()` (the full-360-degree-
// sweep-exhausted signal) remains its own, separate, unconditional,
// immediate trigger - see below - since that signal is already a
// complete, one-incident proof the exact cell is not passable.
constexpr float kOscillationProgressEpsilonWorldUnits = 0.05F;
constexpr int kOscillationStuckWindowFrames = 400;

// Phase 13X final blocker fix: shared helper - adds the
// (radiusCells x radiusCells) neighborhood around (centerCol, centerRow)
// to `blocked` (deduplicated), then enforces `maxEntries` via FIFO
// eviction of the OLDEST entries - the one place this logic lives, used
// by both blacklist triggers below (never two independently-drifting
// copies of the same bookkeeping).
void addBlockedCellNeighborhood(std::vector<robot::visual::GridCoord>& blocked, int centerCol, int centerRow,
                                 int radiusCells, std::size_t maxEntries)
{
    for (int dRow = -radiusCells; dRow <= radiusCells; ++dRow)
    {
        for (int dCol = -radiusCells; dCol <= radiusCells; ++dCol)
        {
            const robot::visual::GridCoord candidate{centerCol + dCol, centerRow + dRow};
            bool alreadyListed = false;
            for (const robot::visual::GridCoord& existing : blocked)
            {
                if (existing == candidate)
                {
                    alreadyListed = true;
                    break;
                }
            }
            if (!alreadyListed)
            {
                blocked.push_back(candidate);
            }
        }
    }
    while (blocked.size() > maxEntries)
    {
        blocked.erase(blocked.begin());
    }
}

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

    // Phase 13X final-approach fix: sits directly in front of
    // hardwareEventSource in the composite chain below - see
    // DockLaneObstacleFilter.hpp's own docs for the exact deadlock this
    // closes (a robot correctly parked at the dock, permanently detecting
    // its own rear housing, with no path back out of
    // WaitingForObstacleClear). `setSuppressed()` is called once per frame,
    // before runtime.step() - see that call site's own docs.
    robot::visual::DockLaneObstacleFilter dockLaneFilter(hardwareEventSource);

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
    // orchestrator that DRIVES Stage 1 of Return Home (global routing
    // through known Free space) and, while Haritalama is actively seeking
    // a frontier target, exploration navigation too - it owns its own
    // internal HomeNavigator for per-waypoint local steering (HomeNavigator
    // itself is UNCHANGED - still exists, still does exactly the Aligning/
    // Driving/Arrived job it always has - see HomeNavigator.hpp), asked
    // each frame to steer toward the CURRENT waypoint of a GridPathPlanner-
    // produced route rather than blindly straight at a possibly-obstructed
    // final goal. The old plain HomeNavigator + HomeArrivalEventSource
    // production wiring is gone from this file (both classes remain fully
    // compiled/tested elsewhere - see docs/technical-decisions.md, Phase
    // 13X, and HomeZoneMonitor's own earlier precedent for a component
    // staying compiled/tested after its production wiring changes).
    //
    // Phase 13X final-approach fix, extended by Phase 13Y's precision
    // reverse docking: for Return Home specifically, WaypointNavigator's
    // own goal is now computeDockStagingPoint() (see
    // DockApproachController.hpp) - a point comfortably in front of the
    // dock, never the literal dock/base position itself. Global A*
    // planning is deliberately NOT responsible for precision docking (see
    // that header's own class docs for the full "why" - human GUI
    // validation found Phase 13X's own literal-goal final segment fully
    // exposed to ReactiveObstacleAvoidance, and separately, the human
    // product requirement is now realistic reverse parking with physical
    // charging-contact alignment, never a nose-first drive-through).
    // DockApproachController is Stage 2 - the small, dedicated "rotate so
    // the rear faces the dock, then reverse the last short validated
    // stretch until both charging contacts mate" controller that takes
    // over once WaypointNavigator reports Arrived at the staging point
    // (see the Return Home orchestration block, below in this function,
    // for exactly how the two stages hand off wheel authority).
    // WaypointArrivalEventSource (still fully compiled/tested, matching
    // HomeNavigator/HomeArrivalEventSource's own precedent) is
    // DELIBERATELY not wired into this file's event chain any more - its
    // own Arrived-edge would now fire the instant WaypointNavigator merely
    // reaches the staging point, which must NEVER by itself end a Return
    // Home mission (it is Stage 1 finishing, not the robot actually being
    // docked). DockApproachArrivalEventSource (below) is the new, correct
    // HomeReached source: edge-triggered on DockApproachController's OWN
    // Docked state, which only ever becomes true once both rear charging
    // contacts are physically aligned with the dock's own pins AND
    // heading is correct (see DockChargingContacts.hpp/
    // DockApproachController.hpp) - the existing HomeReached/Ready
    // semantics are completely unchanged, just now driven by the right
    // controller and a stricter, more physically real arrival condition.
    // Haritalama's own frontier-arrival bookkeeping never depended on
    // WaypointArrivalEventSource's Event either - it already reads
    // mapNavigator.state() directly (see the frontier-selection block
    // below) - so removing it from the event chain has no effect there.
    robot::visual::WaypointNavigator mapNavigator;
    robot::visual::DockApproachController dockApproach;
    robot::visual::DockApproachArrivalEventSource dockApproachArrivalEventSource(dockApproach);

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
    robot::CompositePollingEventSource innerCompletionGroup(dockApproachArrivalEventSource,
                                                              explorationCompletionEventSource);
    robot::CompositePollingEventSource innerHardwareGroup(dockLaneFilter, innerCompletionGroup);
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

    // Phase 13Y dock-capture LATCH fix (real-GUI-traced) - mirrors
    // `dockNeedsStage1ReplanPending`'s own one-frame sticky-read shape
    // exactly. `dockCapturedPending` is set at the END of each frame from
    // that frame's own fresh `dockOutput.captured` (see that field's own
    // docs); `previousDockCapturedSticky` remembers what the STICKY read
    // was the previous time it was computed, purely so the
    // NotCaptured->Captured EDGE (not just the level) can be detected at
    // the top of the frame, before runtime.step() - see the dock-hazard-
    // suppression block's own docs for why the edge specifically is
    // needed (releasing a stale dock-attributable avoidance incident
    // exactly once, not every frame capture remains true).
    bool dockCapturedPending = false;
    bool previousDockCapturedSticky = false;

    // Phase 13X human-validation fix: edge-detection for "Return Home just
    // became the active task" (missionTask freshly reading
    // MissionTask::ReturnHome this frame, false -> true) - see the
    // Manual-override-cancellation block below (in the main loop) for the
    // full reasoning. Mirrors previousAvoidanceActive/previousSafetyActive's
    // own shape exactly.
    bool previousReturningHomeActive = false;

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
    // Phase 13Y: mirrors localRouteBlockedPendingReplan's own one-frame
    // sticky-capture shape exactly - see
    // DockApproachOutput::needsStage1Replan's own docs for why this exists
    // (DockApproachController proved the robot is not precisely enough
    // positioned for pure-rotation AlignForReverse, and needs Stage 1's
    // full 2D steering to genuinely re-drive rather than idling on a stale
    // "Arrived").
    bool dockNeedsStage1ReplanPending = false;
    bool avoidanceSuppressedAfterBlock = false;
    float headingAtLastLocalRouteBlocked = 0.0F;
    // Phase 13X blocker fix (deadlock repair): consecutive-collision
    // debounce for the release condition above - see
    // kAvoidanceResumeCollisionStallFrames's own docs.
    int consecutiveCollisionsWhileAvoidanceSuppressed = 0;
    // Obstacle-deadlock fix: unconditional last-resort frame count - see
    // kAvoidanceSuppressionMaxFrames's own docs above for the full "why."
    int framesSuppressedSinceAvoidanceBlock = 0;

    // Phase 13X blocker fix (deadlock repair): mirrors
    // avoidanceSuppressedAfterBlock's own shape, one authority tier up.
    // TableEdgeSafetyController::recoveryBlockedThisUpdate() (bounded
    // recovery-stall escape - see that class's own docs) means Safety
    // just released a translating recovery it could not complete, most
    // often because an obstacle sits between the robot and this
    // incident's fixed recovery target. Safety unconditionally outranks
    // AutonomousAvoidance/Navigation/Fsm (see driveAuthority()), so
    // without withholding its own re-arming for a bit, it would
    // immediately re-claim the wheels the very next frame (cliff sensors
    // read from an all-but-unmoved position almost certainly still report
    // unsafe), starving avoidance/Navigation of the genuine chance to
    // move the robot away from the stuck spot that the release was
    // supposed to grant - recreating the same deadlock one bounded-stall
    // cycle at a time instead of resolving it. Lifted the moment the
    // robot's position has moved meaningfully since the block (avoidance
    // or Navigation made real progress - Safety should resume watching
    // from the new position), or after kSafetyResumeMaxSuppressedFrames
    // as an unconditional last-resort valve. VirtualRobotHardware's own
    // unconditional last-resort guards (obstacle-collision rejection,
    // all-four-corners-off-table rejection) remain fully active
    // throughout regardless - this only ever withholds the PROACTIVE
    // recovery layer's own re-arming, never the hard backstop.
    bool safetyRecoverySuppressedAfterBlock = false;
    robot::visual::Vec3 positionAtLastSafetyBlock{};
    int framesSuppressedSinceSafetyBlock = 0;

    // Phase 13X blocker fix (deadlock repair): Return-Home-only cell
    // blacklist (mirrors frontierBlacklist's own shape, one level more
    // granular) - see GridPathPlanner's own `extraBlockedCells`
    // constructor-parameter docs for the full "why." Grows only when
    // avoidance's bounded TurnAway sweep proves (via
    // localRouteBlockedThisUpdate()) that the specific waypoint cell
    // Navigation was steering toward is not really passable; cleared
    // whenever Return Home is not the active task, so a fresh Return Home
    // request always begins with a clean slate, exactly like
    // frontierBlacklist's own reset-on-task-change semantics.
    // `previousNavOutput` is the one piece of state needed to know WHICH
    // waypoint cell was being pursued at the moment a block is detected
    // (mapNavigator.update() itself has not run yet this frame when
    // avoidance's block is read).
    std::vector<robot::visual::GridCoord> returnHomeBlockedCells;
    robot::visual::WaypointNavigatorOutput previousNavOutput;
    // Phase 13X final blocker fix ("NO OSCILLATION INVARIANT"): whole-
    // Return-Home-session distance-to-home progress bookkeeping - see the
    // tracking block's own docs (in main()) for the full reasoning.
    // `bestDistanceToHomeThisSession` starts at +infinity so the very
    // first frame of a fresh Return Home always counts as an improvement.
    float bestDistanceToHomeThisSession = std::numeric_limits<float>::infinity();
    int framesSinceDistanceImproved = 0;

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

    // Phase 13X quick fix (Bug B): true from the moment KEY_ONE finds the
    // map already logically complete (see that handler above) until a
    // genuine mission task actually becomes active again (cleared below,
    // once missionTask is Roam or ReturnHome) - drives the "Harita zaten
    // tamamlandı" HUD notice. Presentation-only; never influences
    // completionSignal/frontier/navigation logic itself.
    bool mapAlreadyCompleteNoticeActive = false;

    // Phase 13Z: New Map (`N`) - the two-press confirmation gate (see
    // MapResetController.hpp's own class docs) plus the two transient,
    // mutually-exclusive HUD notice countdowns main3d.cpp itself decides
    // when to arm - "stop the mission first" (the state gate rejected the
    // request) and "new map created" (a reset actually happened). Both
    // timers count DOWN to zero (0 = not currently showing);
    // kMapResetNoticeDisplayDurationSeconds is their shared duration.
    robot::visual::MapResetController mapResetController;
    float mapResetRejectedNoticeSecondsRemaining = 0.0F;
    float mapResetSuccessNoticeSecondsRemaining = 0.0F;

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
            // Phase 13X quick fix (Bug B): if the CURRENT map (fresh from
            // a persisted load, or left over from an earlier completed
            // session) already has nothing reachable left to explore, do
            // not begin a Roam session at all - the exploration loop's
            // own needNewTarget check (below) would immediately re-derive
            // that same fact on Roam's very first frame and auto-return
            // home before the robot ever physically moves, which is
            // exactly the reproduced "press 1, mapping stops almost
            // immediately" incident. Uses the SAME frontierCells()/
            // exploredCellCount() facts the exploration loop itself
            // already relies on - never a second, different completion
            // definition. Still transitions Idle -> Ready (via
            // requestScenarioLoadedOnly() below) so the app remains
            // interactive; only the doomed StartMission is withheld.
            const bool mapAlreadyComplete =
                explorationMap.exploredCellCount() > 0 && frontierExplorer.frontierCells().empty();
            if (mapAlreadyComplete)
            {
                mapAlreadyCompleteNoticeActive = true;
                missionControl.requestScenarioLoadedOnly(stateMachine.currentState());
            }
            else
            {
                // Start Roam - edge-triggered, state-aware (see
                // MissionControlEventSource::requestStartRoam()): queues
                // ScenarioLoaded+StartMission from Idle, only StartMission
                // from Ready, or nothing at all if already Moving/
                // ReturningHome/etc. Reads stateMachine.currentState() as
                // of the END of the previous frame's runtime.step() - the
                // correct up-to-date snapshot, since input is handled
                // before this frame's own step() below.
                mapAlreadyCompleteNoticeActive = false;
                missionControl.requestStartRoam(stateMachine.currentState());
            }
        }

        if (IsKeyPressed(KEY_TWO) || IsKeyPressed(KEY_R))
        {
            // Phase 13X quick fix (Bug A, unchanged): an EXPLICIT user
            // Return Home command always means "autonomous control takes
            // over now" - cancel any active Manual override immediately,
            // right here, BEFORE the command composition below, so a
            // currently-held arrow key/the per-frame manual-input block
            // further down cannot re-latch it this same frame. Covers the
            // "Manual re-enabled while already ReturningHome" gap the
            // returningHome-edge cancel (further below) cannot reach on
            // its own, since RobotStateMachine's ReturningHome case has no
            // EventType::ReturnHomeRequested handler (a second request
            // while already returning is a no-op transition either way).
            manualDriveMode = false;
            hardware.clearManualWheelOverride();

            // Phase 13X quick fix (Bug C): `2`/`R` (preserved from Phase
            // 13T, still the exact same one-implementation intent) is now
            // state-aware, mirroring requestStartRoam()'s own Idle-vs-
            // everything-else split. From Idle - e.g. Manual free-drive
            // used directly after launch, with no mission ever started -
            // plain requestReturnHome() is silently rejected forever
            // (RobotStateMachine has no Idle + ReturnHomeRequested
            // transition at all; only Ready does), leaving the robot
            // stuck exactly as human GUI validation reproduced ("Durum:
            // Bekliyor / Görev: YOK" after pressing 2). Composes the
            // SAME two existing, unmodified transitions requestStartRoam()
            // already uses for Idle (ScenarioLoaded -> Ready) with the
            // existing Ready -> ReturningHome transition, via
            // requestReturnHomeFromIdle() - delivered across two separate
            // runtime.step() calls/frames (never more than one Event
            // consumed per step()), invisible to the user as anything
            // other than "I pressed 2 and the robot went home." From any
            // other state (Ready/Moving/ReturningHome/
            // WaitingForObstacleClear/...), the existing unconditional
            // requestReturnHome() is unchanged - Ready/Moving accept it
            // exactly as before, and ReturningHome/WaitingForObstacleClear
            // safely no-op (RobotStateMachine rejects it there), leaving
            // resumeState_ and the current Return Home task alone -
            // Manual was already cleared above regardless.
            if (stateMachine.currentState() == robot::RobotState::Idle)
            {
                missionControl.requestReturnHomeFromIdle();
            }
            else
            {
                missionControl.requestReturnHome();
            }
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

        // Phase 13Z: tick the New Map confirmation window down every
        // frame, unconditionally (mirrors MapResetController::update()'s
        // own per-frame contract - a no-op while no confirmation is
        // pending), before reacting to this frame's own KEY_N below.
        // Independently, count down whichever transient notice (if any)
        // is currently showing.
        mapResetController.update(GetFrameTime());
        if (mapResetRejectedNoticeSecondsRemaining > 0.0F)
        {
            mapResetRejectedNoticeSecondsRemaining -= GetFrameTime();
            if (mapResetRejectedNoticeSecondsRemaining < 0.0F)
            {
                mapResetRejectedNoticeSecondsRemaining = 0.0F;
            }
        }
        if (mapResetSuccessNoticeSecondsRemaining > 0.0F)
        {
            mapResetSuccessNoticeSecondsRemaining -= GetFrameTime();
            if (mapResetSuccessNoticeSecondsRemaining < 0.0F)
            {
                mapResetSuccessNoticeSecondsRemaining = 0.0F;
            }
        }

        if (IsKeyPressed(KEY_N))
        {
            // Phase 13Z: New Map - MapResetController owns the two-press
            // confirmation/state-gate decision (see that class's own
            // docs); this handler only ever reacts to its outcome. A
            // confirmed reset calls resetExplorationSession() (see
            // ExplorationSessionReset.hpp) - the one place this session's
            // map/trail/navigator/frontier/completion state is actually
            // reset, never scattered inline here. Never an FSM event -
            // RobotStateMachine's transition table is completely
            // unmodified by this feature (this phase's own brief).
            const robot::visual::MapResetRequestOutcome outcome =
                mapResetController.requestKeyPress(stateMachine.currentState());
            switch (outcome)
            {
                case robot::visual::MapResetRequestOutcome::ConfirmationRequested:
                    mapResetRejectedNoticeSecondsRemaining = 0.0F;
                    mapResetSuccessNoticeSecondsRemaining = 0.0F;
                    break;
                case robot::visual::MapResetRequestOutcome::RejectedActiveMission:
                    mapResetRejectedNoticeSecondsRemaining = kMapResetNoticeDisplayDurationSeconds;
                    break;
                case robot::visual::MapResetRequestOutcome::Confirmed:
                {
                    robot::visual::ExplorationSessionState session{
                        explorationMap,
                        coverageTrail,
                        mapNavigator,
                        completionSignal,
                        currentFrontierTarget,
                        frontierBlacklist,
                        framesSinceLastFrontierAttempt,
                        consecutiveNoTargetFound,
                        returnHomeBlockedCells,
                        previousNavOutput,
                        bestDistanceToHomeThisSession,
                        framesSinceDistanceImproved,
                        mapAlreadyCompleteNoticeActive,
                        mapSaveTimer,
                        mapWasLoadedAtStartup,
                    };
                    robot::visual::resetExplorationSession(mapStoragePath, mapPersistenceAvailable, session);
                    mapResetSuccessNoticeSecondsRemaining = kMapResetNoticeDisplayDurationSeconds;
                    break;
                }
            }
        }

        // Phase 13X final-approach fix ("FINAL APPROACH HAZARD CONTRACT"),
        // extended for Phase 13Y's precision reverse docking and then again
        // for the dock-capture LATCH fix (real-GUI-traced): read as the
        // STICKY state left by the END of the PREVIOUS frame's own
        // dockApproach.update() call (below, after navOutput) - the same
        // one-frame-lag sticky-read convention consumeLocalRouteBlockedReplan
        // already established for this file, needed here specifically
        // because dockLaneFilter must be armed/disarmed BEFORE
        // runtime.step() below, but this frame's own fresh
        // DockApproachController state is not known until later in the
        // frame.
        //
        // Driven directly off DockApproachOutput::captured (the session
        // LATCH - see that field's own docs) rather than re-deriving
        // eligibility every frame: a real GUI trace proved the raw entry
        // predicate (DockCaptureRegion::isDockCaptureEligible()) can
        // legitimately flicker false again mid-maneuver (this project's
        // real desk layout has a genuine ambiguous region where a desk
        // object briefly becomes numerically nearer than the dock housing),
        // even while the robot is still physically deep inside the
        // validated docking lane - re-deriving eligibility here would
        // re-introduce exactly that flicker into suppression too, exposing
        // the FSM-pause/avoidance-trigger gate again mid-session. Once a
        // session is captured, this stays true for the WHOLE session
        // (never re-earned frame to frame) - it only ever goes false again
        // when the controller itself releases the session (Docked/Failed/
        // enabled=false via `returningHome` ending), never from a momentary
        // hazard-attribution flicker. While DockApproachController owns a
        // captured session, it is driving (or correctly parked in) a
        // validated, known-safe reverse-docking lane it derived
        // analytically (isDockStagingGeometryValid()) that deliberately
        // passes close to the dock's own rear housing and charging pins - a
        // REAL, permanent obstacle/contact geometry the robot must enter
        // near and touch, not avoid. dockLaneFilter withholds only the
        // ObstacleDetected EVENT from ever reaching RobotStateMachine for
        // these frames (see DockLaneObstacleFilter.hpp's own docs for the
        // exact deadlock this closes - WaitingForObstacleClear has no path
        // back out for a robot that is correctly never going to move away
        // from what it detected) - obstacle SENSING itself
        // (hardware.obstacleDetected()), ReactiveObstacleAvoidance, and
        // Safety are all completely unaffected, and this is the smallest
        // dock-specific exception the contract needs, never a global
        // suppression.
        const bool dockCapturedSticky = dockCapturedPending;
        // Edge (NotCaptured -> Captured), settled one frame further behind
        // `dockCapturedSticky` itself - see the avoidance-release block
        // below (right before avoidance.update()) for why this exact edge,
        // not merely the level, is what a stale pre-capture avoidance
        // incident needs to release on.
        const bool dockCaptureJustEnteredSticky = dockCapturedSticky && !previousDockCapturedSticky;
        previousDockCapturedSticky = dockCapturedSticky;
        const bool dockHazardSuppressionActive = dockCapturedSticky;
        dockLaneFilter.setSuppressed(dockHazardSuppressionActive);

        // Obstacle-deadlock fix, map-complete audit: the ONLY reason this
        // read exists - detecting the exact WaitingForObstacleClear ->
        // Moving edge after runtime.step() below (see the re-request block
        // at the end of this frame) - never used for anything else.
        const robot::RobotState stateBeforeStepForCompletionRearm = stateMachine.currentState();

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
        // Phase 13Y: same sticky-read shape, one frame lag - see
        // dockNeedsStage1ReplanPending's own docs above.
        const bool consumeDockNeedsStage1Replan = dockNeedsStage1ReplanPending;

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
        // Both of THESE two conditions are deterministic and physically
        // grounded, never a frame-count/wall-clock timer - but (obstacle-
        // deadlock fix, real-GUI-reproduced) both also share one unstated
        // assumption: SOME authority is currently able to move the robot
        // at all. When nothing can (e.g. Haritalama with no reachable
        // frontier target held - `navigationEnabled` false - and Safety/
        // Manual both inactive), the robot sits at exactly zero wheel
        // speed forever and NEITHER condition can ever become true - a
        // real, reproduced PERMANENT deadlock this file's own suppression
        // was supposed to be temporary, not indefinite. A THIRD,
        // unconditional, bounded-frame-count valve
        // (kAvoidanceSuppressionMaxFrames) below closes that gap - see
        // that constant's own docs for the full reasoning (mirrors
        // TableEdgeSafetyController's own kSafetyResumeMaxSuppressedFrames
        // valve exactly). Safety/Manual/RobotCollision's own hard guard
        // remain fully active and unaffected throughout - this only
        // withholds the LOCAL reactive layer's own re-arming, never any of
        // those.
        if (avoidanceSuppressedAfterBlock)
        {
            const float headingChangeSinceBlock = std::fabs(robot::visual::shortestSignedHeadingErrorDegrees(
                headingAtLastLocalRouteBlocked, world.robotPose().headingDegrees));
            // Phase 13X blocker fix (deadlock repair): the collision path
            // now requires kAvoidanceResumeCollisionStallFrames CONSECUTIVE
            // collided frames, never merely one - see that constant's own
            // docs for the ping-pong this closes.
            if (hardware.collidedLastUpdate())
            {
                ++consecutiveCollisionsWhileAvoidanceSuppressed;
            }
            else
            {
                consecutiveCollisionsWhileAvoidanceSuppressed = 0;
            }
            ++framesSuppressedSinceAvoidanceBlock;
            if (headingChangeSinceBlock >= kAvoidanceResumeHeadingChangeDegrees ||
                consecutiveCollisionsWhileAvoidanceSuppressed >= kAvoidanceResumeCollisionStallFrames ||
                framesSuppressedSinceAvoidanceBlock >= kAvoidanceSuppressionMaxFrames)
            {
                avoidanceSuppressedAfterBlock = false;
                consecutiveCollisionsWhileAvoidanceSuppressed = 0;
                framesSuppressedSinceAvoidanceBlock = 0;
            }
        }

        // `dockHazardSuppressionActive` (sticky, captured at the top of
        // this frame before runtime.step() - see that capture's own docs)
        // also gates the ordinary obstacle-triggered avoidance TRIGGER
        // here: never avoidance.update() itself, never Safety, never any
        // other frame/task - ReactiveObstacleAvoidance remains fully
        // unmodified and fully active for every other lane/task/obstacle in
        // the scene.
        const bool triggerAvoidance = avoidanceEnabled && !avoidanceSuppressedAfterBlock &&
                                       !dockHazardSuppressionActive &&
                                       stateMachine.currentState() == robot::RobotState::WaitingForObstacleClear &&
                                       hardware.obstacleDetected();
        const robot::visual::ObstacleHazardSample avoidanceHazard{
            obstacleRays.frontLeftDistance, obstacleRays.frontCenterDistance, obstacleRays.frontRightDistance};

        // Phase 13Y stale dock-attributable avoidance release (real-GUI-
        // traced): the exact frame a docking session transitions
        // NotCaptured -> Captured (`dockCaptureJustEnteredSticky`, computed
        // above before runtime.step()), an avoidance incident that started
        // BEFORE capture (triggered by the dock housing itself, back when
        // suppression did not yet cover this pose) can still be active() -
        // ReactiveObstacleAvoidance's own `triggerAvoidance` going false
        // does NOT release an already-in-progress incident (see that
        // class's own docs: "the latch, once started, is driven by
        // forwardCorridorClear and displacement alone"), so simply
        // suppressing NEW triggers (dockHazardSuppressionActive above) is
        // not sufficient on its own - a real GUI trace proved this exact
        // stale incident kept AutonomousAvoidance authority ahead of
        // Navigation for the first ~25 capture frames. Since capture only
        // EVER became true on a frame DockCaptureRegion::isDockCaptureEligible()
        // held (radius + physical safety + hazard-attribution + no
        // unrelated obstacle nearer - see that function's own docs), this
        // edge already PROVES the current incident (if any) is dock-
        // attributable; only Safety is re-checked live here, since it can
        // change independently and must always win regardless. One frame
        // of `enabled=false` is ReactiveObstacleAvoidance's own documented
        // way to force Inactive immediately (see update()'s own docs) -
        // this never resets an unrelated incident (dockCaptureJustEnteredSticky
        // can only be true when the entry predicate held, which itself
        // requires dock attribution), never changes DriveAuthority
        // ordering, and never disables avoidance beyond this single frame.
        const bool dockAttributableAvoidanceReleaseThisFrame =
            dockCaptureJustEnteredSticky && avoidance.active() && !tableEdgeSafety.active();
        const bool avoidanceEnabledThisFrame = avoidanceEnabled && !dockAttributableAvoidanceReleaseThisFrame;
        avoidance.update(avoidanceEnabledThisFrame, triggerAvoidance, forwardCorridorClear, world.robotPose(),
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
            consecutiveCollisionsWhileAvoidanceSuppressed = 0;
            framesSuppressedSinceAvoidanceBlock = 0;
        }
        localRouteBlockedPendingReplan = avoidance.localRouteBlockedThisUpdate();

        // Phase 13X blocker fix: lift safety-recovery suppression once the
        // robot has genuinely moved away from where it was blocked, or the
        // last-resort frame valve elapses - see
        // safetyRecoverySuppressedAfterBlock's own docs above.
        if (safetyRecoverySuppressedAfterBlock)
        {
            const float dxSinceBlock = world.robotPose().position.x - positionAtLastSafetyBlock.x;
            const float dzSinceBlock = world.robotPose().position.z - positionAtLastSafetyBlock.z;
            const float displacementSinceBlock = std::sqrt((dxSinceBlock * dxSinceBlock) + (dzSinceBlock * dzSinceBlock));
            ++framesSuppressedSinceSafetyBlock;
            if (displacementSinceBlock >= kSafetyResumeMinDisplacementWorldUnits ||
                framesSuppressedSinceSafetyBlock >= kSafetyResumeMaxSuppressedFrames)
            {
                safetyRecoverySuppressedAfterBlock = false;
            }
        }

        // Compute this frame's cliff-sensor readings and advance the
        // table-edge safety recovery latch (Phase 13S) - runs every frame
        // exactly like avoidance's own update() above, so it stays in
        // sync with the real robot pose regardless of manual/autonomous/
        // FSM state, UNLESS Phase 13X blocker fix suppression (above) is
        // currently withholding it - skipping the call entirely (rather
        // than calling it with some "disabled" flag) leaves it latched at
        // whatever state its own bounded recovery-stall escape just left
        // it in (Inactive - see recoveryBlockedThisUpdate()'s own docs),
        // never silently re-evaluating cliff sensors that almost
        // certainly still read unsafe from a barely-moved position.
        const robot::visual::CliffSensorReadings cliffReadings = cliffSensor.readings();
        if (!safetyRecoverySuppressedAfterBlock)
        {
            tableEdgeSafety.update(cliffReadings, world.robotPose(), world.tableSurface());
        }

        if (tableEdgeSafety.recoveryBlockedThisUpdate())
        {
            safetyRecoverySuppressedAfterBlock = true;
            positionAtLastSafetyBlock = world.robotPose().position;
            framesSuppressedSinceSafetyBlock = 0;
        }

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

        // Phase 13X quick fix (Bug B): the notice is only ever meant to
        // cover the idle gap right after a suppressed KEY_ONE press - the
        // moment ANY real task takes over (a genuine Roam session, or a
        // Return Home), it is stale and must clear, regardless of how
        // that task started.
        if (roaming || missionTask == robot::visual::MissionTask::ReturnHome)
        {
            mapAlreadyCompleteNoticeActive = false;
        }

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

        // Phase 13X human-validation fix: whenever Return Home freshly
        // becomes the active high-level task (edge-triggered - false ->
        // true THIS frame), any stale Manual override must not be allowed
        // to permanently outrank Navigation. Human GUI validation found
        // that enabling Manual mode, then requesting Return Home while it
        // was still toggled on (even with the arrow keys not currently
        // held - manualDriveMode's per-frame block below still calls
        // hardware.setManualWheelSpeeds(0, 0) every frame regardless,
        // which keeps VirtualRobotHardware's manual override LATCHED,
        // i.e. driveAuthority() stays Manual with zero wheels), left the
        // robot sitting motionless forever even though the FSM/MissionTask
        // both correctly showed ReturningHome/ReturnHome - a real
        // reproduced bug (see docs/technical-decisions.md, Phase 13X
        // human-validation fix, for the full traced 9-step reproduction).
        // driveAuthority()'s fixed priority itself (Safety > Manual >
        // AutonomousAvoidance > Navigation > Fsm) is completely UNCHANGED
        // here - Manual still legitimately outranks Navigation whenever it
        // is genuinely active; this only ever clears a STALE manual
        // request the instant a fresh Return Home task begins, using
        // VirtualRobotHardware's own existing clearManualWheelOverride()
        // API (never reaching into its private override state directly).
        // Applies identically whether Return Home was just requested by
        // the user (`2`/`R`, via MissionControlEventSource) or
        // automatically by ExplorationCompletionEventSource once no
        // reachable frontier remains - both paths land here via the exact
        // same RobotStateMachine ReturningHome transition, so no separate
        // handling is needed for the two cases. Fires ONLY on the false ->
        // true edge (never every frame while ReturnHome remains active),
        // so the user may still explicitly press `M` again afterward to
        // resume manual driving mid Return-Home - chosen over permanently
        // rejecting Manual for the rest of the Return Home session, for
        // minimal behavioral churn: the `M` key's existing toggle
        // semantics (see the KEY_M block above) are otherwise completely
        // unchanged, and this matches the codebase's own pre-existing
        // "Manual always wins the instant it is next set" precedent (e.g.
        // ManualInterruptionDuringReturnHomeResumesNavigationAfterward).
        // manualDriveMode is cleared BEFORE the manual-input block further
        // below runs this same frame (see that block's own `if
        // (manualDriveMode)` guard), so a currently-held arrow key cannot
        // immediately re-latch the override on this exact frame.
        if (returningHome && !previousReturningHomeActive)
        {
            manualDriveMode = false;
            hardware.clearManualWheelOverride();
        }
        previousReturningHomeActive = returningHome;

        const bool navigationEnabled = returningHome || (roaming && currentFrontierTarget.has_value());
        // Phase 13Y: Return Home's global-route goal is the dock STAGING
        // point, never the literal base position - see
        // DockApproachController.hpp's own class docs, and this file's own
        // WaypointNavigator/DockApproachController declaration comments
        // above, for the full two-stage "why."
        const robot::visual::Vec3 navigationGoal =
            returningHome
                ? robot::visual::computeDockStagingPoint(world.basePlatform(), world.tableSurface())
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
                                  (previousSafetyActive && !tableEdgeSafety.active()) ||
                                  consumeDockNeedsStage1Replan;

        // Phase 13X final blocker fix ("NO OSCILLATION INVARIANT"):
        // whole-session distance-to-home progress tracking - the SECOND,
        // WHOLE-TRAJECTORY signal (distinct from
        // NavigationProgressTracker's own DISPLACEMENT-based anti-spin
        // check inside WaypointNavigator, which only ever answers "has the
        // robot moved," not "is it getting closer to the GOAL") that
        // detects the exact oscillation signature this phase's own brief
        // describes: distance-to-home repeatedly alternating within a
        // band, heading kept changing, replans kept happening, yet no
        // meaningful net progress toward home over an extended window - a
        // traced, reproduced failure mode (see
        // docs/technical-decisions.md, Phase 13X final blocker fix) that
        // NEITHER localRouteBlockedThisUpdate() (TurnAway always found
        // SOME clear heading, never exhausting its full sweep) NOR
        // per-incident heuristics (proven, by regression against this
        // project's own pre-existing Return Home test suite, to
        // over-trigger on ordinary avoidance incidents that ARE part of a
        // genuinely converging route - a temporary lateral/backward
        // detour around an obstacle is completely normal and must not be
        // penalized) can distinguish from ordinary, converging navigation.
        // `bestDistanceToHomeThisSession`/`framesSinceDistanceImproved`
        // reset only when Return Home itself starts/stops (never on an
        // individual replan/incident) - genuinely whole-session bookkeeping.
        if (!returningHome)
        {
            bestDistanceToHomeThisSession = std::numeric_limits<float>::infinity();
            framesSinceDistanceImproved = 0;
        }
        // Phase 13Y: once Stage 1 has reported Arrived (handed off to
        // DockApproachController) - or is Failed/Inactive - it is no
        // longer the thing driving toward the goal, so "distance to base
        // center not improving" is expected and meaningless here, never a
        // stuck symptom Stage 1 itself needs to react to. Without this
        // guard, Stage 2's own legitimate slow/rotational precision
        // maneuvering (AlignForReverse's in-place turning, the staging-
        // point creep, brief ReverseApproach/AlignForReverse cycling) reads
        // as "stuck" by this whole-session distance metric, repeatedly
        // blacklisting the very staging-point cell Stage 2 needs the next
        // time something legitimately forces a Stage 1 replan (e.g. a
        // large-displacement re-entry) - corrupting that replan into
        // snapping to a much worse, obstacle-adjacent cell (a real,
        // integration-test-traced defect this guard closes).
        else if (previousNavOutput.state != robot::visual::WaypointNavigatorState::Following)
        {
            framesSinceDistanceImproved = 0;
        }
        else
        {
            const float dxHome = world.robotPose().position.x - world.basePlatform().position.x;
            const float dzHome = world.robotPose().position.z - world.basePlatform().position.z;
            const float distanceToHomeNow = std::sqrt((dxHome * dxHome) + (dzHome * dzHome));
            if (distanceToHomeNow < bestDistanceToHomeThisSession - kOscillationProgressEpsilonWorldUnits)
            {
                bestDistanceToHomeThisSession = distanceToHomeNow;
                framesSinceDistanceImproved = 0;
            }
            else
            {
                ++framesSinceDistanceImproved;
            }
        }

        // Phase 13X blocker fix (deadlock repair): maintain
        // returnHomeBlockedCells - see its own docs above. Scoped to
        // returningHome only (frontier navigation keeps its own separate,
        // already-validated target-level blacklist untouched).
        //
        // Phase 13X final blocker fix (BLOCKED-CELL REPLAN AUDIT): TWO
        // triggers now populate this list, both proven necessary by traced
        // reproductions (see docs/technical-decisions.md, Phase 13X final
        // blocker fix) - never guessed in advance:
        //   1) avoidance.localRouteBlockedThisUpdate() - a full-360-
        //      degree-sweep-exhausted incident is already, on its own, a
        //      complete proof the exact waypoint cell is not passable -
        //      blacklisted unconditionally, exactly as the original
        //      deadlock-repair fix already did.
        //   2) The oscillation window above expiring
        //      (framesSinceDistanceImproved reaching
        //      kOscillationStuckWindowFrames - an edge, consumed exactly
        //      once per window via the reset below) - the CURRENT
        //      waypoint cell is what the route has been repeatedly
        //      steering through this whole non-converging window, so it
        //      is the correct target to exclude and force A* onto
        //      genuinely different route geometry, exactly like trigger
        //      1 above (same helper, same neighborhood/cap).
        // Excludes a small kLocalReplanExclusionRadiusCells neighborhood
        // (not merely the one exact cell - see that constant's own docs),
        // bounded overall by kMaxReturnHomeBlockedCells (FIFO eviction).
        if (!returningHome)
        {
            returnHomeBlockedCells.clear();
        }
        else if (avoidance.localRouteBlockedThisUpdate() && !previousNavOutput.route.empty() &&
                 previousNavOutput.currentWaypointIndex < previousNavOutput.route.size())
        {
            int blockedCol = -1;
            int blockedRow = -1;
            if (explorationMap.worldToCell(previousNavOutput.route[previousNavOutput.currentWaypointIndex],
                                            blockedCol, blockedRow))
            {
                addBlockedCellNeighborhood(returnHomeBlockedCells, blockedCol, blockedRow,
                                            kLocalReplanExclusionRadiusCells, kMaxReturnHomeBlockedCells);
            }
        }
        else if (framesSinceDistanceImproved >= kOscillationStuckWindowFrames && !previousNavOutput.route.empty() &&
                 previousNavOutput.currentWaypointIndex < previousNavOutput.route.size())
        {
            int col = -1;
            int row = -1;
            if (explorationMap.worldToCell(previousNavOutput.route[previousNavOutput.currentWaypointIndex], col,
                                            row))
            {
                addBlockedCellNeighborhood(returnHomeBlockedCells, col, row, kLocalReplanExclusionRadiusCells,
                                            kMaxReturnHomeBlockedCells);
            }
            // Edge-consume: starts a fresh window immediately, so a
            // route that is STILL not converging escalates again after
            // another full window, rather than blacklisting every single
            // frame for as long as the condition remains true.
            framesSinceDistanceImproved = 0;
        }

        const robot::visual::WaypointNavigatorOutput navOutput = mapNavigator.update(
            world.robotPose(), explorationMap, navigationGoal, navigationEnabled, forceReplan,
            returningHome ? returnHomeBlockedCells : std::vector<robot::visual::GridCoord>{});
        previousNavOutput = navOutput;

        // Phase 13Y: Stage 2 - only ever enabled while Return Home is the
        // active task; `arrivedAtStagingPoint` is Stage 1's own Arrived
        // report (WaypointNavigator's goal is computeDockStagingPoint(),
        // never the literal base position - see this file's own docs
        // above) OR'd with this frame's own FRESH (post-runtime.step(),
        // unlike the sticky `dockCaptureEligibleSticky` gate above)
        // DockCaptureRegion::isDockCaptureEligible() check - see
        // DockApproachController::kDockStagingCaptureRadius's own docs for
        // why relying on WaypointNavigatorState::Arrived exclusively is not
        // robust (a real GUI trace proved ReactiveObstacleAvoidance can
        // repeatedly win DriveAuthority over Navigation right next to the
        // dock and leave WaypointNavigator Failed or perpetually
        // Following). DockApproachController never recomputes either fact
        // itself, so it can never disagree with the caller about whether
        // the handoff happened. See this class's own header docs for the
        // full NavigateToStagingPoint -> AlignForReverse -> ReverseApproach
        // -> Docked precision reverse-docking state machine.
        const bool dockCaptureEligibleFresh =
            returningHome && robot::visual::isDockCaptureEligible(world.robotPose(), world.basePlatform(),
                                                                    world.tableSurface(), world.obstacles(),
                                                                    !cliffReadings.anyCliff());
        const robot::visual::DockApproachOutput dockOutput = dockApproach.update(
            world.robotPose(), world.basePlatform(), world.tableSurface(), returningHome,
            returningHome && (navOutput.state == robot::visual::WaypointNavigatorState::Arrived ||
                               dockCaptureEligibleFresh));
        // Captured for NEXT frame's consumeDockNeedsStage1Replan sticky
        // read (see that capture's own docs, top of this frame).
        dockNeedsStage1ReplanPending = dockOutput.needsStage1Replan;
        // Captured for NEXT frame's dockCapturedSticky/dockCaptureJustEnteredSticky
        // sticky reads (see those capture's own docs, top of this frame).
        dockCapturedPending = dockOutput.captured;

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
        //
        // Phase 13X final-approach fix: DockApproachController's own
        // `driving` (Aligning/FinalApproach) takes over the SAME
        // Navigation-tier override the instant Stage 1 hands off - never a
        // new/separate DriveAuthority tier (Dock approach uses Navigation
        // authority, per this fix's own brief). At most one of
        // navOutput/dockOutput is ever actually driving at a time by
        // construction (WaypointNavigator reports zero speeds once
        // Arrived; DockApproachController reports zero until Stage 1
        // arrives), so preferring dockOutput whenever it is driving is
        // never a real conflict.
        const bool navigationDriving =
            dockOutput.driving || navOutput.state == robot::visual::WaypointNavigatorState::Following;
        if (dockOutput.driving)
        {
            hardware.setNavigationWheelSpeeds(dockOutput.wheelSpeeds.left, dockOutput.wheelSpeeds.right);
        }
        else if (navigationDriving)
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
        // Phase 13X human-validation fix: completion/coverage display
        // telemetry - see Renderer3D.hpp's own docs on these three fields.
        // `completionSignal.complete`/`roaming` are the exact same values
        // already driving the frontier-selection block above (never a
        // second, separately-derived copy), and
        // explorationMap.exploredPercentage() is the same truthful raw
        // getter the HARİTA panel already reads directly - this is purely
        // a second copy for the Ayrıntılı HUD, which does not receive
        // ExplorationMap itself.
        // Phase 13X quick fix (Bug B): also true while
        // mapAlreadyCompleteNoticeActive is latched (KEY_ONE found the map
        // already complete and withheld StartMission - see that handler
        // above) - reuses the SAME Tamamlandı/%100 display this fix
        // already has, rather than a second notice mechanism, and never
        // touches completionSignal itself (still exactly the real
        // Haritalama-session completion fact - no event-source edge risk).
        telemetry.logicalExplorationComplete = completionSignal.complete || mapAlreadyCompleteNoticeActive;
        telemetry.explorationActive = roaming;
        telemetry.rawExploredPercentage = explorationMap.exploredPercentage();
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
        // Phase 13Y: once Stage 2 has anything meaningful to show
        // (AlignForReverse/ReverseApproach/Docked), its own state text
        // replaces WaypointNavigator's - otherwise the HUD would
        // misleadingly keep reading "Ulaşıldı" (Arrived) for the entire
        // reverse-docking maneuver, the instant Stage 1 merely reaches the
        // staging point.
        telemetry.homeNavigationStateText =
            (returningHome && dockOutput.state != robot::visual::DockApproachState::Inactive &&
             dockOutput.state != robot::visual::DockApproachState::NavigateToStagingPoint)
                ? robot::visual::turkishText(dockOutput.state)
                : robot::visual::turkishText(navOutput.state);
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

        // Phase 13Z: New Map (`N`) notice telemetry - at most one of the
        // three is ever non-empty (see VisualTelemetry's own docs);
        // Renderer3D never decides which applies, it only ever draws
        // whichever field is currently non-empty.
        if (mapResetController.confirmationPending())
        {
            telemetry.mapResetConfirmLine1 = "Mevcut harita silinecek.";
            telemetry.mapResetConfirmLine2 = "Onaylamak için N'ye tekrar basın.";
        }
        else if (mapResetRejectedNoticeSecondsRemaining > 0.0F)
        {
            telemetry.mapResetRejectedLine = "Önce görevi durdurun (3)";
        }
        else if (mapResetSuccessNoticeSecondsRemaining > 0.0F)
        {
            telemetry.mapResetSuccessLine = "Yeni harita oluşturuldu";
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

        // Obstacle-deadlock fix, map-complete audit: RobotStateMachine's
        // WaitingForObstacleClear case has no handler for
        // ReturnHomeRequested (see RobotStateMachine.cpp -
        // InvalidTransition, the event is simply dropped -
        // RobotRuntime::step() never requeues a rejected event), and
        // ExplorationCompletionEventSource only ever fires
        // ReturnHomeRequested on completionSignal.complete's ONE
        // false -> true edge for a whole session (by design). Worse,
        // CompositePollingEventSource short-circuits (see that class's own
        // pollEvent()): whenever a higher-priority source - here,
        // hardwareEventSource/dockLaneFilter - has an event to report on a
        // given frame, lower-priority sources (including
        // ExplorationCompletionEventSource) are never even polled that
        // frame, so their own edge-tracking is simply frozen for the
        // frame, not lost - EXCEPT for the one case that matters here: if
        // exploration first reaches logical completion while the robot is
        // genuinely stuck in WaitingForObstacleClear (no competing
        // hardware event that frame), the completion source IS polled,
        // its edge IS genuinely consumed, and RobotStateMachine genuinely
        // rejects it - permanently spent (`wasComplete_` latches true
        // regardless of acceptance) - exactly the real-reported
        // "Map=100%/Roam/WaitingForObstacleClear" screenshot state. The
        // robot would then resume plain Roam wandering forever once the
        // obstacle clears, never auto-returning home despite being
        // logically Complete.
        //
        // Fix: detect the exact WaitingForObstacleClear -> Moving edge
        // (the obstacle that interrupted Roam has genuinely cleared,
        // resuming the SAME Roam mission - resumeState_ == Moving, never a
        // Return-Home-interrupted pause, which resumes into ReturningHome
        // instead and is correctly excluded here) while completion is
        // still true, and explicitly call
        // missionControl.requestReturnHome() - the SAME production API
        // `2`/`R` already use, deliberately NOT another attempt at
        // re-triggering completionSignal.complete's own edge (an earlier
        // version of this fix tried exactly that and was proven, via this
        // phase's own regression test, to silently fail: forcing the
        // signal false then true again is ITSELF subject to the same
        // short-circuiting - the low-priority completion source never
        // actually observes the intermediate `false` value on the exact
        // frame a competing ObstacleCleared event is also present, so no
        // genuine edge is ever seen). MissionControlEventSource is the
        // HIGHEST-priority source in the composite chain, so a request
        // queued here is guaranteed delivery on the very next
        // runtime.step() - never short-circuited by anything.
        // completionSignal.complete/ExplorationCompletionEventSource
        // themselves are completely untouched by this fix.
        if (stateBeforeStepForCompletionRearm == robot::RobotState::WaitingForObstacleClear &&
            stateMachine.currentState() == robot::RobotState::Moving && completionSignal.complete)
        {
            missionControl.requestReturnHome();
        }
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
