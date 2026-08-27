#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "robot/visual/ExplorationMap.hpp"
#include "robot/visual/GridPathPlanner.hpp"
#include "robot/visual/HomeNavigator.hpp"
#include "robot/visual/NavigationProgressTracker.hpp"

namespace robot::visual
{

// V1 map-aware waypoint-following states (Phase 13X) - NOT
// RobotStateMachine states, and never exposed to/consumed by
// RobotStateMachine/RobotController in any way, exactly like
// HomeNavigationState/TableEdgeSafetyController::RecoveryState's own
// command-vs-effective-actuator distinction.
enum class WaypointNavigatorState
{
    Inactive,
    Following,
    Arrived,

    // A (re)plan attempt found no path to the current goal - the caller
    // (main3d.cpp's exploration loop) is expected to react (e.g. blacklist
    // the current frontier target and pick another), never to treat this
    // as a crash/hang condition. See WaypointNavigator's own class docs.
    Failed
};

constexpr std::string_view toString(WaypointNavigatorState value) noexcept
{
    switch (value)
    {
        case WaypointNavigatorState::Inactive: return "Inactive";
        case WaypointNavigatorState::Following: return "Following";
        case WaypointNavigatorState::Arrived: return "Arrived";
        case WaypointNavigatorState::Failed: return "Failed";
    }
    return "Unknown";
}

// Everything one update() call reports.
struct WaypointNavigatorOutput
{
    WheelSpeeds wheelSpeeds;
    WaypointNavigatorState state = WaypointNavigatorState::Inactive;

    // The current SIMPLIFIED planned route (world-space) - empty while
    // Inactive/Failed. Exposed for rendering (Renderer3D draws it, never
    // computes it - see docs/technical-decisions.md) and tests.
    std::vector<Vec3> route;
    std::size_t currentWaypointIndex = 0;
    bool arrived = false;
    bool failed = false;
};

// Deterministic, raylib-free map-aware waypoint-following orchestrator
// (Phase 13X) - the layer GridPathPlanner's global route sits above
// HomeNavigator's own local point-to-point steering:
//
//   RobotPose + ExplorationMap + goal -> GridPathPlanner -> simplified
//   waypoints -> WaypointNavigator keeps a current-waypoint index and
//   asks an internally-owned HomeNavigator to steer toward it -> wheel
//   speeds.
//
// This is what fixes the human-observed Return Home problem: HomeNavigator
// itself is UNCHANGED (still exists, still does exactly the local
// Aligning/Driving/Arrived point-to-point job it always has - see
// HomeNavigator.hpp, never deleted per this phase's own brief) - it is now
// only ever asked to steer toward the CURRENT WAYPOINT of a map-aware
// route, never blindly straight at a possibly-obstructed final goal, so it
// can no longer repeatedly re-aim back through a known obstacle the moment
// a local avoidance maneuver rotates the corridor momentarily clear (the
// old bug's exact mechanism - see docs/technical-decisions.md, Phase 13X).
//
// Owns a private HomeNavigator instance purely for per-waypoint local
// steering - never a second, duplicated steering implementation - and a
// private NavigationProgressTracker for the anti-360-spin guarantee (see
// that class's own docs). This class decides WHEN to (re)plan and which
// waypoint is current; HomeNavigator decides HOW to steer toward one fixed
// point; NavigationProgressTracker decides whether the current attempt is
// making genuine progress.
//
// REPLANNING (a fresh GridPathPlanner::planPath() call, constructed fresh
// from the CURRENT `map` passed into update() - never a stale cached
// planner) happens when:
//   - this is a fresh enable (state was Inactive) - "Return Home begins"
//   - the caller passes forceReplan=true (main3d.cpp sets this on the
//     frame avoidance/Safety just released, since the robot may now sit
//     meaningfully off the previously-planned route - see class docs on
//     "replanning + avoidance")
//   - `goalWorld` changed materially since the last plan (a new frontier
//     target was chosen)
//   - the current route's remaining cells include one the map now reports
//     Occupied ("path becomes invalid" / newly discovered obstacle)
//   - NavigationProgressTracker reports stuck (the anti-360-spin fix)
//   - the previous replan attempt itself reported Failed AND, since that
//     attempt, either the map's content actually changed
//     (ExplorationMap::revision(), a real GUI-traced fix - see this
//     class's own .cpp docs) or the robot's pose moved meaningfully (e.g.
//     Manual/Safety displaced it) - never merely "still Failed," which
//     would re-run A* every single frame for as long as nothing about the
//     situation had changed at all
// - i.e. event/state/dirty-driven, never a per-frame unconditional re-plan
// (this phase's own brief: "Do NOT re-run A* every frame").
class WaypointNavigator
{
public:
    // `map` is read fresh every call - always the CURRENT ExplorationMap,
    // never cached across calls. `goalWorld` is the caller's current
    // desired destination (world.basePlatform().position for Return Home,
    // or the current FrontierTarget's worldPosition for exploration -
    // main3d.cpp decides which, this class has no notion of Return
    // Home/exploration itself, only "a goal"). `enabled` false immediately
    // resets to Inactive, mirroring HomeNavigator's own enabled contract
    // exactly. `forceReplan` true forces a fresh plan attempt this call
    // regardless of every other condition above. `extraBlockedCells`
    // (Phase 13X blocker fix, deadlock repair) is forwarded verbatim to
    // GridPathPlanner's own identically-named constructor parameter for
    // every replan this call performs - see that parameter's own docs
    // (GridPathPlanner.hpp) for why it exists; empty by default, so every
    // pre-existing caller is completely unaffected.
    WaypointNavigatorOutput update(const RobotPose& pose, const ExplorationMap& map, const Vec3& goalWorld,
                                    bool enabled, bool forceReplan,
                                    const std::vector<GridCoord>& extraBlockedCells = {}) noexcept;

    // Explicit reset to Inactive - equivalent to the next update() call
    // with enabled=false, exposed for callers/tests that want a clean
    // state without waiting a frame (mirrors HomeNavigator::reset()).
    void reset() noexcept;

    WaypointNavigatorState state() const noexcept;
    const std::vector<Vec3>& currentRoute() const noexcept;

private:
    void planRoute(const RobotPose& pose, const ExplorationMap& map, const Vec3& goalWorld,
                    const std::vector<GridCoord>& extraBlockedCells);
    bool routeStillClear(const ExplorationMap& map) const;

    HomeNavigator localSteering_;
    NavigationProgressTracker progressTracker_;
    WaypointNavigatorState state_ = WaypointNavigatorState::Inactive;
    std::vector<Vec3> route_;
    std::size_t waypointIndex_ = 0;
    Vec3 lastGoal_{};
    bool hasLastGoal_ = false;

    // Phase 13X blocker fix (Failed-state retry audit): the map
    // revision/pose recorded at the moment the CURRENT Failed attempt was
    // made - see this class's own .cpp docs for why a Failed retry is
    // gated on these rather than retried unconditionally every call.
    std::uint64_t mapRevisionAtLastFailedAttempt_ = 0;
    Vec3 poseAtLastFailedAttempt_{};
    bool hasFailedAttemptContext_ = false;
};

} // namespace robot::visual
