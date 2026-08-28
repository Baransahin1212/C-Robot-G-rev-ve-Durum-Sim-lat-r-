#pragma once

#include <optional>
#include <string>
#include <vector>

#include "robot/visual/CoverageTrail.hpp"
#include "robot/visual/ExplorationCompletionEventSource.hpp"
#include "robot/visual/ExplorationMap.hpp"
#include "robot/visual/FrontierExplorer.hpp"
#include "robot/visual/GridPathPlanner.hpp"
#include "robot/visual/WaypointNavigator.hpp"

namespace robot::visual
{

// Phase 13Z: every piece of state belonging to ONE exploration-mapping
// SESSION that New Map (`N`) must reset so a fresh session can begin
// immediately, with no application restart required - see
// resetExplorationSession() below for exactly what happens to each field.
// A plain bundle of references to main3d.cpp's own already-owned local
// variables, not an owning struct - this stays a thin reset-orchestration
// helper, never a second place any of this state actually lives (mirrors
// this codebase's existing "pass many already-owned pieces by reference"
// style, e.g. WaypointNavigator::update()'s own parameter list).
//
// Deliberately excludes robot pose/heading/battery/hardware/
// RobotStateMachine/DifferentialDrive/manual-control configuration/table
// geometry/desk objects/dock geometry/charging contacts/DriveAuthority
// priority/obstacle-avoidance/Safety state - this is a MAP SESSION reset,
// never an application factory reset (this phase's own brief).
struct ExplorationSessionState
{
    ExplorationMap& map;
    CoverageTrail& trail;
    WaypointNavigator& navigator;
    ExplorationCompletionSignal& completionSignal;

    std::optional<FrontierTarget>& currentFrontierTarget;
    std::vector<GridCoord>& frontierBlacklist;
    int& framesSinceLastFrontierAttempt;
    int& consecutiveNoTargetFound;

    std::vector<GridCoord>& returnHomeBlockedCells;
    WaypointNavigatorOutput& previousNavOutput;
    float& bestDistanceToHomeThisSession;
    int& framesSinceDistanceImproved;

    bool& mapAlreadyCompleteNoticeActive;
    float& mapSaveTimer;
    bool& mapWasLoadedAtStartup;
};

// Deletes the persisted exploration-map file at `mapStoragePath` (a no-op
// success if `mapPersistenceAvailable` is false, or if no file exists
// there - see ExplorationMapStorage::remove()'s own docs), then resets
// every field in `session` back to a fresh, all-Unknown, empty-session
// state - the exact same starting point a brand-new application launch
// with no saved map would have.
//
// `session.map`'s own object identity (address) is never replaced - only
// its cell CONTENTS, via ExplorationMap::setCells() (the one sanctioned
// way an Occupied cell may become non-Occupied again - see that method's
// own docs) - so existing references into it (FrontierExplorer's own
// `map_` member, held since construction in main3d.cpp) stay valid, never
// dangling. The fresh grid's width/height/resolution are derived from
// `session.map.bounds()` itself (never hardcoded), so this stays correct
// for whatever the current TableSurface actually is.
void resetExplorationSession(const std::string& mapStoragePath, bool mapPersistenceAvailable,
                              ExplorationSessionState& session);

} // namespace robot::visual
