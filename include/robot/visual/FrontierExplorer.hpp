#pragma once

#include <cstddef>
#include <string_view>
#include <vector>

#include "robot/visual/ExplorationMap.hpp"
#include "robot/visual/GridPathPlanner.hpp"

namespace robot::visual
{

// Phase 13X: minimum number of adjacent frontier cells a cluster must
// contain to be considered a real exploration target rather than isolated
// single/double-cell sensor-edge noise (this phase's own brief - "avoid
// selecting isolated one-cell noise where practical"). Small, named
// threshold, not a magic literal.
inline constexpr std::size_t kMinimumFrontierClusterSize = 3;

// Phase 13X: how much a frontier cluster's SIZE discounts its path-cost
// score in selectTarget() below - see that method's own docs for the
// scoring formula. A modest weight: a materially larger cluster is worth
// a modest extra detour, but this must never dominate reachability/
// distance (a huge cluster on the far side of the desk should still
// usually lose to a small-but-close one).
inline constexpr float kInformationGainWeight = 0.05F;

// Phase 13X: mapping-completion status - see FrontierExplorer::completion()
// below and docs/technical-decisions.md ("mapping completion semantics")
// for the full reasoning behind distinguishing NoReachableFrontier from
// Complete even though both are terminal for the exploration loop (both
// mean "stop seeking, nothing left this session can usefully reach").
enum class ExplorationCompletion
{
    // At least one frontier cell exists (reachable or not).
    Exploring,

    // No Free cell anywhere on the map borders an Unknown cell - genuinely
    // nothing left to seek, for the region the robot has been able to
    // observe.
    Complete,

    // At least one frontier cell exists, but none are reachable (through
    // the current occupancy grid, at planning clearance) from the queried
    // position - e.g. every remaining frontier sits behind geometry the
    // robot cannot actually reach with its own footprint.
    NoReachableFrontier
};

constexpr std::string_view toString(ExplorationCompletion value) noexcept
{
    switch (value)
    {
        case ExplorationCompletion::Exploring: return "Exploring";
        case ExplorationCompletion::Complete: return "Complete";
        case ExplorationCompletion::NoReachableFrontier: return "NoReachableFrontier";
    }
    return "Unknown";
}

// One selectTarget() result. `found` false means no reachable, non-
// blacklisted frontier cluster exists right now - the caller should treat
// this as "nothing to do this attempt" (see ExplorationCompletion for the
// caller-facing distinction between temporarily nothing vs. genuinely
// complete).
struct FrontierTarget
{
    bool found = false;
    Vec3 worldPosition;
    GridCoord cell{};
    std::vector<Vec3> pathWaypoints;
    float pathCost = 0.0F;
    std::size_t clusterSize = 0;
};

// Deterministic, raylib-free frontier-based exploration target selector
// (Phase 13X) - occupancy-grid FRONTIER EXPLORATION, explicitly NOT SLAM:
// it only ever reads ExplorationMap's already-computed cell states plus a
// caller-supplied authoritative world position; it performs no scan
// matching, loop closure, or particle localization of any kind.
//
// May read ONLY ExplorationMap (directly, and indirectly through the
// GridPathPlanner it constructs internally for reachability/cost scoring)
// - never VirtualWorld::obstacles() or DeskObjectType, preserving the same
// perception boundary GridPathPlanner itself establishes (see that
// class's own docs).
//
// A FRONTIER CELL is a planner-traversable Free cell that is 4-connected-
// adjacent (N/S/E/W only - deliberately not 8-connected for this
// adjacency check, since a merely diagonal Unknown neighbor is a much
// weaker "boundary of the known" signal than a direct edge-adjacent one,
// and 4-connected keeps frontier detection cheap and unambiguous) to at
// least one Unknown cell. Frontier cells are grouped into CLUSTERS by
// 8-connected adjacency to EACH OTHER (a flood-fill over the frontier-cell
// set only), and clusters smaller than kMinimumFrontierClusterSize are
// discarded.
class FrontierExplorer
{
public:
    // `map` must outlive this object - the same non-owning-reference
    // pattern GridPathPlanner itself uses. Unlike GridPathPlanner, this
    // class does NOT cache a planner/traversability snapshot at
    // construction - every public method builds a fresh GridPathPlanner
    // internally from the CURRENT state of `map` (this phase's own brief:
    // "do NOT re-run A* every frame" governs how OFTEN a caller invokes
    // these methods, not whether this class itself may hold a stale,
    // construction-time-only view of a map that keeps changing underneath
    // it every frame via ExplorationMapper).
    explicit FrontierExplorer(const ExplorationMap& map) noexcept;

    // Detects every frontier cluster, scores each reachable one by
    //   score = pathCost - kInformationGainWeight * clusterSize
    // (lower is better - a nearby, large cluster beats a distant, tiny
    // one), and returns the best-scoring cluster whose representative
    // cell is not present in `blacklist`. A cluster's representative
    // candidate cell is its member CLOSEST (straight-line) to
    // `robotWorldPosition` - a single GridPathPlanner::planPath() call per
    // cluster then proves reachability and supplies the real path cost
    // (never a plain Euclidean-distance target choice - see class docs).
    // Returns FrontierTarget{found=false} if no frontier cell exists at
    // all, or none (after removing blacklisted clusters) are reachable.
    FrontierTarget selectTarget(const Vec3& robotWorldPosition, const std::vector<GridCoord>& blacklist = {}) const;

    // See ExplorationCompletion's own docs above. Equivalent to (but
    // cheaper than, for the Complete case) calling selectTarget() with an
    // empty blacklist and inspecting `found`.
    ExplorationCompletion completion(const Vec3& robotWorldPosition) const;

    // Every currently-detected frontier cell (see class docs) - a small,
    // genuinely useful diagnostic surface (not merely for tests: a future
    // caller could visualize raw frontier cells directly), never filtered
    // by cluster size or reachability. selectTarget() reuses the exact
    // same detection internally.
    std::vector<GridCoord> frontierCells() const;

    // Frontier cells grouped into 8-connected clusters (see class docs),
    // in deterministic detection order - NOT filtered by
    // kMinimumFrontierClusterSize (selectTarget() applies that filter
    // itself). Exposed for the same reason as frontierCells() above.
    std::vector<std::vector<GridCoord>> frontierClusters() const;

private:
    std::vector<GridCoord> detectFrontierCells(const GridPathPlanner& planner) const;
    std::vector<std::vector<GridCoord>> clusterFrontierCells(const std::vector<GridCoord>& frontierCells) const;

    const ExplorationMap& map_;
};

} // namespace robot::visual
