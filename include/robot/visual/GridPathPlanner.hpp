#pragma once

#include <string_view>
#include <vector>

#include "robot/visual/ExplorationMap.hpp"
#include "robot/visual/NavigationClearance.hpp"
#include "robot/visual/RobotCollision.hpp"

namespace robot::visual
{

// Phase 13X final blocker fix: small named planning-only extra safety
// margin ON TOP OF RobotCollision::kRobotCollisionRadius - a global route
// planned right at the bare collision radius would route the robot along
// paths a single frame of sensor/localization noise could turn into a
// collision; this margin is the same "half-diagonal plus a little"
// philosophy kRobotCollisionRadius itself already uses, applied a second
// time at the planning layer.
//
// VALUE (Phase 13X final blocker fix): previously a bare 0.05F literal,
// chosen independently of ForwardClearanceProbe/ReactiveObstacleAvoidance's
// own local reactive-hazard margin (0.08F) - a real, traced defect (see
// docs/technical-decisions.md, Phase 13X final blocker fix, and
// NavigationClearance.hpp's own top-level docs): a globally planned
// straight segment could sit outside GridPathPlanner's own (too-small)
// planning clearance while still sitting INSIDE the local reactive layer's
// hazard envelope, so the robot's own reactive avoidance kept rejecting a
// route GridPathPlanner considered clear, replanning into a materially
// identical route, forever. Now derived as
// NavigationClearance::kPlanningRadius - kRobotCollisionRadius, i.e.
// kRobotCollisionRadius + kPlanningSafetyMargin always exactly equals
// NavigationClearance::kPlanningRadius (max(physical, local-hazard) +
// a small additional planning-only margin) - see that header for the full
// derivation. Still small relative to a grid cell (0.12F) - deliberately
// not large enough to make an already-narrow real gap (e.g. between two
// desk objects) unplannable.
inline const float kPlanningSafetyMargin = NavigationClearance::kPlanningRadius - kRobotCollisionRadius;

// One grid cell coordinate - (column, row), matching ExplorationMap's own
// worldToCell()/cellAt() convention exactly.
struct GridCoord
{
    int col = 0;
    int row = 0;
};

constexpr bool operator==(const GridCoord& lhs, const GridCoord& rhs) noexcept
{
    return lhs.col == rhs.col && lhs.row == rhs.row;
}

constexpr bool operator!=(const GridCoord& lhs, const GridCoord& rhs) noexcept
{
    return !(lhs == rhs);
}

// Everything one planPath() call reports. `waypoints` is the SIMPLIFIED
// world-space route (see simplifyPath() below) - start first, goal (or a
// point very near it - see GridPathPlanner's own docs on goal snapping)
// last. `cellPath` is the full, unsimplified grid path, exposed for tests
// and diagnostics. `pathCostWorldUnits` is the total Euclidean length of
// `cellPath` (cell-center to cell-center), the metric FrontierExplorer
// uses to score candidate frontiers.
struct PathPlanResult
{
    bool success = false;
    std::vector<Vec3> waypoints;
    std::vector<GridCoord> cellPath;
    float pathCostWorldUnits = 0.0F;
};

// Deterministic, raylib-free A* grid path planner (Phase 13X) - global
// route planning over an ExplorationMap's occupancy grid, for both
// map-aware Return Home and frontier navigation. This is occupancy-grid
// PATH PLANNING, explicitly NOT SLAM: it consumes ExplorationMap's already-
// computed cell states plus the caller's own authoritative RobotPose; it
// performs no scan matching, loop closure, or particle localization of any
// kind, and RobotPose itself is never derived from this class.
//
// May read ONLY ExplorationMap - never VirtualWorld::obstacles() or
// DeskObjectType (this header never includes VirtualWorld.hpp beyond the
// plain Vec3/TableSurface data ExplorationMap.hpp itself already exposes),
// preserving the same perception boundary ExplorationMapper already
// established: a planning component can only ever know what the robot has
// actually sensed, never ground truth. See docs/technical-decisions.md
// (Phase 13X) for the full rationale.
//
// UNKNOWN CELL POLICY: Unknown cells are NEVER traversable for planning -
// isTraversable() only ever accepts Free cells. This is deliberate for
// both of this class's callers: Return Home must never plan a route
// through space the robot has not actually observed, and frontier
// navigation only ever plans TO a Free cell that borders Unknown (see
// FrontierExplorer.hpp) - never THROUGH Unknown itself.
//
// CLEARANCE/INFLATION: a cell is only traversable if its center is at
// least (RobotCollision::kRobotCollisionRadius + kPlanningSafetyMargin)
// away from every Occupied cell AND from every TableSurface edge (global
// planning must never intentionally route somewhere Safety would
// immediately have to rescue the robot from - see
// docs/technical-decisions.md). This inflation is purely a PLANNING-time
// traversability predicate; it never mutates ExplorationMap's own raw
// cell data.
//
// CONNECTIVITY: 8-connected, with corner-cutting prevented - a diagonal
// step between two cells is only allowed when BOTH orthogonal cells it
// would otherwise cut across are also traversable (chosen over 4-connected
// because it produces materially shorter/more natural routes on this
// project's grid resolution, at negligible extra cost on a map this
// small - see docs/technical-decisions.md for the full 4- vs 8-connected
// discussion). Tie-breaking is fully deterministic: ties in f-score are
// broken by ascending row-major cell index, never by pointer/hash/
// insertion-timing order, so the same map+start+goal always produces the
// exact same path.
//
// GOAL SNAPPING: if the requested goal cell itself is not traversable
// (e.g. a goal point sitting inside another obstacle's planning-clearance
// margin, such as the charging dock's own rear-housing obstacle sitting
// close to BasePlatform's center - see docs/technical-decisions.md,
// "dock goal snapping"), the search target is instead the nearest
// traversable cell to it (bounded local search - see the .cpp), and the
// literal requested goal point is appended as one final waypoint after the
// simplified route, so the caller's local steering (HomeNavigator, inside
// WaypointNavigator) still closes the exact last few centimeters via its
// own existing arrival-radius logic, exactly as it already does today.
// This never happens silently across a genuinely blocked/disconnected
// region - the bounded local search simply fails, and so does planPath(),
// exactly like any other unreachable goal.
class GridPathPlanner
{
public:
    // `map` must outlive this object - the same non-owning-reference
    // pattern every other visual-simulation component in this codebase
    // uses. Traversability is precomputed once here from `map`'s state AT
    // CONSTRUCTION TIME (see class docs on why this is not a live query) -
    // callers that need a fresh plan after the map has changed must
    // construct a new GridPathPlanner, never reuse a stale one across
    // multiple planPath() calls spanning map updates.
    //
    // `extraBlockedCells` (Phase 13X blocker fix, deadlock repair):
    // additional cells treated as NOT traversable, on top of whatever
    // `map` itself already implies - never mutates `map`. Empty by default
    // (every pre-existing caller/behavior is completely unchanged). Exists
    // for exactly one caller-facing reason: ExplorationMap's own Occupied/
    // Free classification is honest sensor-derived data, but it can never
    // capture EVERY real-world obstruction a live perception+collision
    // system encounters (e.g. geometry a 3-ray sensor array's cone simply
    // never crosses, even though the robot's own circular body collides
    // with it) - when ReactiveObstacleAvoidance's bounded TurnAway proves,
    // via an actual full local sweep, that a specific cell the global
    // route keeps proposing is not really passable, the caller (main3d.cpp)
    // may pass that cell here so THIS replan (and only this one - never
    // persisted inside GridPathPlanner/ExplorationMap itself) routes
    // around it instead of deterministically recomputing the exact same
    // doomed route from an unchanged map. This is an additional
    // traversability CONSTRAINT applied on top of the same deterministic
    // A* search - never a different algorithm, never a search over
    // multiple candidate blacklists.
    explicit GridPathPlanner(const ExplorationMap& map, const std::vector<GridCoord>& extraBlockedCells = {}) noexcept;

    // True if (col, row) is inside the grid, Free, and at least
    // (RobotCollision::kRobotCollisionRadius + kPlanningSafetyMargin) away
    // from every Occupied cell and every TableSurface edge - see class
    // docs above. O(1) - precomputed at construction.
    bool isTraversable(int col, int row) const noexcept;

    // Phase 13X final blocker fix ("PATH SIMPLIFICATION AUDIT"): the same
    // clearance predicate isTraversable()'s own per-cell precomputation is
    // built from, but evaluated at an arbitrary CONTINUOUS world point
    // rather than only at a cell's own center - i.e. "is `point` itself
    // (not merely the grid cell it happens to fall in) at least the
    // planning clearance away from every occupied cell's square footprint
    // and every table edge, AND does its enclosing cell still pass
    // isTraversable() (Free, not Unknown/Occupied, not table-edge-close,
    // not in `extraBlockedCells`)?" simplifyPath() below uses this at many
    // points along a candidate straight shortcut segment (not merely the
    // Bresenham-sampled grid cells the old, pre-Phase-13X-final-fix
    // implementation checked) so a simplified waypoint pair is proven
    // continuously clear, not merely clear at the handful of grid cells a
    // discrete line-walk happened to sample. Not free (O(occupied cells
    // near `point`) per call) - only ever called from simplifyPath() at a
    // bounded number of sub-cell sample points per segment, never from
    // planPath()'s own per-frame-cheap A* search.
    bool isPointClear(const Vec3& point) const noexcept;

    int width() const noexcept;
    int height() const noexcept;
    const ExplorationMap& map() const noexcept;

    // A* search from `startWorld` to `goalWorld` (both projected to grid
    // cells via ExplorationMap::worldToCell()) - see class docs for
    // connectivity, clearance, Unknown policy, and goal snapping. The
    // START cell is exempted from the isTraversable() requirement (the
    // robot may legitimately already be sitting inside another obstacle's
    // planning-clearance margin without actually colliding - see class
    // docs); every other cell in the path, including the first cell
    // stepped into, must be traversable. Returns success=false (empty
    // waypoints/cellPath) if start or goal falls entirely outside `map`'s
    // bounds, or no path exists.
    PathPlanResult planPath(const Vec3& startWorld, const Vec3& goalWorld) const;

private:
    const ExplorationMap& map_;
    int width_;
    int height_;
    std::vector<bool> traversable_;

    // Phase 13X final blocker fix: retained (rather than only ever a
    // constructor-local) so isPointClear() can reuse the exact same
    // occupied-cell list/clearance/search-radius the constructor's own
    // per-cell pass already computed - never a second, independently
    // recomputed copy.
    std::vector<GridCoord> occupiedCells_;
    float clearance_ = 0.0F;
    int inflationRadiusCells_ = 0;
};

// Deterministic greedy line-of-sight ("string pulling") simplification of
// a dense cell-by-cell A* path (Phase 13X) - collapses `cellPath` into the
// minimal set of waypoints such that every straight segment between
// consecutive waypoints stays entirely over `planner`-traversable cells.
// Always keeps the first and last point of `cellPath` (a single-cell path
// returns exactly that one point, a two-cell path returns exactly two).
// Never crosses an inflated-occupied/edge-margin cell, even if doing so
// would produce a shorter/straighter-looking route - see
// GridPathPlannerTests.cpp's SimplificationNeverCrossesOccupiedInflatedCell.
std::vector<Vec3> simplifyPath(const ExplorationMap& map, const GridPathPlanner& planner,
                                const std::vector<GridCoord>& cellPath);

} // namespace robot::visual
