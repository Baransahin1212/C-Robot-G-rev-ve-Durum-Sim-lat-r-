#include "robot/visual/GridPathPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace robot::visual
{

namespace
{
// 8-connected neighbor offsets, fixed deterministic order - N, S, E, W,
// then the four diagonals. Only the ORDER matters for determinism (ties
// are broken by cell index, not by this order), but a fixed order still
// keeps traversal itself reproducible.
constexpr int kNeighborCol[8] = {0, 0, 1, -1, 1, -1, 1, -1};
constexpr int kNeighborRow[8] = {1, -1, 0, 0, 1, 1, -1, -1};

float distanceWorld(const Vec3& a, const Vec3& b) noexcept
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dz * dz));
}

// Phase 13X final blocker fix ("PATH SIMPLIFICATION AUDIT" / geometry-
// contract unification): distance from `point` to the SQUARE FOOTPRINT of
// one occupied grid cell (center `cellCenter`, half-width `halfCell` -
// i.e. the same [center - halfCell, center + halfCell] extent every cell
// in this grid actually covers), not merely to that cell's own center
// point. Uses the identical clamp-to-box technique
// RobotCollision.cpp's own collidesWithObstacle() already uses for the
// real robot-vs-obstacle check (same "closest point on an axis-aligned
// box" primitive, deliberately reused rather than re-derived) - this is
// what makes GridPathPlanner's own clearance test geometry-compatible
// with the reactive layer's exact rectangular Minkowski-sum-style AABB
// expansion (ForwardClearanceProbe.cpp's segmentIntersectsExpandedAabb())
// instead of the old point-distance-between-cell-centers test, which
// systematically UNDER-counted how close a route came to a real obstacle
// near a DIAGONAL corner (a rectangular obstacle's corner protrudes
// further into a diagonal approach than a circle centered on the nearest
// occupied cell's own center point ever could) - see
// NavigationClearance.hpp and docs/technical-decisions.md (Phase 13X
// final blocker fix) for the full traced reproduction this closes.
float distanceToCellFootprint(const Vec3& point, const Vec3& cellCenter, float halfCell) noexcept
{
    const float minX = cellCenter.x - halfCell;
    const float maxX = cellCenter.x + halfCell;
    const float minZ = cellCenter.z - halfCell;
    const float maxZ = cellCenter.z + halfCell;

    const float closestX = std::clamp(point.x, minX, maxX);
    const float closestZ = std::clamp(point.z, minZ, maxZ);

    const float dx = point.x - closestX;
    const float dz = point.z - closestZ;
    return std::sqrt((dx * dx) + (dz * dz));
}

// Phase 13X final blocker fix: the ONE traversability predicate both the
// per-cell A* grid (GridPathPlanner's own constructor, below) and
// simplifyPath()'s continuous line-of-sight validation (further down this
// file) now share - "is `point` at least `clearance` away from every
// occupied cell's own square footprint (distanceToCellFootprint(), not a
// bare center-to-center distance) AND at least `clearance` inside every
// table-surface edge?" Previously simplifyPath() only ever asked "is each
// Bresenham-SAMPLED grid cell traversable," which is a discrete
// approximation of the continuous straight segment a simplified waypoint
// pair actually drives - fine when the sampled cells happen to track the
// true line closely, but never proven to hold at every point of the
// segment. This function takes a plain world point so it can be called
// both once per grid cell (construction) and many times per simplified
// segment (simplifyPath(), at a sub-cell sampling step - see that
// function's own docs) - never two independently-drifting
// implementations of "close enough to be unsafe."
bool pointClearsInflatedGeometry(const Vec3& point, const TableSurface& bounds,
                                  const std::vector<GridCoord>& occupiedCells, const ExplorationMap& map,
                                  float clearance, int inflationRadiusCells) noexcept
{
    if ((point.x - bounds.minX < clearance) || (bounds.maxX - point.x < clearance) ||
        (point.z - bounds.minZ < clearance) || (bounds.maxZ - point.z < clearance))
    {
        return false;
    }

    int pointCol = -1;
    int pointRow = -1;
    const bool inGrid = map.worldToCell(point, pointCol, pointRow);
    const float halfCell = map.cellSize() / 2.0F;

    for (const GridCoord& occ : occupiedCells)
    {
        if (inGrid && (std::abs(occ.col - pointCol) > inflationRadiusCells ||
                       std::abs(occ.row - pointRow) > inflationRadiusCells))
        {
            continue;
        }
        const Vec3 occCenter = map.cellToWorld(occ.col, occ.row);
        if (distanceToCellFootprint(point, occCenter, halfCell) < clearance)
        {
            return false;
        }
    }
    return true;
}

} // namespace

GridPathPlanner::GridPathPlanner(const ExplorationMap& map, const std::vector<GridCoord>& extraBlockedCells) noexcept
    : map_(map)
    , width_(map.width())
    , height_(map.height())
    , traversable_(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), false)
{
    clearance_ = kRobotCollisionRadius + kPlanningSafetyMargin;
    const TableSurface& bounds = map.bounds();
    const float cellSize = map.cellSize();
    inflationRadiusCells_ = static_cast<int>(std::ceil(clearance_ / cellSize)) + 1;

    // Collect Occupied cells once - reused for every cell's inflation
    // check below (and by isPointClear() later - see this class's own
    // member docs), rather than re-scanning ExplorationMap::cells() per
    // candidate.
    for (int row = 0; row < height_; ++row)
    {
        for (int col = 0; col < width_; ++col)
        {
            if (map.cellAt(col, row) == MapCell::Occupied)
            {
                occupiedCells_.push_back(GridCoord{col, row});
            }
        }
    }

    for (int row = 0; row < height_; ++row)
    {
        for (int col = 0; col < width_; ++col)
        {
            const std::size_t index = (static_cast<std::size_t>(row) * static_cast<std::size_t>(width_)) +
                                       static_cast<std::size_t>(col);
            if (map.cellAt(col, row) != MapCell::Free)
            {
                traversable_[index] = false;
                continue;
            }

            const Vec3 center = map.cellToWorld(col, row);

            // Phase 13X final blocker fix: shared predicate (table-edge
            // margin + box-aware, not center-to-center, occupied-cell
            // inflation) - see pointClearsInflatedGeometry()'s own docs
            // above for why this replaced the old bare
            // distanceWorld(center, occCenter) < clearance check.
            traversable_[index] =
                pointClearsInflatedGeometry(center, bounds, occupiedCells_, map, clearance_, inflationRadiusCells_);
        }
    }

    // Phase 13X blocker fix (deadlock repair): applied AFTER the map-
    // derived pass above, so an extra-blocked cell always overrides
    // whatever the map alone would have said - see this constructor's own
    // parameter docs (GridPathPlanner.hpp) for why this list exists.
    for (const GridCoord& blocked : extraBlockedCells)
    {
        if (blocked.col < 0 || blocked.col >= width_ || blocked.row < 0 || blocked.row >= height_)
        {
            continue;
        }
        const std::size_t index = (static_cast<std::size_t>(blocked.row) * static_cast<std::size_t>(width_)) +
                                   static_cast<std::size_t>(blocked.col);
        traversable_[index] = false;
    }
}

bool GridPathPlanner::isTraversable(int col, int row) const noexcept
{
    if (col < 0 || col >= width_ || row < 0 || row >= height_)
    {
        return false;
    }
    return traversable_[(static_cast<std::size_t>(row) * static_cast<std::size_t>(width_)) +
                         static_cast<std::size_t>(col)];
}

bool GridPathPlanner::isPointClear(const Vec3& point) const noexcept
{
    int col = -1;
    int row = -1;
    if (!map_.worldToCell(point, col, row))
    {
        return false;
    }
    // Enclosing cell must itself already pass isTraversable() - folds in
    // Free/Unknown/Occupied, table-edge-at-the-cell-center, and (Phase 13X
    // blocker fix) extraBlockedCells in one call, exactly like every other
    // caller of isTraversable() - never a second, separately-maintained
    // copy of that logic here.
    if (!isTraversable(col, row))
    {
        return false;
    }
    // Then prove the exact continuous point (not merely its enclosing
    // cell's own center) is itself far enough from every occupied cell's
    // square footprint and every table edge - see
    // pointClearsInflatedGeometry()'s own docs above.
    return pointClearsInflatedGeometry(point, map_.bounds(), occupiedCells_, map_, clearance_, inflationRadiusCells_);
}

int GridPathPlanner::width() const noexcept
{
    return width_;
}

int GridPathPlanner::height() const noexcept
{
    return height_;
}

const ExplorationMap& GridPathPlanner::map() const noexcept
{
    return map_;
}

namespace
{
// Bounded expanding-ring search for the nearest traversable cell to `from`
// (Phase 13X "goal snapping" - see GridPathPlanner's own class docs). Not a
// connectivity-respecting search (it does not walk through walls to prove
// reachability - planPath()'s own A* is what proves that afterward); this
// only ever needs to find a nearby, uninflated cell near a small
// obstruction like the charging dock's rear housing, never to route around
// a genuine wall. Returns false if nothing traversable is found within
// `maxRingRadius` cells.
bool findNearestTraversableCell(const GridPathPlanner& planner, GridCoord from, int maxRingRadius, GridCoord& outCell)
{
    if (planner.isTraversable(from.col, from.row))
    {
        outCell = from;
        return true;
    }
    for (int radius = 1; radius <= maxRingRadius; ++radius)
    {
        // Deterministic scan order: row-major over the ring's bounding
        // box, skipping interior cells already covered by a smaller
        // radius.
        for (int dr = -radius; dr <= radius; ++dr)
        {
            for (int dc = -radius; dc <= radius; ++dc)
            {
                if (std::max(std::abs(dr), std::abs(dc)) != radius)
                {
                    continue;
                }
                const GridCoord candidate{from.col + dc, from.row + dr};
                if (planner.isTraversable(candidate.col, candidate.row))
                {
                    outCell = candidate;
                    return true;
                }
            }
        }
    }
    return false;
}

// Small, fixed search radius for goal snapping - enough to step around a
// small nearby obstruction (e.g. the ~0.28x0.10 dock housing, roughly 2-3
// cells at this project's 0.12F cell size) without silently "solving" a
// genuinely blocked/enclosed test scenario by wandering arbitrarily far.
constexpr int kGoalSnapMaxRingRadius = 6;
} // namespace

PathPlanResult GridPathPlanner::planPath(const Vec3& startWorld, const Vec3& goalWorld) const
{
    PathPlanResult result;

    int startCol = -1;
    int startRow = -1;
    int goalCol = -1;
    int goalRow = -1;
    if (!map_.worldToCell(startWorld, startCol, startRow) || !map_.worldToCell(goalWorld, goalCol, goalRow))
    {
        return result;
    }

    const GridCoord startCell{startCol, startRow};
    GridCoord requestedGoalCell{goalCol, goalRow};
    GridCoord goalCell{};
    const bool goalSnapped = !isTraversable(requestedGoalCell.col, requestedGoalCell.row);
    if (goalSnapped)
    {
        if (!findNearestTraversableCell(*this, requestedGoalCell, kGoalSnapMaxRingRadius, goalCell))
        {
            return result;
        }
    }
    else
    {
        goalCell = requestedGoalCell;
    }

    if (startCell == goalCell)
    {
        result.success = true;
        result.cellPath = {startCell};
        result.waypoints = {map_.cellToWorld(startCell.col, startCell.row)};
        if (goalSnapped)
        {
            result.waypoints.push_back(goalWorld);
        }
        result.pathCostWorldUnits = 0.0F;
        return result;
    }

    const std::size_t cellCount = static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_);
    const auto indexOf = [this](GridCoord c) {
        return (static_cast<std::size_t>(c.row) * static_cast<std::size_t>(width_)) + static_cast<std::size_t>(c.col);
    };

    std::vector<float> gScore(cellCount, std::numeric_limits<float>::infinity());
    std::vector<bool> closed(cellCount, false);
    std::vector<int> cameFrom(cellCount, -1);

    struct OpenNode
    {
        float fScore;
        std::size_t cellIndex;
    };
    struct OpenNodeCompare
    {
        // Min-heap on fScore, deterministic tie-break on ascending
        // cellIndex (never insertion order/pointer/hash) - see class docs.
        bool operator()(const OpenNode& a, const OpenNode& b) const noexcept
        {
            if (a.fScore != b.fScore)
            {
                return a.fScore > b.fScore;
            }
            return a.cellIndex > b.cellIndex;
        }
    };

    std::priority_queue<OpenNode, std::vector<OpenNode>, OpenNodeCompare> open;

    const float cellSize = map_.cellSize();
    const float diagonalCost = cellSize * 1.41421356237F;

    const auto heuristic = [&](GridCoord c) {
        const float dc = static_cast<float>(std::abs(c.col - goalCell.col));
        const float dr = static_cast<float>(std::abs(c.row - goalCell.row));
        return std::sqrt((dc * dc) + (dr * dr)) * cellSize;
    };

    gScore[indexOf(startCell)] = 0.0F;
    open.push(OpenNode{heuristic(startCell), indexOf(startCell)});

    bool found = false;
    while (!open.empty())
    {
        const OpenNode current = open.top();
        open.pop();
        if (closed[current.cellIndex])
        {
            continue;
        }
        closed[current.cellIndex] = true;

        const GridCoord currentCell{static_cast<int>(current.cellIndex % static_cast<std::size_t>(width_)),
                                     static_cast<int>(current.cellIndex / static_cast<std::size_t>(width_))};

        if (currentCell == goalCell)
        {
            found = true;
            break;
        }

        for (int n = 0; n < 8; ++n)
        {
            const GridCoord neighbor{currentCell.col + kNeighborCol[n], currentCell.row + kNeighborRow[n]};
            if (neighbor.col < 0 || neighbor.col >= width_ || neighbor.row < 0 || neighbor.row >= height_)
            {
                continue;
            }
            // Every cell other than the start itself must be traversable
            // (see class docs on the start-cell exemption).
            if (!isTraversable(neighbor.col, neighbor.row))
            {
                continue;
            }

            const bool diagonal = (kNeighborCol[n] != 0) && (kNeighborRow[n] != 0);
            if (diagonal)
            {
                // Corner-cutting prevention: both orthogonal cells the
                // diagonal step would cut across must also be traversable.
                const GridCoord orthoA{currentCell.col + kNeighborCol[n], currentCell.row};
                const GridCoord orthoB{currentCell.col, currentCell.row + kNeighborRow[n]};
                if (!isTraversable(orthoA.col, orthoA.row) || !isTraversable(orthoB.col, orthoB.row))
                {
                    continue;
                }
            }

            const std::size_t neighborIndex = indexOf(neighbor);
            if (closed[neighborIndex])
            {
                continue;
            }

            const float tentativeG = gScore[current.cellIndex] + (diagonal ? diagonalCost : cellSize);
            if (tentativeG < gScore[neighborIndex])
            {
                gScore[neighborIndex] = tentativeG;
                cameFrom[neighborIndex] = static_cast<int>(current.cellIndex);
                open.push(OpenNode{tentativeG + heuristic(neighbor), neighborIndex});
            }
        }
    }

    if (!found)
    {
        return result;
    }

    std::vector<GridCoord> reversePath;
    int cursor = static_cast<int>(indexOf(goalCell));
    while (cursor != -1)
    {
        reversePath.push_back(GridCoord{cursor % width_, cursor / width_});
        cursor = cameFrom[static_cast<std::size_t>(cursor)];
    }
    std::reverse(reversePath.begin(), reversePath.end());

    result.success = true;
    result.cellPath = reversePath;

    float cost = 0.0F;
    for (std::size_t i = 1; i < reversePath.size(); ++i)
    {
        cost += distanceWorld(map_.cellToWorld(reversePath[i - 1].col, reversePath[i - 1].row),
                               map_.cellToWorld(reversePath[i].col, reversePath[i].row));
    }
    result.pathCostWorldUnits = cost;

    result.waypoints = simplifyPath(map_, *this, reversePath);
    if (goalSnapped)
    {
        result.waypoints.push_back(goalWorld);
    }

    return result;
}

namespace
{
// Phase 13X final blocker fix ("PATH SIMPLIFICATION AUDIT" - see
// docs/technical-decisions.md for the full traced reproduction this
// closes): CONTINUOUS straight-segment validity check between two grid
// cells' world-space centers, replacing the old Bresenham-integer-line-
// walk-then-isTraversable()-per-visited-cell approach. The old approach
// only ever proved the handful of grid cells a discrete integer line-walk
// happened to step through were traversable at THEIR OWN cell centers -
// never that the actual continuous straight line the robot would drive
// stays clear of the inflated geometry between those sampled cell
// centers, which is exactly how a diagonal shortcut could graze
// perilously close to (or, per the reactive layer's own rectangular
// hazard geometry, effectively inside) an obstacle's real corner despite
// every individually-sampled grid cell reporting traversable=true.
//
// Samples GridPathPlanner::isPointClear() (isTraversable() of the
// enclosing cell, AND continuous box-aware distance from every occupied
// cell's real footprint - see that method's own docs) at a fixed sub-cell
// step small enough that no two consecutive samples can ever straddle a
// full grid cell in either axis, so a real cell-sized safety margin gap
// can never be skipped over between samples.
constexpr float kLineOfSightSampleStep = ExplorationMap::kCellSizeWorldUnits / 4.0F;

bool lineOfSightTraversable(const GridPathPlanner& planner, const ExplorationMap& map, GridCoord a, GridCoord b)
{
    const Vec3 from = map.cellToWorld(a.col, a.row);
    const Vec3 to = map.cellToWorld(b.col, b.row);
    const float dx = to.x - from.x;
    const float dz = to.z - from.z;
    const float length = std::sqrt((dx * dx) + (dz * dz));
    const int steps = std::max(1, static_cast<int>(std::ceil(length / kLineOfSightSampleStep)));
    for (int step = 0; step <= steps; ++step)
    {
        const float t = static_cast<float>(step) / static_cast<float>(steps);
        const Vec3 sample{from.x + (dx * t), 0.0F, from.z + (dz * t)};
        if (!planner.isPointClear(sample))
        {
            return false;
        }
    }
    return true;
}
} // namespace

std::vector<Vec3> simplifyPath(const ExplorationMap& map, const GridPathPlanner& planner,
                                const std::vector<GridCoord>& cellPath)
{
    std::vector<Vec3> waypoints;
    if (cellPath.empty())
    {
        return waypoints;
    }

    waypoints.push_back(map.cellToWorld(cellPath.front().col, cellPath.front().row));
    if (cellPath.size() == 1)
    {
        return waypoints;
    }

    std::size_t anchor = 0;
    while (anchor < cellPath.size() - 1)
    {
        std::size_t farthest = anchor + 1;
        for (std::size_t candidate = cellPath.size() - 1; candidate > anchor + 1; --candidate)
        {
            if (lineOfSightTraversable(planner, map, cellPath[anchor], cellPath[candidate]))
            {
                farthest = candidate;
                break;
            }
        }
        waypoints.push_back(map.cellToWorld(cellPath[farthest].col, cellPath[farthest].row));
        anchor = farthest;
    }

    return waypoints;
}

} // namespace robot::visual
