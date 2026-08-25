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

// Bresenham-style integer line walk between two grid cells (inclusive of
// both endpoints) - shared by the goal-snapping search's "is this cell
// visible" style checks and simplifyPath()'s own line-of-sight test.
std::vector<GridCoord> walkLine(GridCoord from, GridCoord to)
{
    std::vector<GridCoord> cells;
    int x0 = from.col;
    int y0 = from.row;
    const int x1 = to.col;
    const int y1 = to.row;
    const int dx = std::abs(x1 - x0);
    const int dy = -std::abs(y1 - y0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;
    while (true)
    {
        cells.push_back(GridCoord{x0, y0});
        if (x0 == x1 && y0 == y1)
        {
            break;
        }
        const int e2 = 2 * err;
        if (e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
    return cells;
}
} // namespace

GridPathPlanner::GridPathPlanner(const ExplorationMap& map) noexcept
    : map_(map)
    , width_(map.width())
    , height_(map.height())
    , traversable_(static_cast<std::size_t>(width_) * static_cast<std::size_t>(height_), false)
{
    const float clearance = kRobotCollisionRadius + kPlanningSafetyMargin;
    const TableSurface& bounds = map.bounds();
    const float cellSize = map.cellSize();
    const int inflationRadiusCells = static_cast<int>(std::ceil(clearance / cellSize)) + 1;

    // Collect Occupied cells once - reused for every cell's inflation
    // check below, rather than re-scanning ExplorationMap::cells() per
    // candidate.
    std::vector<GridCoord> occupiedCells;
    for (int row = 0; row < height_; ++row)
    {
        for (int col = 0; col < width_; ++col)
        {
            if (map.cellAt(col, row) == MapCell::Occupied)
            {
                occupiedCells.push_back(GridCoord{col, row});
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

            // Table-edge planning margin.
            bool safe = (center.x - bounds.minX >= clearance) && (bounds.maxX - center.x >= clearance) &&
                        (center.z - bounds.minZ >= clearance) && (bounds.maxZ - center.z >= clearance);

            // Occupied-cell inflation - only check nearby occupied cells
            // (bounded by inflationRadiusCells), not the whole list, for
            // every candidate.
            if (safe)
            {
                for (const GridCoord& occ : occupiedCells)
                {
                    if (std::abs(occ.col - col) > inflationRadiusCells || std::abs(occ.row - row) > inflationRadiusCells)
                    {
                        continue;
                    }
                    const Vec3 occCenter = map.cellToWorld(occ.col, occ.row);
                    if (distanceWorld(center, occCenter) < clearance)
                    {
                        safe = false;
                        break;
                    }
                }
            }

            traversable_[index] = safe;
        }
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
bool lineOfSightTraversable(const GridPathPlanner& planner, GridCoord a, GridCoord b)
{
    for (const GridCoord& cell : walkLine(a, b))
    {
        if (!planner.isTraversable(cell.col, cell.row))
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
            if (lineOfSightTraversable(planner, cellPath[anchor], cellPath[candidate]))
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
