#include "robot/visual/FrontierExplorer.hpp"

#include <cmath>
#include <deque>
#include <limits>

namespace robot::visual
{

namespace
{
constexpr int kFourConnectedCol[4] = {0, 0, 1, -1};
constexpr int kFourConnectedRow[4] = {1, -1, 0, 0};
constexpr int kEightConnectedCol[8] = {0, 0, 1, -1, 1, -1, 1, -1};
constexpr int kEightConnectedRow[8] = {1, -1, 0, 0, 1, 1, -1, -1};

float distanceWorld(const Vec3& a, const Vec3& b) noexcept
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dz * dz));
}

bool contains(const std::vector<GridCoord>& list, GridCoord value) noexcept
{
    for (const GridCoord& item : list)
    {
        if (item == value)
        {
            return true;
        }
    }
    return false;
}
} // namespace

FrontierExplorer::FrontierExplorer(const ExplorationMap& map) noexcept
    : map_(map)
{
}

std::vector<GridCoord> FrontierExplorer::detectFrontierCells(const GridPathPlanner& planner) const
{
    std::vector<GridCoord> frontierCells;
    for (int row = 0; row < map_.height(); ++row)
    {
        for (int col = 0; col < map_.width(); ++col)
        {
            if (!planner.isTraversable(col, row))
            {
                continue;
            }
            bool bordersUnknown = false;
            for (int n = 0; n < 4; ++n)
            {
                const int nc = col + kFourConnectedCol[n];
                const int nr = row + kFourConnectedRow[n];
                if (map_.cellAt(nc, nr) == MapCell::Unknown)
                {
                    bordersUnknown = true;
                    break;
                }
            }
            if (bordersUnknown)
            {
                frontierCells.push_back(GridCoord{col, row});
            }
        }
    }
    return frontierCells;
}

std::vector<std::vector<GridCoord>> FrontierExplorer::clusterFrontierCells(
    const std::vector<GridCoord>& frontierCells) const
{
    std::vector<std::vector<GridCoord>> clusters;
    if (frontierCells.empty())
    {
        return clusters;
    }

    const int width = map_.width();
    const int height = map_.height();
    const auto index = [width](GridCoord c) {
        return (static_cast<std::size_t>(c.row) * static_cast<std::size_t>(width)) + static_cast<std::size_t>(c.col);
    };

    std::vector<bool> isFrontier(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), false);
    for (const GridCoord& cell : frontierCells)
    {
        isFrontier[index(cell)] = true;
    }
    std::vector<bool> visited(isFrontier.size(), false);

    // Deterministic seed order: frontierCells is itself already produced
    // in row-major scan order (see detectFrontierCells()).
    for (const GridCoord& seed : frontierCells)
    {
        if (visited[index(seed)])
        {
            continue;
        }
        std::vector<GridCoord> cluster;
        std::deque<GridCoord> queue;
        queue.push_back(seed);
        visited[index(seed)] = true;
        while (!queue.empty())
        {
            const GridCoord current = queue.front();
            queue.pop_front();
            cluster.push_back(current);
            for (int n = 0; n < 8; ++n)
            {
                const int nc = current.col + kEightConnectedCol[n];
                const int nr = current.row + kEightConnectedRow[n];
                if (nc < 0 || nc >= width || nr < 0 || nr >= height)
                {
                    continue;
                }
                const GridCoord neighbor{nc, nr};
                const std::size_t neighborIndex = index(neighbor);
                if (!isFrontier[neighborIndex] || visited[neighborIndex])
                {
                    continue;
                }
                visited[neighborIndex] = true;
                queue.push_back(neighbor);
            }
        }
        clusters.push_back(std::move(cluster));
    }

    return clusters;
}

FrontierTarget FrontierExplorer::selectTarget(const Vec3& robotWorldPosition,
                                               const std::vector<GridCoord>& blacklist) const
{
    const GridPathPlanner planner(map_);
    const std::vector<GridCoord> frontierCells = detectFrontierCells(planner);
    if (frontierCells.empty())
    {
        return FrontierTarget{};
    }

    const std::vector<std::vector<GridCoord>> clusters = clusterFrontierCells(frontierCells);

    FrontierTarget best;
    float bestScore = std::numeric_limits<float>::infinity();

    for (const std::vector<GridCoord>& cluster : clusters)
    {
        if (cluster.size() < kMinimumFrontierClusterSize)
        {
            continue;
        }

        // Representative: cluster member closest (straight-line) to the
        // robot - deterministic tie-break by keeping the first-found
        // minimum in the cluster's own (row-major seeded, BFS) order.
        GridCoord representative = cluster.front();
        float representativeDistance = std::numeric_limits<float>::infinity();
        for (const GridCoord& member : cluster)
        {
            const float d = distanceWorld(robotWorldPosition, map_.cellToWorld(member.col, member.row));
            if (d < representativeDistance)
            {
                representativeDistance = d;
                representative = member;
            }
        }

        if (contains(blacklist, representative))
        {
            continue;
        }

        const Vec3 targetWorld = map_.cellToWorld(representative.col, representative.row);
        const PathPlanResult planResult = planner.planPath(robotWorldPosition, targetWorld);
        if (!planResult.success)
        {
            continue;
        }

        const float score =
            planResult.pathCostWorldUnits - (kInformationGainWeight * static_cast<float>(cluster.size()));
        if (score < bestScore)
        {
            bestScore = score;
            best.found = true;
            best.worldPosition = targetWorld;
            best.cell = representative;
            best.pathWaypoints = planResult.waypoints;
            best.pathCost = planResult.pathCostWorldUnits;
            best.clusterSize = cluster.size();
        }
    }

    return best;
}

std::vector<GridCoord> FrontierExplorer::frontierCells() const
{
    const GridPathPlanner planner(map_);
    return detectFrontierCells(planner);
}

std::vector<std::vector<GridCoord>> FrontierExplorer::frontierClusters() const
{
    return clusterFrontierCells(frontierCells());
}

ExplorationCompletion FrontierExplorer::completion(const Vec3& robotWorldPosition) const
{
    const GridPathPlanner planner(map_);
    const std::vector<GridCoord> frontierCells = detectFrontierCells(planner);
    if (frontierCells.empty())
    {
        return ExplorationCompletion::Complete;
    }

    const FrontierTarget target = selectTarget(robotWorldPosition);
    return target.found ? ExplorationCompletion::Exploring : ExplorationCompletion::NoReachableFrontier;
}

} // namespace robot::visual
