#include <algorithm>
#include <vector>

#include <gtest/gtest.h>

#include "robot/visual/FrontierExplorer.hpp"

namespace
{

using robot::visual::ExplorationCompletion;
using robot::visual::ExplorationMap;
using robot::visual::FrontierExplorer;
using robot::visual::FrontierTarget;
using robot::visual::GridCoord;
using robot::visual::kMinimumFrontierClusterSize;
using robot::visual::MapCell;
using robot::visual::TableSurface;
using robot::visual::Vec3;

// Same generously sized grid as GridPathPlannerTests.cpp, for the same
// reason - planning clearance is large relative to a single cell, so a
// small test grid leaves too little genuinely traversable interior. See
// that file's own docs.
TableSurface bigBounds()
{
    return TableSurface{-2.4F, 2.4F, -2.4F, 2.4F};
}

void markFreeBlock(ExplorationMap& map, int colFrom, int colTo, int rowFrom, int rowTo)
{
    for (int row = rowFrom; row <= rowTo; ++row)
    {
        for (int col = colFrom; col <= colTo; ++col)
        {
            map.markFree(col, row);
        }
    }
}

void markOccupiedBlock(ExplorationMap& map, int colFrom, int colTo, int rowFrom, int rowTo)
{
    for (int row = rowFrom; row <= rowTo; ++row)
    {
        for (int col = colFrom; col <= colTo; ++col)
        {
            map.markOccupied(col, row);
        }
    }
}

bool containsCell(const std::vector<GridCoord>& cells, GridCoord value)
{
    return std::find(cells.begin(), cells.end(), value) != cells.end();
}

// --- 1: FullyUnknownMapHasNoReachableFrontierWithoutFreeSeed ---
TEST(FrontierExplorerTest, FullyUnknownMapHasNoReachableFrontierWithoutFreeSeed)
{
    ExplorationMap map(bigBounds());
    FrontierExplorer explorer(map);

    EXPECT_TRUE(explorer.frontierCells().empty());
    const FrontierTarget target = explorer.selectTarget(map.cellToWorld(20, 20));
    EXPECT_FALSE(target.found);
    EXPECT_EQ(explorer.completion(map.cellToWorld(20, 20)), ExplorationCompletion::Complete);
}

// --- 2: FreeCellAdjacentToUnknownIsFrontier ---
TEST(FrontierExplorerTest, FreeCellAdjacentToUnknownIsFrontier)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 10, 20, 10, 20);
    FrontierExplorer explorer(map);

    const std::vector<GridCoord> frontier = explorer.frontierCells();
    ASSERT_FALSE(frontier.empty());
    for (const GridCoord& cell : frontier)
    {
        EXPECT_EQ(map.cellAt(cell.col, cell.row), MapCell::Free);
        const bool bordersUnknown = map.cellAt(cell.col + 1, cell.row) == MapCell::Unknown ||
                                     map.cellAt(cell.col - 1, cell.row) == MapCell::Unknown ||
                                     map.cellAt(cell.col, cell.row + 1) == MapCell::Unknown ||
                                     map.cellAt(cell.col, cell.row - 1) == MapCell::Unknown;
        EXPECT_TRUE(bordersUnknown);
    }
}

// --- 3: InteriorFreeCellIsNotFrontier ---
TEST(FrontierExplorerTest, InteriorFreeCellIsNotFrontier)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 10, 20, 10, 20);
    FrontierExplorer explorer(map);

    const std::vector<GridCoord> frontier = explorer.frontierCells();
    EXPECT_FALSE(containsCell(frontier, GridCoord{15, 15})); // deep interior of the blob
}

// --- 4: OccupiedCellIsNeverFrontier ---
TEST(FrontierExplorerTest, OccupiedCellIsNeverFrontier)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 10, 20, 10, 20);
    map.markOccupied(10, 15); // overwrite a border cell as Occupied instead of Free
    FrontierExplorer explorer(map);

    const std::vector<GridCoord> frontier = explorer.frontierCells();
    EXPECT_FALSE(containsCell(frontier, GridCoord{10, 15}));
}

// --- 5: FrontierClustersDetected ---
TEST(FrontierExplorerTest, FrontierClustersDetected)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 8, 13, 8, 13);   // blob A
    markFreeBlock(map, 25, 30, 25, 30); // blob B, far away
    FrontierExplorer explorer(map);

    const std::vector<std::vector<GridCoord>> clusters = explorer.frontierClusters();
    EXPECT_GE(clusters.size(), 2U);
}

// --- 6: TinyNoiseClusterHandledDeterministically ---
TEST(FrontierExplorerTest, TinyNoiseClusterHandledDeterministically)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 10, 20, 10, 20); // large blob - real cluster
    map.markFree(33, 20);               // isolated single-cell "island" - noise
    FrontierExplorer explorer(map);

    bool foundTinyCluster = false;
    for (const std::vector<GridCoord>& cluster : explorer.frontierClusters())
    {
        if (cluster.size() < kMinimumFrontierClusterSize)
        {
            foundTinyCluster = true;
        }
    }
    EXPECT_TRUE(foundTinyCluster);

    const FrontierTarget target = explorer.selectTarget(map.cellToWorld(15, 15));
    ASSERT_TRUE(target.found);
    EXPECT_NE(target.cell, (GridCoord{33, 20}));
    EXPECT_GE(target.clusterSize, kMinimumFrontierClusterSize);
}

// --- 7: ReachableFrontierSelected ---
TEST(FrontierExplorerTest, ReachableFrontierSelected)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 10, 20, 10, 20);
    FrontierExplorer explorer(map);

    const FrontierTarget target = explorer.selectTarget(map.cellToWorld(15, 15));
    ASSERT_TRUE(target.found);
    EXPECT_FALSE(target.pathWaypoints.empty());
    EXPECT_EQ(map.cellAt(target.cell.col, target.cell.row), MapCell::Free);
}

// --- 8: UnreachableFrontierSkipped ---
TEST(FrontierExplorerTest, UnreachableFrontierSkipped)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 8, 14, 8, 14);   // blob A - robot's reachable side
    markOccupiedBlock(map, 20, 20, 0, 39); // full-height dividing wall, no gap
    markFreeBlock(map, 25, 31, 8, 14);  // blob C - unreachable from blob A
    FrontierExplorer explorer(map);

    const FrontierTarget target = explorer.selectTarget(map.cellToWorld(11, 11));
    ASSERT_TRUE(target.found);
    EXPECT_LT(target.cell.col, 20); // must be blob A's side, never blob C's
}

// --- 9: SelectionUsesPathCost ---
TEST(FrontierExplorerTest, SelectionUsesPathCost)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 10, 15, 10, 15); // close blob, robot right next to it
    markFreeBlock(map, 28, 33, 28, 33); // far blob, same size
    FrontierExplorer explorer(map);

    const FrontierTarget target = explorer.selectTarget(map.cellToWorld(12, 12));
    ASSERT_TRUE(target.found);
    // The closer cluster (blob A, cols 10-15) must win over the equally-
    // sized but much farther one.
    EXPECT_LT(target.cell.col, 20);
}

// --- 10: TargetIsFreeNotUnknown ---
TEST(FrontierExplorerTest, TargetIsFreeNotUnknown)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 10, 20, 10, 20);
    FrontierExplorer explorer(map);

    const FrontierTarget target = explorer.selectTarget(map.cellToWorld(15, 15));
    ASSERT_TRUE(target.found);
    EXPECT_NE(map.cellAt(target.cell.col, target.cell.row), MapCell::Unknown);
}

// --- 11: SameMapProducesSameTarget ---
TEST(FrontierExplorerTest, SameMapProducesSameTarget)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 10, 20, 10, 20);
    FrontierExplorer explorer(map);

    const Vec3 robotPos = map.cellToWorld(15, 15);
    const FrontierTarget first = explorer.selectTarget(robotPos);
    const FrontierTarget second = explorer.selectTarget(robotPos);

    ASSERT_EQ(first.found, second.found);
    ASSERT_TRUE(first.found);
    EXPECT_EQ(first.cell, second.cell);
}

// --- 12: MapUpdateCanCreateNewFrontier ---
TEST(FrontierExplorerTest, MapUpdateCanCreateNewFrontier)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 12, 18, 12, 18);
    FrontierExplorer beforeExplorer(map);
    const std::vector<GridCoord> before = beforeExplorer.frontierCells();
    ASSERT_FALSE(before.empty());

    // Simulate the map growing (new sensor observations) - one extra ring
    // outward.
    markFreeBlock(map, 11, 19, 11, 19);
    FrontierExplorer afterExplorer(map);
    const std::vector<GridCoord> after = afterExplorer.frontierCells();
    ASSERT_FALSE(after.empty());

    // The old innermost border (e.g. (12,15)) is no longer a frontier
    // cell (its neighbor is Free now, not Unknown); a cell on the new,
    // farther-out border is.
    EXPECT_FALSE(containsCell(after, GridCoord{12, 15}));
    EXPECT_TRUE(containsCell(after, GridCoord{11, 15}));
}

// --- 13: ExploredRegionExpansionMovesTarget ---
TEST(FrontierExplorerTest, ExploredRegionExpansionMovesTarget)
{
    ExplorationMap map(bigBounds());
    markFreeBlock(map, 12, 18, 12, 18);
    const Vec3 robotPos = map.cellToWorld(15, 15);

    FrontierExplorer beforeExplorer(map);
    const FrontierTarget before = beforeExplorer.selectTarget(robotPos);
    ASSERT_TRUE(before.found);

    markFreeBlock(map, 11, 19, 11, 19);
    FrontierExplorer afterExplorer(map);
    const FrontierTarget after = afterExplorer.selectTarget(robotPos);
    ASSERT_TRUE(after.found);

    EXPECT_NE(before.cell, after.cell);
}

// --- 14: NoReachableFrontierReportsComplete (completion() semantics) ---
TEST(FrontierExplorerTest, NoReachableFrontierReportsComplete)
{
    ExplorationMap map(bigBounds());
    // The robot's own side (cols 0-19) has NO Free cells at all - only
    // Unknown, so it contributes zero frontier candidates and nothing for
    // A* to expand into. The only frontier cluster on the whole map sits
    // on the far side of a full-height, gap-free dividing wall.
    markOccupiedBlock(map, 20, 20, 0, 39);
    markFreeBlock(map, 25, 31, 8, 14);

    FrontierExplorer explorer(map);
    const Vec3 robotPos = map.cellToWorld(11, 11);

    ASSERT_FALSE(explorer.frontierCells().empty());
    EXPECT_EQ(explorer.completion(robotPos), ExplorationCompletion::NoReachableFrontier);
}

// --- 15: RectangularDeskSupported ---
TEST(FrontierExplorerTest, RectangularDeskSupported)
{
    ExplorationMap map(TableSurface{-2.4F, 2.4F, -1.2F, 1.2F}); // 40x20
    markFreeBlock(map, 10, 20, 6, 13);
    FrontierExplorer explorer(map);

    const FrontierTarget target = explorer.selectTarget(map.cellToWorld(15, 9));
    EXPECT_TRUE(target.found);
}

} // namespace
