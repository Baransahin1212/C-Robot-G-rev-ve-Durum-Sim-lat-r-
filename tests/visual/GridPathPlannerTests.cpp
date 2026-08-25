#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "robot/visual/GridPathPlanner.hpp"
#include "robot/visual/RobotCollision.hpp"

namespace
{

using robot::visual::ExplorationMap;
using robot::visual::GridCoord;
using robot::visual::GridPathPlanner;
using robot::visual::kPlanningSafetyMargin;
using robot::visual::kRobotCollisionRadius;
using robot::visual::MapCell;
using robot::visual::PathPlanResult;
using robot::visual::simplifyPath;
using robot::visual::TableSurface;
using robot::visual::Vec3;

// A generously sized 4.8x4.8 table -> exactly 40x40 cells at
// ExplorationMap::kCellSizeWorldUnits (0.12F). Planning clearance
// (kRobotCollisionRadius + kPlanningSafetyMargin, ~0.40F, ~3.3 cells) is
// large relative to a single cell - a small test grid leaves almost no
// genuinely traversable interior once table-edge margin is applied, so
// these synthetic tests deliberately use a large grid with generous
// (>= 6-cell, ~0.72F) buffers between any two features, comfortably
// exceeding the real clearance requirement with no fiddly off-by-one edge
// arithmetic. Production-scale geometry (the real ~8x4 desk) is exercised
// separately below (MonitorKeyboardMouseLayoutCanProduceValidPaths/
// DockGoalReachable).
TableSurface bigBounds()
{
    return TableSurface{-2.4F, 2.4F, -2.4F, 2.4F};
}

void markAllFree(ExplorationMap& map)
{
    for (int row = 0; row < map.height(); ++row)
    {
        for (int col = 0; col < map.width(); ++col)
        {
            map.markFree(col, row);
        }
    }
}

void markOccupied(ExplorationMap& map, int colFrom, int colTo, int rowFrom, int rowTo)
{
    for (int row = rowFrom; row <= rowTo; ++row)
    {
        for (int col = colFrom; col <= colTo; ++col)
        {
            map.markOccupied(col, row);
        }
    }
}

float distanceWorld(const Vec3& a, const Vec3& b)
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dz * dz));
}

// --- 1: EmptyKnownMapFindsStraightPath ---
TEST(GridPathPlannerTest, EmptyKnownMapFindsStraightPath)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(8, 8);
    const Vec3 goal = map.cellToWorld(30, 30);
    const PathPlanResult result = planner.planPath(start, goal);

    ASSERT_TRUE(result.success);
    ASSERT_FALSE(result.waypoints.empty());
    EXPECT_LT(distanceWorld(result.waypoints.front(), start), ExplorationMap::kCellSizeWorldUnits * 1.5F);
    EXPECT_LT(distanceWorld(result.waypoints.back(), goal), ExplorationMap::kCellSizeWorldUnits * 1.5F);
}

// --- 2: OccupiedWallForcesDetour ---
TEST(GridPathPlannerTest, OccupiedWallForcesDetour)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    // A wall across row 20, cols 6-24, leaving a wide gap (cols 25-33,
    // comfortably clear of both the wall's end and the table edge) - the
    // only way to cross row 20 at all.
    markOccupied(map, 6, 24, 20, 20);
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(8, 10);
    const Vec3 goal = map.cellToWorld(8, 30);
    const PathPlanResult result = planner.planPath(start, goal);

    ASSERT_TRUE(result.success);
    for (const GridCoord& cell : result.cellPath)
    {
        EXPECT_NE(map.cellAt(cell.col, cell.row), MapCell::Occupied);
    }
}

// --- 3: NoPathReturnsFailure ---
TEST(GridPathPlannerTest, NoPathReturnsFailure)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    // A full-width wall with no gap at all.
    markOccupied(map, 0, map.width() - 1, 20, 20);
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(8, 10);
    const Vec3 goal = map.cellToWorld(8, 30);
    const PathPlanResult result = planner.planPath(start, goal);

    EXPECT_FALSE(result.success);
    EXPECT_TRUE(result.waypoints.empty());
}

// --- 4: UnknownBlockedForReturnHome ---
TEST(GridPathPlannerTest, UnknownBlockedForReturnHome)
{
    ExplorationMap map(bigBounds());
    // Only mark small Free patches around start/goal - everything else
    // (including the corridor between them) stays Unknown, with no
    // Occupied cell anywhere. If Unknown were traversable, this would
    // trivially succeed; it must not.
    for (int row = 7; row <= 9; ++row)
    {
        for (int col = 7; col <= 9; ++col)
        {
            map.markFree(col, row);
        }
    }
    for (int row = 29; row <= 31; ++row)
    {
        for (int col = 29; col <= 31; ++col)
        {
            map.markFree(col, row);
        }
    }
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(8, 8);
    const Vec3 goal = map.cellToWorld(30, 30);
    const PathPlanResult result = planner.planPath(start, goal);

    EXPECT_FALSE(result.success);
}

// --- 5: InflationKeepsPathAwayFromObstacle ---
TEST(GridPathPlannerTest, InflationKeepsPathAwayFromObstacle)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    markOccupied(map, 18, 20, 18, 20);
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(8, 8);
    const Vec3 goal = map.cellToWorld(30, 30);
    const PathPlanResult result = planner.planPath(start, goal);

    ASSERT_TRUE(result.success);
    const float clearance = kRobotCollisionRadius + kPlanningSafetyMargin;
    const Vec3 obstacleCenter = map.cellToWorld(19, 19);
    for (const GridCoord& cell : result.cellPath)
    {
        const Vec3 cellCenter = map.cellToWorld(cell.col, cell.row);
        EXPECT_GE(distanceWorld(cellCenter, obstacleCenter), clearance - ExplorationMap::kCellSizeWorldUnits);
    }
}

// --- 6: InflationUsesRobotRadius ---
TEST(GridPathPlannerTest, InflationUsesRobotRadius)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    const int occCol = 20;
    const int occRow = 20;
    map.markOccupied(occCol, occRow);
    GridPathPlanner planner(map);

    const float clearance = kRobotCollisionRadius + kPlanningSafetyMargin;
    const Vec3 occupiedCenter = map.cellToWorld(occCol, occRow);

    bool foundFarTraversable = false;
    bool foundNearBlocked = false;
    for (int row = 10; row <= 30; ++row)
    {
        for (int col = 10; col <= 30; ++col)
        {
            if (col == occCol && row == occRow)
            {
                continue;
            }
            const float d = distanceWorld(map.cellToWorld(col, row), occupiedCenter);
            if (d < clearance && !planner.isTraversable(col, row))
            {
                foundNearBlocked = true;
            }
            if (d > clearance + ExplorationMap::kCellSizeWorldUnits && planner.isTraversable(col, row))
            {
                foundFarTraversable = true;
            }
        }
    }
    EXPECT_TRUE(foundNearBlocked);
    EXPECT_TRUE(foundFarTraversable);
}

// --- 7: TableEdgeClearanceApplied ---
TEST(GridPathPlannerTest, TableEdgeClearanceApplied)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    GridPathPlanner planner(map);

    // Column 0 sits at the very left table edge - within clearance of
    // bounds.minX regardless of any obstacle.
    EXPECT_FALSE(planner.isTraversable(0, map.height() / 2));

    // The center of the map is far from every edge.
    EXPECT_TRUE(planner.isTraversable(map.width() / 2, map.height() / 2));
}

// --- 8: DiagonalCornerCuttingPrevented ---
//
// At this project's actual robot scale (kRobotCollisionRadius +
// kPlanningSafetyMargin ~= 0.40F, roughly 3.3 grid cells), two occupied
// cells close enough to form a genuine 1-cell diagonal "corner-cut"
// scenario are already close enough that inflation alone would block both
// cells the cut would connect - a narrow contrived single-cell gap cannot
// isolate corner-cutting from inflation in a meaningful way at production
// scale (see docs/technical-decisions.md, Phase 13X). This test instead
// verifies the actual guaranteed INVARIANT directly against a real,
// successful multi-step detour around an obstacle (the same scenario as
// OccupiedWallForcesDetour): no consecutive pair of cells in the returned
// path is ever a diagonal step whose two orthogonal "cutting" neighbors
// are BOTH occupied - the exact rule GridPathPlanner::planPath()'s
// neighbor-expansion enforces on every candidate diagonal move.
TEST(GridPathPlannerTest, DiagonalCornerCuttingPrevented)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    markOccupied(map, 6, 24, 20, 20);
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(8, 10);
    const Vec3 goal = map.cellToWorld(8, 30);
    const PathPlanResult result = planner.planPath(start, goal);
    ASSERT_TRUE(result.success);

    for (std::size_t i = 1; i < result.cellPath.size(); ++i)
    {
        const GridCoord& a = result.cellPath[i - 1];
        const GridCoord& b = result.cellPath[i];
        const int dCol = b.col - a.col;
        const int dRow = b.row - a.row;
        const bool diagonal = (dCol != 0) && (dRow != 0);
        if (!diagonal)
        {
            continue;
        }
        const bool orthoAOccupied = map.cellAt(a.col + dCol, a.row) == MapCell::Occupied;
        const bool orthoBOccupied = map.cellAt(a.col, a.row + dRow) == MapCell::Occupied;
        EXPECT_FALSE(orthoAOccupied && orthoBOccupied) << "corner-cut at step " << i;
    }
}

// --- 9: StartEqualsGoal ---
TEST(GridPathPlannerTest, StartEqualsGoal)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    GridPathPlanner planner(map);

    const Vec3 point = map.cellToWorld(20, 20);
    const PathPlanResult result = planner.planPath(point, point);

    ASSERT_TRUE(result.success);
    ASSERT_FALSE(result.waypoints.empty());
    EXPECT_NEAR(result.pathCostWorldUnits, 0.0F, 1e-4F);
}

// --- 10 & 11: PathStartsNearRequestedStart / PathEndsNearRequestedGoal ---
TEST(GridPathPlannerTest, PathStartsNearRequestedStartAndEndsNearRequestedGoal)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(9, 11);
    const Vec3 goal = map.cellToWorld(28, 24);
    const PathPlanResult result = planner.planPath(start, goal);

    ASSERT_TRUE(result.success);
    EXPECT_LT(distanceWorld(result.waypoints.front(), start), ExplorationMap::kCellSizeWorldUnits * 1.5F);
    EXPECT_LT(distanceWorld(result.waypoints.back(), goal), ExplorationMap::kCellSizeWorldUnits * 1.5F);
}

// --- 12: DeterministicForSameInput ---
TEST(GridPathPlannerTest, DeterministicForSameInput)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    markOccupied(map, 17, 22, 15, 25);
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(8, 20);
    const Vec3 goal = map.cellToWorld(32, 20);
    const PathPlanResult first = planner.planPath(start, goal);
    const PathPlanResult second = planner.planPath(start, goal);

    ASSERT_TRUE(first.success);
    ASSERT_TRUE(second.success);
    ASSERT_EQ(first.cellPath.size(), second.cellPath.size());
    for (std::size_t i = 0; i < first.cellPath.size(); ++i)
    {
        EXPECT_EQ(first.cellPath[i].col, second.cellPath[i].col);
        EXPECT_EQ(first.cellPath[i].row, second.cellPath[i].row);
    }
}

// --- 13: RectangularMapSupported ---
TEST(GridPathPlannerTest, RectangularMapSupported)
{
    ExplorationMap map(TableSurface{-2.4F, 2.4F, -1.2F, 1.2F}); // 4.8 x 2.4 -> 40x20
    markAllFree(map);
    ASSERT_NE(map.width(), map.height());
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(8, 8);
    const Vec3 goal = map.cellToWorld(32, 12);
    const PathPlanResult result = planner.planPath(start, goal);

    EXPECT_TRUE(result.success);
}

namespace production
{
// The real production desk (VirtualWorld.cpp): 8x4 table, Monitor/
// Keyboard/Mouse footprints, base platform + dock housing - reproduced
// here as plain ExplorationMap cell data (never via VirtualWorld itself -
// GridPathPlanner must never depend on it) so tests 14/15 exercise the
// exact real geometry.
TableSurface productionBounds()
{
    return TableSurface{-4.0F, 4.0F, -2.0F, 2.0F};
}

void markProductionDeskFree(ExplorationMap& map)
{
    markAllFree(map);
    const auto occupyFootprint = [&](Vec3 position, Vec3 size) {
        const float minX = position.x - (size.x / 2.0F);
        const float maxX = position.x + (size.x / 2.0F);
        const float minZ = position.z - (size.z / 2.0F);
        const float maxZ = position.z + (size.z / 2.0F);
        for (float z = minZ; z <= maxZ; z += ExplorationMap::kCellSizeWorldUnits)
        {
            for (float x = minX; x <= maxX; x += ExplorationMap::kCellSizeWorldUnits)
            {
                int col = -1;
                int row = -1;
                if (map.worldToCell(Vec3{x, 0.0F, z}, col, row))
                {
                    map.markOccupied(col, row);
                }
            }
        }
    };
    occupyFootprint(Vec3{-1.2F, 0.25F, -1.2F}, Vec3{1.2F, 0.5F, 0.5F});    // Monitor
    occupyFootprint(Vec3{-1.2F, 0.05F, -0.4F}, Vec3{2.3F, 0.1F, 0.75F});   // Keyboard
    occupyFootprint(Vec3{0.6F, 0.05F, -0.4F}, Vec3{0.4F, 0.1F, 0.6F});     // Mouse
    occupyFootprint(Vec3{1.3F, 0.11F, -1.85F}, Vec3{0.28F, 0.22F, 0.10F}); // dock housing
}
} // namespace production

// --- 14: MonitorKeyboardMouseLayoutCanProduceValidPaths ---
TEST(GridPathPlannerTest, MonitorKeyboardMouseLayoutCanProduceValidPaths)
{
    ExplorationMap map(production::productionBounds());
    production::markProductionDeskFree(map);
    GridPathPlanner planner(map);

    // Robot start area (near the base) to the open desk interior beyond
    // the keyboard/mouse cluster.
    const Vec3 start{1.3F, 0.0F, -0.5F};
    const Vec3 goal{-2.5F, 0.0F, 0.5F};
    const PathPlanResult result = planner.planPath(start, goal);

    EXPECT_TRUE(result.success);
    for (const GridCoord& cell : result.cellPath)
    {
        EXPECT_NE(map.cellAt(cell.col, cell.row), MapCell::Occupied);
    }
}

// --- 15: DockGoalReachable ---
TEST(GridPathPlannerTest, DockGoalReachable)
{
    ExplorationMap map(production::productionBounds());
    production::markProductionDeskFree(map);
    GridPathPlanner planner(map);

    const Vec3 start{-2.5F, 0.0F, 0.5F};
    const Vec3 dockGoal{1.3F, 0.0F, -1.5F}; // BasePlatform::position, close to the dock housing
    const PathPlanResult result = planner.planPath(start, dockGoal);

    ASSERT_TRUE(result.success);
    // The literal requested dock point is reachable via the final
    // waypoint (goal snapping - see GridPathPlanner's own class docs).
    EXPECT_LT(distanceWorld(result.waypoints.back(), dockGoal), ExplorationMap::kCellSizeWorldUnits * 2.0F);
}

// ============================================================
// Path simplification
// ============================================================

// --- Simplification 1: StraightGridPathBecomesMinimalWaypoints ---
TEST(GridPathPlannerTest, StraightGridPathBecomesMinimalWaypoints)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    GridPathPlanner planner(map);

    std::vector<GridCoord> straightPath;
    for (int col = 5; col <= 35; ++col)
    {
        straightPath.push_back(GridCoord{col, 20});
    }

    const std::vector<Vec3> waypoints = simplifyPath(map, planner, straightPath);
    EXPECT_EQ(waypoints.size(), 2U);
}

// --- Simplification 2: CornerPathPreservesRequiredTurn ---
TEST(GridPathPlannerTest, CornerPathPreservesRequiredTurn)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    markOccupied(map, 20, 20, 6, 20); // vertical wall segment forcing an L-shaped detour
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(10, 10);
    const Vec3 goal = map.cellToWorld(30, 10);
    const PathPlanResult result = planner.planPath(start, goal);

    ASSERT_TRUE(result.success);
    // A genuine detour around a wall cannot simplify down to a single
    // straight 2-point segment.
    EXPECT_GT(result.waypoints.size(), 2U);
}

// --- Simplification 3: SimplificationNeverCrossesOccupiedInflatedCell ---
TEST(GridPathPlannerTest, SimplificationNeverCrossesOccupiedInflatedCell)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    markOccupied(map, 18, 20, 10, 30);
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(8, 20);
    const Vec3 goal = map.cellToWorld(32, 20);
    const PathPlanResult result = planner.planPath(start, goal);
    ASSERT_TRUE(result.success);

    // Sample many points along each simplified segment and confirm every
    // sampled cell is planner-traversable - the structural guarantee
    // simplifyPath()'s own line-of-sight check exists to provide.
    constexpr int kSamplesPerSegment = 40;
    for (std::size_t i = 1; i < result.waypoints.size(); ++i)
    {
        const Vec3& a = result.waypoints[i - 1];
        const Vec3& b = result.waypoints[i];
        for (int s = 0; s <= kSamplesPerSegment; ++s)
        {
            const float t = static_cast<float>(s) / static_cast<float>(kSamplesPerSegment);
            const Vec3 sample{a.x + ((b.x - a.x) * t), 0.0F, a.z + ((b.z - a.z) * t)};
            int col = -1;
            int row = -1;
            if (map.worldToCell(sample, col, row))
            {
                EXPECT_TRUE(planner.isTraversable(col, row))
                    << "segment " << i << " sample " << s << " at (" << sample.x << ", " << sample.z << ")";
            }
        }
    }
}

// --- Simplification 4: RectangularMapCoordinatesPreserved ---
TEST(GridPathPlannerTest, RectangularMapCoordinatesPreserved)
{
    ExplorationMap map(TableSurface{-2.4F, 2.4F, -1.2F, 1.2F});
    markAllFree(map);
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(8, 8);
    const Vec3 goal = map.cellToWorld(32, 12);
    const PathPlanResult result = planner.planPath(start, goal);

    ASSERT_TRUE(result.success);
    for (const Vec3& waypoint : result.waypoints)
    {
        EXPECT_FLOAT_EQ(waypoint.y, 0.0F);
        EXPECT_GE(waypoint.x, map.bounds().minX);
        EXPECT_LE(waypoint.x, map.bounds().maxX);
        EXPECT_GE(waypoint.z, map.bounds().minZ);
        EXPECT_LE(waypoint.z, map.bounds().maxZ);
    }
}

} // namespace
