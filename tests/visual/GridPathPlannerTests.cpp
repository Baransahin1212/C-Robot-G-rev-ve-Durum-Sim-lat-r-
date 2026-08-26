#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "robot/visual/GridPathPlanner.hpp"
#include "robot/visual/NavigationClearance.hpp"
#include "robot/visual/RobotCollision.hpp"

namespace
{

using robot::visual::ExplorationMap;
using robot::visual::GridCoord;
using robot::visual::GridPathPlanner;
using robot::visual::kPlanningSafetyMargin;
using robot::visual::kRobotCollisionRadius;
using robot::visual::MapCell;
using robot::visual::NavigationClearance::kLocalHazardRadius;
using robot::visual::NavigationClearance::kPlanningRadius;
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

// Phase 13X final blocker fix: distance from `point` to the real SQUARE
// FOOTPRINT of the nearest Occupied cell in `map` (center +- half a cell,
// via the standard clamp-to-box technique) - deliberately reproduced HERE
// as an independent check, rather than calling GridPathPlanner's own
// (identically-derived) isPointClear()/pointClearsInflatedGeometry(), so
// these tests verify the actual geometric invariant against production
// desk geometry, not merely that the implementation agrees with itself.
float minDistanceToOccupiedFootprint(const ExplorationMap& map, const Vec3& point)
{
    float best = std::numeric_limits<float>::infinity();
    const float halfCell = ExplorationMap::kCellSizeWorldUnits / 2.0F;
    for (int row = 0; row < map.height(); ++row)
    {
        for (int col = 0; col < map.width(); ++col)
        {
            if (map.cellAt(col, row) != MapCell::Occupied)
            {
                continue;
            }
            const Vec3 center = map.cellToWorld(col, row);
            const float closestX = std::clamp(point.x, center.x - halfCell, center.x + halfCell);
            const float closestZ = std::clamp(point.z, center.z - halfCell, center.z + halfCell);
            const float dx = point.x - closestX;
            const float dz = point.z - closestZ;
            best = std::min(best, std::sqrt((dx * dx) + (dz * dz)));
        }
    }
    return best;
}

// Minimum clearance over an entire ROUTE (every waypoint-to-waypoint
// segment, densely sampled) - "is this route locally navigable," i.e.
// does it ever sit inside the same clearance radius the reactive
// avoidance layer (ForwardClearanceProbe) treats as hazardous.
float minRouteClearanceFromOccupied(const ExplorationMap& map, const std::vector<Vec3>& waypoints)
{
    float best = std::numeric_limits<float>::infinity();
    constexpr int kSamplesPerSegment = 60;
    for (std::size_t i = 1; i < waypoints.size(); ++i)
    {
        const Vec3& a = waypoints[i - 1];
        const Vec3& b = waypoints[i];
        for (int s = 0; s <= kSamplesPerSegment; ++s)
        {
            const float t = static_cast<float>(s) / static_cast<float>(kSamplesPerSegment);
            const Vec3 sample{a.x + ((b.x - a.x) * t), 0.0F, a.z + ((b.z - a.z) * t)};
            best = std::min(best, minDistanceToOccupiedFootprint(map, sample));
        }
    }
    return best;
}

// Phase 13X final blocker fix ("DOCK FINAL APPROACH" audit): clearance
// over every segment EXCEPT the final one. GridPathPlanner's own "goal
// snapping" class docs already document that the literal final segment
// into an exact requested goal point (e.g. BasePlatform's own position -
// intentionally close to the dock housing by explicit product design,
// see VirtualWorld.cpp's own kDockHousingZ derivation) is never validated
// against planning clearance at all: "the caller's own local steering...
// closes the exact last few centimeters via its own existing arrival-
// radius logic." Excluded here deliberately, not silently ignored - see
// DockApproachSegmentIsLocallyNavigable below for the (weaker, physical-
// collision-only) guarantee that final segment IS held to.
float minRouteClearanceExcludingFinalApproach(const ExplorationMap& map, const std::vector<Vec3>& waypoints)
{
    if (waypoints.size() < 2)
    {
        return std::numeric_limits<float>::infinity();
    }
    const std::vector<Vec3> withoutFinalSegment(waypoints.begin(), waypoints.end() - 1);
    return minRouteClearanceFromOccupied(map, withoutFinalSegment);
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

// ============================================================
// Phase 13X blocker fix (deadlock repair) - extraBlockedCells
// ============================================================
// GridPathPlanner's own constructor now accepts an optional list of
// additional cells to treat as not traversable, on top of whatever the
// map alone implies - see the constructor's own docs (GridPathPlanner.hpp)
// for why: ReactiveObstacleAvoidance's bounded TurnAway sweep can prove,
// via real sensor readings, that a specific cell the map calls Free is not
// actually passable (e.g. geometry a sparse sensor pass never crossed),
// and the caller (main3d.cpp) needs a way to route AROUND that cell on the
// very next replan without waiting for the map itself to somehow learn
// about it.

// --- ExtraBlocked 1: DefaultConstructorBehaviorUnchanged ---
TEST(GridPathPlannerTest, ExtraBlockedCellsDefaultsToEmptyAndBehaviorUnchanged)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    GridPathPlanner plannerDefault(map);
    GridPathPlanner plannerExplicitEmpty(map, {});

    const Vec3 start = map.cellToWorld(8, 8);
    const Vec3 goal = map.cellToWorld(30, 30);

    const PathPlanResult resultDefault = plannerDefault.planPath(start, goal);
    const PathPlanResult resultExplicitEmpty = plannerExplicitEmpty.planPath(start, goal);

    ASSERT_TRUE(resultDefault.success);
    ASSERT_TRUE(resultExplicitEmpty.success);
    EXPECT_EQ(resultDefault.cellPath.size(), resultExplicitEmpty.cellPath.size());
}

// --- ExtraBlocked 2: BlockedCellIsNeverTraversable ---
TEST(GridPathPlannerTest, ExtraBlockedCellIsNeverTraversable)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    const GridCoord blocked{20, 20};
    GridPathPlanner planner(map, std::vector<GridCoord>{blocked});

    EXPECT_TRUE(GridPathPlanner(map).isTraversable(blocked.col, blocked.row))
        << "sanity: the map alone considers this cell traversable";
    EXPECT_FALSE(planner.isTraversable(blocked.col, blocked.row));
}

// --- ExtraBlocked 3: RouteAvoidsExtraBlockedCells ---
TEST(GridPathPlannerTest, RouteAvoidsExtraBlockedCells)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);

    const Vec3 start = map.cellToWorld(8, 10);
    const Vec3 goal = map.cellToWorld(12, 30);

    // Block a partial wall on a row strictly between start and goal -
    // leaves both ends of the row open so a detour genuinely exists,
    // forcing any successful path to go around it rather than straight
    // through (start/goal themselves are off this row, so the start-cell
    // exemption - see GridPathPlanner's own docs - never interferes with
    // this check).
    std::vector<GridCoord> blocked;
    for (int col = 0; col < map.width() - 6; ++col)
    {
        blocked.push_back(GridCoord{col, 20});
    }
    GridPathPlanner planner(map, blocked);

    const PathPlanResult result = planner.planPath(start, goal);

    ASSERT_TRUE(result.success);
    for (const GridCoord& cell : result.cellPath)
    {
        for (const GridCoord& blockedCell : blocked)
        {
            EXPECT_FALSE(cell.col == blockedCell.col && cell.row == blockedCell.row)
                << "route crossed extra-blocked cell (" << blockedCell.col << ", " << blockedCell.row << ")";
        }
    }
    // Sanity: the wall genuinely forced a detour (the path actually visits
    // row 20 through the open gap on the far side, never a coincidence of
    // an unrelated, much shorter unconstrained route).
    bool usedRow20Gap = false;
    for (const GridCoord& cell : result.cellPath)
    {
        if (cell.row == 20 && cell.col >= map.width() - 6)
        {
            usedRow20Gap = true;
            break;
        }
    }
    EXPECT_TRUE(usedRow20Gap);
}

// --- ExtraBlocked 4: AllRoutesBlockedReportsFailure ---
TEST(GridPathPlannerTest, ExtraBlockedCellsCanMakeGoalUnreachable)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);

    const Vec3 start = map.cellToWorld(8, 20);
    const Vec3 goal = map.cellToWorld(32, 20);

    // Block an entire column spanning the whole map height - a genuine
    // wall no detour can get around.
    std::vector<GridCoord> blocked;
    for (int row = 0; row < map.height(); ++row)
    {
        blocked.push_back(GridCoord{20, row});
    }
    GridPathPlanner planner(map, blocked);

    const PathPlanResult result = planner.planPath(start, goal);

    EXPECT_FALSE(result.success);
}

// --- ExtraBlocked 5: FailedRouteExclusionProducesMeaningfullyDifferentReplan ---
//
// Phase 13X final blocker fix ("MAP-AWARE REPLAN" / "BLOCKED-CELL REPLAN
// AUDIT"): once the SPECIFIC corridor a route was using becomes excluded,
// the very next replan's route must genuinely change geometry - cross the
// wall through a DIFFERENT gap entirely - not merely shift by one grid
// cell within the same, now-excluded, corridor (the exact traced defect
// this phase fixed - see docs/technical-decisions.md).
TEST(GridPathPlannerTest, FailedRouteExclusionProducesMeaningfullyDifferentReplan)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    // Wall across row 20 with two widely-separated gaps, each wide enough
    // to stay traversable at this project's actual planning clearance
    // (kPlanningRadius, ~3.75 cells) - mirrors OccupiedWallForcesDetour's
    // own >= 9-cell gap convention above.
    markOccupied(map, 0, 7, 20, 20);   // wall segment 1
    // Gap A: cols 8-18 (open)
    markOccupied(map, 19, 26, 20, 20); // wall segment 2
    // Gap B: cols 27-37 (open)

    const Vec3 start = map.cellToWorld(13, 10); // above gap A
    const Vec3 goal = map.cellToWorld(13, 30);   // below gap A, same column

    GridPathPlanner unblocked(map);
    const PathPlanResult before = unblocked.planPath(start, goal);
    ASSERT_TRUE(before.success);

    int crossingCol = -1;
    for (const GridCoord& cell : before.cellPath)
    {
        if (cell.row == 20)
        {
            crossingCol = cell.col;
            break;
        }
    }
    ASSERT_NE(crossingCol, -1);
    EXPECT_LE(crossingCol, 18) << "sanity: the unblocked route uses the nearer gap A";

    // Exclude the ENTIRE gap A corridor (not just one cell) - the local-
    // hazard-proven "this whole corridor is not really passable" signal
    // main3d.cpp's own kLocalReplanExclusionRadiusCells neighborhood
    // exists to approximate on a real, sensor-driven map.
    std::vector<GridCoord> blockedCells;
    for (int col = 8; col <= 18; ++col)
    {
        blockedCells.push_back(GridCoord{col, 20});
    }
    GridPathPlanner blocked(map, blockedCells);
    const PathPlanResult after = blocked.planPath(start, goal);
    ASSERT_TRUE(after.success);

    int newCrossingCol = -1;
    for (const GridCoord& cell : after.cellPath)
    {
        if (cell.row == 20)
        {
            newCrossingCol = cell.col;
            break;
        }
    }
    ASSERT_NE(newCrossingCol, -1);
    EXPECT_GE(newCrossingCol, 27) << "forced onto the far gap B";
    EXPECT_GT(std::abs(newCrossingCol - crossingCol), 10)
        << "replan must genuinely change route geometry, not merely shift by one cell";
}

// --- ExtraBlocked 6: TemporaryBlockedCellsDoNotMutateExplorationMap ---
//
// Phase 13X final blocker fix ("TEMPORARY BLACKLIST SEMANTICS"):
// extraBlockedCells is a planner-local temporary constraint only -
// constructing a GridPathPlanner with it, and planning through it, must
// never mutate the underlying ExplorationMap's own Free/Occupied cell
// data. The occupancy map stays sensor-derived truth only - a local
// navigation failure (what forced this cell onto the blacklist) must
// never get silently recorded as a real, permanent obstacle.
TEST(GridPathPlannerTest, TemporaryBlockedCellsDoNotMutateExplorationMap)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    const GridCoord blocked{20, 20};
    ASSERT_EQ(map.cellAt(blocked.col, blocked.row), MapCell::Free);

    GridPathPlanner planner(map, std::vector<GridCoord>{blocked});
    const PathPlanResult result = planner.planPath(map.cellToWorld(5, 5), map.cellToWorld(35, 35));
    ASSERT_TRUE(result.success);

    EXPECT_EQ(map.cellAt(blocked.col, blocked.row), MapCell::Free)
        << "extraBlockedCells must never mutate the underlying ExplorationMap";
}

// ============================================================
// Phase 13X final blocker fix - global/local clearance-contract unification
// ============================================================
// The oscillation/deadlock this phase fixed was ultimately a mismatch
// between GridPathPlanner's own planning clearance and the local reactive
// layer's (ForwardClearanceProbe/ReactiveObstacleAvoidance's) hazard
// clearance - see NavigationClearance.hpp and
// docs/technical-decisions.md for the full traced reproduction. These
// tests assert the actual geometry contract, not merely that the fix
// "doesn't crash."

// --- ClearanceContract 1: PlannerClearanceCoversLocalCorridorRequirement ---
TEST(GridPathPlannerTest, PlannerClearanceCoversLocalCorridorRequirement)
{
    // The exact invariant this phase's whole fix exists to guarantee: a
    // route GridPathPlanner considers clear (kRobotCollisionRadius +
    // kPlanningSafetyMargin) must never be planned closer to an obstacle
    // than the local reactive layer's own hazard clearance
    // (NavigationClearance::kLocalHazardRadius, the exact radius
    // ForwardClearanceProbe's corridor test uses) - otherwise a globally
    // planned nominal path could sit inside the local hazard envelope,
    // the traced defect this phase fixed.
    EXPECT_GT(kRobotCollisionRadius + kPlanningSafetyMargin, kLocalHazardRadius)
        << "planning clearance must strictly exceed the local reactive-hazard clearance, "
           "with its own additional small planning-only margin on top";
    EXPECT_FLOAT_EQ(kRobotCollisionRadius + kPlanningSafetyMargin, kPlanningRadius);
}

// --- ClearanceContract 2: SimplifiedDiagonalDoesNotGrazeInflatedObstacle ---
//
// Direct reproduction of the traced defect (docs/technical-decisions.md):
// an L-shaped occupied corner, with start/goal positioned so the natural
// shortest route must pass diagonally near the corner - the exact
// "planner-safe-but-locally-hazardous corner cut" scenario the OLD
// point-distance-between-cell-centers clearance model missed (a diagonal
// simplified segment could pass within planning clearance of an
// obstacle's real square footprint while still measuring "far enough"
// from the nearest occupied cell's own CENTER point alone).
TEST(GridPathPlannerTest, SimplifiedDiagonalDoesNotGrazeInflatedObstacle)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    // Horizontal arm: row 20, cols 5-24. Vertical arm: col 24, rows 5-20.
    // Shared corner cell: (24, 20).
    markOccupied(map, 5, 24, 20, 20);
    markOccupied(map, 24, 24, 5, 20);
    GridPathPlanner planner(map);

    const Vec3 start = map.cellToWorld(6, 25);
    const Vec3 goal = map.cellToWorld(30, 8);
    const PathPlanResult result = planner.planPath(start, goal);
    ASSERT_TRUE(result.success);

    const float clearance = production::minRouteClearanceFromOccupied(map, result.waypoints);
    EXPECT_GE(clearance, kPlanningRadius - 1.0e-3F)
        << "a simplified segment passed closer to the corner's real footprint than planning clearance allows";
}

// --- ClearanceContract 3-5: ReturnHomeRouteAroundProduction{Keyboard,Mouse,MonitorStand}IsLocallyNavigable ---
//
// Over the REAL production desk geometry (see the `production` namespace
// above) - proves a planned route around each named object stays outside
// the SAME clearance radius the reactive avoidance layer treats as
// hazardous (NavigationClearance::kLocalHazardRadius), i.e. that
// GridPathPlanner's own route would never have triggered the traced
// oscillation for these specific, human-observed scenarios.
TEST(GridPathPlannerTest, ReturnHomeRouteAroundProductionKeyboardIsLocallyNavigable)
{
    ExplorationMap map(production::productionBounds());
    production::markProductionDeskFree(map);
    GridPathPlanner planner(map);

    const Vec3 start{-3.0F, 0.0F, -0.4F}; // west of the keyboard
    const Vec3 goal{1.3F, 0.0F, -1.5F};   // dock
    const PathPlanResult result = planner.planPath(start, goal);
    ASSERT_TRUE(result.success);

    EXPECT_GE(production::minRouteClearanceExcludingFinalApproach(map, result.waypoints),
              kLocalHazardRadius - 1.0e-3F);
}

TEST(GridPathPlannerTest, ReturnHomeRouteAroundProductionMouseIsLocallyNavigable)
{
    ExplorationMap map(production::productionBounds());
    production::markProductionDeskFree(map);
    GridPathPlanner planner(map);

    const Vec3 start{1.3F, 0.0F, 0.6F};   // north of the mouse
    const Vec3 goal{1.3F, 0.0F, -1.5F};   // dock
    const PathPlanResult result = planner.planPath(start, goal);
    ASSERT_TRUE(result.success);

    EXPECT_GE(production::minRouteClearanceExcludingFinalApproach(map, result.waypoints),
              kLocalHazardRadius - 1.0e-3F);
}

TEST(GridPathPlannerTest, ReturnHomeRouteAroundProductionMonitorStandIsLocallyNavigable)
{
    ExplorationMap map(production::productionBounds());
    production::markProductionDeskFree(map);
    GridPathPlanner planner(map);

    const Vec3 start{-1.2F, 0.0F, 1.5F}; // north of the monitor
    const Vec3 goal{1.3F, 0.0F, -1.5F};  // dock
    const PathPlanResult result = planner.planPath(start, goal);
    ASSERT_TRUE(result.success);

    EXPECT_GE(production::minRouteClearanceExcludingFinalApproach(map, result.waypoints),
              kLocalHazardRadius - 1.0e-3F);
}

// --- ClearanceContract 6: DockApproachSegmentIsLocallyNavigable ---
//
// Phase 13X final blocker fix ("DOCK FINAL APPROACH" audit): the ONE
// segment excluded from the three tests above - the literal final
// approach from the last A*-planned/inflated waypoint into BasePlatform's
// own exact position (see GridPathPlanner's own "goal snapping" class
// docs) - is INTENTIONALLY not held to the general local-hazard
// clearance (proven, empirically, by this exact test file: the dock
// housing sits close enough to BasePlatform's own position, by explicit
// product design, that NO route can maintain full
// NavigationClearance::kLocalHazardRadius clearance all the way to the
// literal dock point). It IS still held to the weaker, non-negotiable
// PHYSICAL collision guarantee (RobotCollision::kRobotCollisionRadius) -
// the final approach may be tight, but must never plan directly through
// the dock housing's own physical footprint.
TEST(GridPathPlannerTest, DockApproachSegmentIsLocallyNavigable)
{
    ExplorationMap map(production::productionBounds());
    production::markProductionDeskFree(map);
    GridPathPlanner planner(map);

    const Vec3 start{0.0F, 0.0F, 0.5F};
    const Vec3 dockGoal{1.3F, 0.0F, -1.5F};
    const PathPlanResult result = planner.planPath(start, dockGoal);
    ASSERT_TRUE(result.success);
    ASSERT_GE(result.waypoints.size(), 2U);

    const std::vector<Vec3> finalApproachSegment{result.waypoints[result.waypoints.size() - 2],
                                                  result.waypoints.back()};
    const float clearance = production::minRouteClearanceFromOccupied(map, finalApproachSegment);
    EXPECT_GE(clearance, kRobotCollisionRadius - 1.0e-3F)
        << "dock final-approach segment must never plan through the dock housing's own physical footprint";
}

} // namespace
