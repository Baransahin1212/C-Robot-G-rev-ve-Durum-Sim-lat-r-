#include <gtest/gtest.h>

#include "robot/visual/ExplorationMap.hpp"

namespace
{

using robot::visual::ExplorationMap;
using robot::visual::MapCell;
using robot::visual::TableSurface;
using robot::visual::Vec3;

// Matches VirtualWorld.cpp's actual demo table (kTableHalfExtent = 6.0F)
// - 12x12 world units -> exactly 100x100 cells at
// ExplorationMap::kCellSizeWorldUnits (0.12F), per ExplorationMap.hpp's
// own derivation.
TableSurface demoTable()
{
    return TableSurface{-6.0F, 6.0F, -6.0F, 6.0F};
}

} // namespace

// 1: AllCellsStartUnknown
TEST(ExplorationMapTest, AllCellsStartUnknown)
{
    ExplorationMap map(demoTable());

    for (MapCell cell : map.cells())
    {
        EXPECT_EQ(cell, MapCell::Unknown);
    }
    EXPECT_EQ(map.exploredCellCount(), 0U);
}

// 2: WorldCenterConvertsToValidCell
TEST(ExplorationMapTest, WorldCenterConvertsToValidCell)
{
    ExplorationMap map(demoTable());

    int col = -1;
    int row = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 0.0F}, col, row));
    EXPECT_TRUE(map.isInsideGrid(col, row));
    EXPECT_EQ(col, map.width() / 2);
    EXPECT_EQ(row, map.height() / 2);
}

// 3: WorldEdgesConvertDeterministically
TEST(ExplorationMapTest, WorldEdgesConvertDeterministically)
{
    ExplorationMap map(demoTable());

    int col = -1;
    int row = -1;
    // Near-min corner (inside the half-open [min, max) interval).
    ASSERT_TRUE(map.worldToCell(Vec3{-5.999F, 0.0F, -5.999F}, col, row));
    EXPECT_EQ(col, 0);
    EXPECT_EQ(row, 0);

    // Near-max corner - the last valid cell, one cell size inside maxX/maxZ.
    ASSERT_TRUE(map.worldToCell(Vec3{5.999F, 0.0F, 5.999F}, col, row));
    EXPECT_EQ(col, map.width() - 1);
    EXPECT_EQ(row, map.height() - 1);
}

// 4: OutOfBoundsRejected
TEST(ExplorationMapTest, OutOfBoundsRejected)
{
    ExplorationMap map(demoTable());

    int col = -1;
    int row = -1;
    EXPECT_FALSE(map.worldToCell(Vec3{100.0F, 0.0F, 0.0F}, col, row));
    EXPECT_FALSE(map.worldToCell(Vec3{0.0F, 0.0F, -100.0F}, col, row));
    // Exactly at maxX/maxZ is outside the half-open [min, max) interval.
    EXPECT_FALSE(map.worldToCell(Vec3{6.0F, 0.0F, 0.0F}, col, row));
    EXPECT_FALSE(map.worldToCell(Vec3{0.0F, 0.0F, 6.0F}, col, row));

    EXPECT_FALSE(map.isInsideGrid(-1, 0));
    EXPECT_FALSE(map.isInsideGrid(0, -1));
    EXPECT_FALSE(map.isInsideGrid(map.width(), 0));
    EXPECT_FALSE(map.isInsideGrid(0, map.height()));
    EXPECT_EQ(map.cellAt(-1, 0), MapCell::Unknown);
    EXPECT_EQ(map.cellAt(map.width(), 0), MapCell::Unknown);
}

// 5: CellToWorldRoundTrip
TEST(ExplorationMapTest, CellToWorldRoundTrip)
{
    ExplorationMap map(demoTable());

    for (int row = 0; row < map.height(); row += 7)
    {
        for (int col = 0; col < map.width(); col += 7)
        {
            const Vec3 center = map.cellToWorld(col, row);
            int roundTripCol = -1;
            int roundTripRow = -1;
            ASSERT_TRUE(map.worldToCell(center, roundTripCol, roundTripRow));
            EXPECT_EQ(roundTripCol, col);
            EXPECT_EQ(roundTripRow, row);
        }
    }
}

// 6: MarkFree
TEST(ExplorationMapTest, MarkFree)
{
    ExplorationMap map(demoTable());

    map.markFree(10, 10);
    EXPECT_EQ(map.cellAt(10, 10), MapCell::Free);
    EXPECT_EQ(map.exploredCellCount(), 1U);

    // Out-of-range is a silent no-op, never UB.
    map.markFree(-1, -1);
    map.markFree(map.width(), map.height());
    EXPECT_EQ(map.exploredCellCount(), 1U);
}

// 7: MarkOccupied
TEST(ExplorationMapTest, MarkOccupied)
{
    ExplorationMap map(demoTable());

    map.markOccupied(20, 20);
    EXPECT_EQ(map.cellAt(20, 20), MapCell::Occupied);
    EXPECT_EQ(map.exploredCellCount(), 1U);
}

// 8: OccupiedHasCorrectPrecedence
TEST(ExplorationMapTest, OccupiedHasCorrectPrecedence)
{
    ExplorationMap map(demoTable());

    map.markOccupied(5, 5);
    ASSERT_EQ(map.cellAt(5, 5), MapCell::Occupied);

    // A later markFree() on the same cell must never downgrade it.
    map.markFree(5, 5);
    EXPECT_EQ(map.cellAt(5, 5), MapCell::Occupied);

    // The reverse direction is allowed: Unknown -> Free -> Occupied.
    map.markFree(6, 6);
    ASSERT_EQ(map.cellAt(6, 6), MapCell::Free);
    map.markOccupied(6, 6);
    EXPECT_EQ(map.cellAt(6, 6), MapCell::Occupied);
}

// 9: ExploredCellCount
TEST(ExplorationMapTest, ExploredCellCount)
{
    ExplorationMap map(demoTable());

    map.markFree(0, 0);
    map.markFree(1, 0);
    map.markOccupied(2, 0);
    EXPECT_EQ(map.exploredCellCount(), 3U);

    // Marking the same cell again must not double-count.
    map.markFree(0, 0);
    EXPECT_EQ(map.exploredCellCount(), 3U);
}

// 10: ExploredPercentage
TEST(ExplorationMapTest, ExploredPercentage)
{
    ExplorationMap map(demoTable());

    EXPECT_FLOAT_EQ(map.exploredPercentage(), 0.0F);

    const std::size_t total = map.totalCellCount();
    map.markFree(0, 0);
    const float expected = 100.0F * (1.0F / static_cast<float>(total));
    EXPECT_NEAR(map.exploredPercentage(), expected, 0.001F);
}

// 11: GridDimensionsMatchTableBounds
TEST(ExplorationMapTest, GridDimensionsMatchTableBounds)
{
    ExplorationMap map(demoTable());

    // 12 world units / 0.12 world units per cell = 100 cells exactly.
    EXPECT_EQ(map.width(), 100);
    EXPECT_EQ(map.height(), 100);
    EXPECT_FLOAT_EQ(map.cellSize(), ExplorationMap::kCellSizeWorldUnits);
    EXPECT_EQ(map.totalCellCount(), static_cast<std::size_t>(map.width()) * static_cast<std::size_t>(map.height()));

    // A differently-sized table still produces a correctly-derived grid,
    // never a hardcoded 100x100.
    ExplorationMap smallerMap(TableSurface{-1.0F, 1.0F, -1.0F, 1.0F});
    EXPECT_EQ(smallerMap.width(), static_cast<int>((2.0F / ExplorationMap::kCellSizeWorldUnits) + 0.5F));
}

// 12: XZAxesNotMirrored
TEST(ExplorationMapTest, XZAxesNotMirrored)
{
    ExplorationMap map(demoTable());

    // Moving along +X only must change column, never row.
    int baseCol = -1;
    int baseRow = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 0.0F}, baseCol, baseRow));

    int xMovedCol = -1;
    int xMovedRow = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{1.2F, 0.0F, 0.0F}, xMovedCol, xMovedRow));
    EXPECT_GT(xMovedCol, baseCol);
    EXPECT_EQ(xMovedRow, baseRow);

    // Moving along +Z only must change row, never column.
    int zMovedCol = -1;
    int zMovedRow = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 1.2F}, zMovedCol, zMovedRow));
    EXPECT_EQ(zMovedCol, baseCol);
    EXPECT_GT(zMovedRow, baseRow);
}
