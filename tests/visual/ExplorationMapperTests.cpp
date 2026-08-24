#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include "robot/visual/ExplorationMap.hpp"
#include "robot/visual/ExplorationMapper.hpp"
#include "robot/visual/RangeObservation.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::visual::ExplorationMap;
using robot::visual::ExplorationMapper;
using robot::visual::MapCell;
using robot::visual::RangeObservation;
using robot::visual::RobotPose;
using robot::visual::TableSurface;
using robot::visual::Vec3;
using robot::visual::VirtualWorld;

TableSurface demoTable()
{
    return TableSurface{-6.0F, 6.0F, -6.0F, 6.0F};
}

RangeObservation makeObservation(Vec3 origin, Vec3 direction, float distance, float maxRange, bool hit)
{
    RangeObservation observation;
    observation.origin = origin;
    observation.direction = direction;
    observation.distance = distance;
    observation.maxRange = maxRange;
    observation.hit = hit;
    return observation;
}

} // namespace

// 1: NoHitMarksRayFree
TEST(ExplorationMapperTest, NoHitMarksRayFree)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    const RangeObservation observation = makeObservation(Vec3{0.0F, 0.0F, 0.0F}, Vec3{0.0F, 0.0F, 1.0F}, 2.5F, 2.5F, false);
    mapper.update(RobotPose{Vec3{-10.0F, 0.0F, -10.0F}, 0.0F}, std::vector<RangeObservation>{observation});

    int col = -1;
    int row = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 2.0F}, col, row));
    EXPECT_EQ(map.cellAt(col, row), MapCell::Free);

    int endCol = -1;
    int endRow = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 2.49F}, endCol, endRow));
    EXPECT_NE(map.cellAt(endCol, endRow), MapCell::Occupied);
}

// 2: HitMarksTraversedCellsFree
TEST(ExplorationMapperTest, HitMarksTraversedCellsFree)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    const RangeObservation observation = makeObservation(Vec3{0.0F, 0.0F, 0.0F}, Vec3{0.0F, 0.0F, 1.0F}, 1.0F, 2.5F, true);
    mapper.update(RobotPose{Vec3{-10.0F, 0.0F, -10.0F}, 0.0F}, std::vector<RangeObservation>{observation});

    int col = -1;
    int row = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 0.5F}, col, row));
    EXPECT_EQ(map.cellAt(col, row), MapCell::Free);
}

// 3: HitMarksEndpointOccupied
TEST(ExplorationMapperTest, HitMarksEndpointOccupied)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    const RangeObservation observation = makeObservation(Vec3{0.0F, 0.0F, 0.0F}, Vec3{0.0F, 0.0F, 1.0F}, 1.0F, 2.5F, true);
    mapper.update(RobotPose{Vec3{-10.0F, 0.0F, -10.0F}, 0.0F}, std::vector<RangeObservation>{observation});

    int col = -1;
    int row = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 1.0F}, col, row));
    EXPECT_EQ(map.cellAt(col, row), MapCell::Occupied);
}

// 4: HeadingZeroMapsPositiveZ
TEST(ExplorationMapperTest, HeadingZeroMapsPositiveZ)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    // Heading 0 -> forwardDirection() == (0, 0, 1): a hit straight ahead
    // must land at a HIGHER row (increasing Z), same column.
    const RangeObservation observation = makeObservation(Vec3{0.0F, 0.0F, 0.0F}, Vec3{0.0F, 0.0F, 1.0F}, 1.2F, 2.5F, true);
    mapper.update(RobotPose{Vec3{-10.0F, 0.0F, -10.0F}, 0.0F}, std::vector<RangeObservation>{observation});

    int originCol = -1;
    int originRow = -1;
    int hitCol = -1;
    int hitRow = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 0.0F}, originCol, originRow));
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 1.2F}, hitCol, hitRow));
    EXPECT_EQ(hitCol, originCol);
    EXPECT_GT(hitRow, originRow);
    EXPECT_EQ(map.cellAt(hitCol, hitRow), MapCell::Occupied);
}

// 5: HeadingNinetyMapsPositiveX
TEST(ExplorationMapperTest, HeadingNinetyMapsPositiveX)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    // Heading 90 -> forwardDirection() == (1, 0, 0): a hit straight ahead
    // must land at a HIGHER column (increasing X), same row.
    const RangeObservation observation = makeObservation(Vec3{0.0F, 0.0F, 0.0F}, Vec3{1.0F, 0.0F, 0.0F}, 1.2F, 2.5F, true);
    mapper.update(RobotPose{Vec3{-10.0F, 0.0F, -10.0F}, 90.0F}, std::vector<RangeObservation>{observation});

    int originCol = -1;
    int originRow = -1;
    int hitCol = -1;
    int hitRow = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 0.0F}, originCol, originRow));
    ASSERT_TRUE(map.worldToCell(Vec3{1.2F, 0.0F, 0.0F}, hitCol, hitRow));
    EXPECT_GT(hitCol, originCol);
    EXPECT_EQ(hitRow, originRow);
    EXPECT_EQ(map.cellAt(hitCol, hitRow), MapCell::Occupied);
}

// 6: LeftObservationMapsLeft
TEST(ExplorationMapperTest, LeftObservationMapsLeft)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    // Heading 0: rightDirection() == (1, 0, 0), so "left" is -X.
    const RangeObservation leftObservation =
        makeObservation(Vec3{-0.3F, 0.0F, 0.0F}, Vec3{0.0F, 0.0F, 1.0F}, 1.0F, 2.5F, true);
    mapper.update(RobotPose{Vec3{-10.0F, 0.0F, -10.0F}, 0.0F}, std::vector<RangeObservation>{leftObservation});

    int centerCol = -1;
    int centerRow = -1;
    int leftCol = -1;
    int leftRow = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 0.0F}, centerCol, centerRow));
    ASSERT_TRUE(map.worldToCell(Vec3{-0.3F, 0.0F, 1.0F}, leftCol, leftRow));
    EXPECT_LT(leftCol, centerCol);
    EXPECT_EQ(map.cellAt(leftCol, leftRow), MapCell::Occupied);
}

// 7: RightObservationMapsRight
TEST(ExplorationMapperTest, RightObservationMapsRight)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    // Heading 0: rightDirection() == (1, 0, 0), so "right" is +X.
    const RangeObservation rightObservation =
        makeObservation(Vec3{0.3F, 0.0F, 0.0F}, Vec3{0.0F, 0.0F, 1.0F}, 1.0F, 2.5F, true);
    mapper.update(RobotPose{Vec3{-10.0F, 0.0F, -10.0F}, 0.0F}, std::vector<RangeObservation>{rightObservation});

    int centerCol = -1;
    int centerRow = -1;
    int rightCol = -1;
    int rightRow = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 0.0F}, centerCol, centerRow));
    ASSERT_TRUE(map.worldToCell(Vec3{0.3F, 0.0F, 1.0F}, rightCol, rightRow));
    EXPECT_GT(rightCol, centerCol);
    EXPECT_EQ(map.cellAt(rightCol, rightRow), MapCell::Occupied);
}

// 8: UnknownOutsideObservationRemainsUnknown
TEST(ExplorationMapperTest, UnknownOutsideObservationRemainsUnknown)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    const RangeObservation observation = makeObservation(Vec3{0.0F, 0.0F, 0.0F}, Vec3{0.0F, 0.0F, 1.0F}, 1.0F, 2.5F, true);
    mapper.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F}, std::vector<RangeObservation>{observation});

    // Far corner of the table, nowhere near the ray or the robot's
    // footprint - must remain Unknown.
    int col = -1;
    int row = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{-5.5F, 0.0F, -5.5F}, col, row));
    EXPECT_EQ(map.cellAt(col, row), MapCell::Unknown);
}

// 9: RobotFootprintMarkedFree
TEST(ExplorationMapperTest, RobotFootprintMarkedFree)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    // No observations at all - only the footprint pass should run.
    mapper.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F}, std::vector<RangeObservation>{});

    int col = -1;
    int row = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, 0.0F}, col, row));
    EXPECT_EQ(map.cellAt(col, row), MapCell::Free);
    EXPECT_GT(map.exploredCellCount(), 0U);

    // The footprint must stay small - not "a huge circle" (Phase 13V
    // brief) - a cell several world units away must not be touched by
    // it.
    int farCol = -1;
    int farRow = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{3.0F, 0.0F, 3.0F}, farCol, farRow));
    EXPECT_EQ(map.cellAt(farCol, farRow), MapCell::Unknown);
}

// 10: RepeatedObservationIsIdempotent
TEST(ExplorationMapperTest, RepeatedObservationIsIdempotent)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    const RobotPose pose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F};
    const RangeObservation observation = makeObservation(Vec3{0.0F, 0.0F, 0.0F}, Vec3{0.0F, 0.0F, 1.0F}, 1.0F, 2.5F, true);
    const std::vector<RangeObservation> observations{observation};

    mapper.update(pose, observations);
    const std::vector<MapCell> afterFirst = map.cells();

    mapper.update(pose, observations);
    const std::vector<MapCell> afterSecond = map.cells();

    EXPECT_EQ(afterFirst, afterSecond);
}

// 11: ObstacleBehindRobotNotRevealed
TEST(ExplorationMapperTest, ObstacleBehindRobotNotRevealed)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    // Only a forward-facing observation is ever given to update() - a
    // cell directly BEHIND the robot (negative Z at heading 0) must
    // remain Unknown, since no ray or footprint ever reaches it.
    const RangeObservation observation = makeObservation(Vec3{0.0F, 0.0F, 0.0F}, Vec3{0.0F, 0.0F, 1.0F}, 1.0F, 2.5F, true);
    mapper.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F}, std::vector<RangeObservation>{observation});

    int behindCol = -1;
    int behindRow = -1;
    ASSERT_TRUE(map.worldToCell(Vec3{0.0F, 0.0F, -2.0F}, behindCol, behindRow));
    EXPECT_EQ(map.cellAt(behindCol, behindRow), MapCell::Unknown);
}

// 12: DisabledObstacleDoesNotProduceOccupiedHit
//
// A disabled obstacle is invisible to VirtualObstacleSensorArray's own
// ray-vs-AABB test (see VirtualObstacleSensorArray.cpp - disabled
// obstacles are skipped entirely), so production code only ever hands
// ExplorationMapper a `hit = false` RangeObservation in that case - this
// test exercises exactly that shape directly, without needing a
// VirtualWorld/BoxObstacle at all, and confirms no cell along the ray
// becomes Occupied.
TEST(ExplorationMapperTest, DisabledObstacleDoesNotProduceOccupiedHit)
{
    ExplorationMap map(demoTable());
    ExplorationMapper mapper(map);

    const RangeObservation observation = makeObservation(Vec3{0.0F, 0.0F, 0.0F}, Vec3{0.0F, 0.0F, 1.0F}, 2.5F, 2.5F, false);
    mapper.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F}, std::vector<RangeObservation>{observation});

    for (MapCell cell : map.cells())
    {
        EXPECT_NE(cell, MapCell::Occupied);
    }
}

// 13: MapperDoesNotRequireVirtualWorldReference
//
// Compile-time proof, not just a runtime check: ExplorationMapper::update()
// is NOT invocable with a VirtualWorld argument in any position - its only
// accepted shape is (const RobotPose&, const std::vector<RangeObservation>&).
// This is the architectural guarantee behind the whole phase's "no
// cheating" rule - ExplorationMapper is literally incapable of reading
// VirtualWorld::obstacles() directly, since it never receives a
// VirtualWorld reference at all.
TEST(ExplorationMapperTest, MapperDoesNotRequireVirtualWorldReference)
{
    static_assert(std::is_invocable_v<decltype(&ExplorationMapper::update), ExplorationMapper&, const RobotPose&,
                                       const std::vector<RangeObservation>&>,
                  "update() must accept (RobotPose, observations)");
    static_assert(!std::is_invocable_v<decltype(&ExplorationMapper::update), ExplorationMapper&, const VirtualWorld&,
                                        const std::vector<RangeObservation>&>,
                  "update() must never accept a VirtualWorld reference");
    SUCCEED();
}
