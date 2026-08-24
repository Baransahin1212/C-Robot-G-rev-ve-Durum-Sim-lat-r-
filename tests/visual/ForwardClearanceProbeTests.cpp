#include <gtest/gtest.h>

#include "robot/visual/ForwardClearanceProbe.hpp"
#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace
{

using robot::visual::ForwardClearanceProbe;
using robot::visual::kRobotCollisionRadius;
using robot::visual::Vec3;
using robot::visual::VirtualWorld;

// Same convention every other visual-simulator test file uses: disable
// every default demo obstacle so a test can place/enable exactly one (or
// two) controlled obstacles of its own, with expected results computed
// from known, simple numbers instead of the demo scene's arbitrary
// coordinates.
void disableAllObstacles(VirtualWorld& world)
{
    for (std::size_t i = 0; i < world.obstacles().size(); ++i)
    {
        world.setObstacleEnabled(i, false);
    }
}

// clearanceRadius = kRobotCollisionRadius (0.5F exactly, for this
// project's robot dimensions) + ForwardClearanceProbe::kSafetyMargin.
const float kClearanceRadius = kRobotCollisionRadius + ForwardClearanceProbe::kSafetyMargin;

} // namespace

// 1: EmptyWorldIsClear
TEST(ForwardClearanceProbeTest, EmptyWorldIsClear)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    ForwardClearanceProbe probe(world);

    EXPECT_TRUE(probe.isForwardCorridorClear());
}

// 2: StraightObstacleBlocksCorridor
TEST(ForwardClearanceProbeTest, StraightObstacleBlocksCorridor)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 1.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    ForwardClearanceProbe probe(world);

    EXPECT_FALSE(probe.isForwardCorridorClear());
}

// 3: DisabledObstacleIsIgnored
TEST(ForwardClearanceProbeTest, DisabledObstacleIsIgnored)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 1.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    // Deliberately left disabled (disableAllObstacles() above).
    ForwardClearanceProbe probe(world);

    EXPECT_TRUE(probe.isForwardCorridorClear());
}

// 4: ObstacleBehindRobotDoesNotBlock
TEST(ForwardClearanceProbeTest, ObstacleBehindRobotDoesNotBlock)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, -2.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    ForwardClearanceProbe probe(world);

    EXPECT_TRUE(probe.isForwardCorridorClear());
}

// 5: ObstacleBeyondLookaheadDoesNotBlock
TEST(ForwardClearanceProbeTest, ObstacleBeyondLookaheadDoesNotBlock)
{
    // Near-face-expanded Z: 3.0 - 0.4 - clearanceRadius, still well beyond
    // kLookaheadDistance (1.4F).
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 3.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    ForwardClearanceProbe probe(world);

    ASSERT_LT(ForwardClearanceProbe::kLookaheadDistance, 3.0F - 0.4F - kClearanceRadius);
    EXPECT_TRUE(probe.isForwardCorridorClear());
}

// 6: ObstacleBesideNarrowPointRayButInsideBodyWidthBlocks
TEST(ForwardClearanceProbeTest, ObstacleBesideNarrowPointRayButInsideBodyWidthBlocks)
{
    // Obstacle's raw X footprint (0.3 to 1.1) does NOT contain the
    // centerline X (0.0) - a narrow point ray straight down X 0.0 would
    // miss it entirely - but the clearance-radius-expanded footprint
    // (~-0.13 to ~1.53, expanded by kRobotCollisionRadius + kSafetyMargin
    // =~ 0.43F for this project's rescaled robot) does, since it must
    // account for the robot's full body width, not just its centerline.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.7F, 0.4F, 1.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    ForwardClearanceProbe probe(world);

    ASSERT_GT(0.7F - 0.4F, 0.0F); // raw footprint excludes X 0.0
    EXPECT_FALSE(probe.isForwardCorridorClear());
}

// 7: ObstacleOutsideBodyClearanceDoesNotBlock
TEST(ForwardClearanceProbeTest, ObstacleOutsideBodyClearanceDoesNotBlock)
{
    // Far enough in X that even the clearance-radius-expanded footprint
    // still excludes the centerline.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{2.0F, 0.4F, 1.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    ForwardClearanceProbe probe(world);

    ASSERT_GT(2.0F - 0.4F - kClearanceRadius, 0.0F);
    EXPECT_TRUE(probe.isForwardCorridorClear());
}

// 8: HeadingZeroUsesPositiveZ
TEST(ForwardClearanceProbeTest, HeadingZeroUsesPositiveZ)
{
    VirtualWorld worldAhead;
    disableAllObstacles(worldAhead);
    worldAhead.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    worldAhead.setRobotHeading(0.0F);
    worldAhead.setObstaclePosition(0, Vec3{0.0F, 0.4F, 1.0F});
    worldAhead.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    worldAhead.setObstacleEnabled(0, true);
    ForwardClearanceProbe probeAhead(worldAhead);
    EXPECT_FALSE(probeAhead.isForwardCorridorClear());

    VirtualWorld worldBehind;
    disableAllObstacles(worldBehind);
    worldBehind.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    worldBehind.setRobotHeading(0.0F);
    worldBehind.setObstaclePosition(0, Vec3{0.0F, 0.4F, -1.0F});
    worldBehind.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    worldBehind.setObstacleEnabled(0, true);
    ForwardClearanceProbe probeBehind(worldBehind);
    EXPECT_TRUE(probeBehind.isForwardCorridorClear());
}

// 9: HeadingNinetyUsesPositiveX
TEST(ForwardClearanceProbeTest, HeadingNinetyUsesPositiveX)
{
    VirtualWorld worldAhead;
    disableAllObstacles(worldAhead);
    worldAhead.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    worldAhead.setRobotHeading(90.0F);
    worldAhead.setObstaclePosition(0, Vec3{1.0F, 0.4F, 0.0F});
    worldAhead.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    worldAhead.setObstacleEnabled(0, true);
    ForwardClearanceProbe probeAhead(worldAhead);
    EXPECT_FALSE(probeAhead.isForwardCorridorClear());

    VirtualWorld worldBehind;
    disableAllObstacles(worldBehind);
    worldBehind.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    worldBehind.setRobotHeading(90.0F);
    worldBehind.setObstaclePosition(0, Vec3{-1.0F, 0.4F, 0.0F});
    worldBehind.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    worldBehind.setObstacleEnabled(0, true);
    ForwardClearanceProbe probeBehind(worldBehind);
    EXPECT_TRUE(probeBehind.isForwardCorridorClear());
}

// 10: HeadingMinusNinetyUsesNegativeX
TEST(ForwardClearanceProbeTest, HeadingMinusNinetyUsesNegativeX)
{
    VirtualWorld worldAhead;
    disableAllObstacles(worldAhead);
    worldAhead.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    worldAhead.setRobotHeading(-90.0F);
    worldAhead.setObstaclePosition(0, Vec3{-1.0F, 0.4F, 0.0F});
    worldAhead.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    worldAhead.setObstacleEnabled(0, true);
    ForwardClearanceProbe probeAhead(worldAhead);
    EXPECT_FALSE(probeAhead.isForwardCorridorClear());

    VirtualWorld worldBehind;
    disableAllObstacles(worldBehind);
    worldBehind.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    worldBehind.setRobotHeading(-90.0F);
    worldBehind.setObstaclePosition(0, Vec3{1.0F, 0.4F, 0.0F});
    worldBehind.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    worldBehind.setObstacleEnabled(0, true);
    ForwardClearanceProbe probeBehind(worldBehind);
    EXPECT_TRUE(probeBehind.isForwardCorridorClear());
}

// 11: HeadingOneEightyUsesNegativeZ
TEST(ForwardClearanceProbeTest, HeadingOneEightyUsesNegativeZ)
{
    VirtualWorld worldAhead;
    disableAllObstacles(worldAhead);
    worldAhead.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    worldAhead.setRobotHeading(180.0F);
    worldAhead.setObstaclePosition(0, Vec3{0.0F, 0.4F, -1.0F});
    worldAhead.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    worldAhead.setObstacleEnabled(0, true);
    ForwardClearanceProbe probeAhead(worldAhead);
    EXPECT_FALSE(probeAhead.isForwardCorridorClear());

    VirtualWorld worldBehind;
    disableAllObstacles(worldBehind);
    worldBehind.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    worldBehind.setRobotHeading(180.0F);
    worldBehind.setObstaclePosition(0, Vec3{0.0F, 0.4F, 1.0F});
    worldBehind.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    worldBehind.setObstacleEnabled(0, true);
    ForwardClearanceProbe probeBehind(worldBehind);
    EXPECT_TRUE(probeBehind.isForwardCorridorClear());
}

// 12: ParallelSegmentHandledWithoutDivisionByZero
TEST(ForwardClearanceProbeTest, ParallelSegmentHandledWithoutDivisionByZero)
{
    // Heading 0 makes direction.x exactly 0.0 (parallel to the X slab).
    // Obstacle X range excludes the robot's X, so the parallel-axis
    // special case must resolve cleanly (no crash/NaN) to "clear," and
    // repeated calls must agree.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{5.0F, 0.4F, 1.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    ForwardClearanceProbe probe(world);

    const bool first = probe.isForwardCorridorClear();
    const bool second = probe.isForwardCorridorClear();
    EXPECT_TRUE(first);
    EXPECT_EQ(first, second);
}

// 13: BoundaryContactHandledDeterministically
TEST(ForwardClearanceProbeTest, BoundaryContactHandledDeterministically)
{
    // Near-face-expanded Z lands exactly at kLookaheadDistance: obstacle
    // half-depth 0.4, so position.z = kLookaheadDistance + 0.4 +
    // clearanceRadius.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    const float boundaryZ = ForwardClearanceProbe::kLookaheadDistance + 0.4F + kClearanceRadius;
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, boundaryZ});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    ForwardClearanceProbe probe(world);

    const bool first = probe.isForwardCorridorClear();
    const bool second = probe.isForwardCorridorClear();
    EXPECT_EQ(first, second);
    // Exact tangent contact counts as blocked - consistent with this
    // project's existing sensor/collision boundary convention.
    EXPECT_FALSE(first);
}

// 14: NearestOfMultipleObstaclesBlocks
TEST(ForwardClearanceProbeTest, NearestOfMultipleObstaclesBlocks)
{
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, 10.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F}); // far - beyond lookahead
    world.setObstacleEnabled(0, true);
    world.setObstaclePosition(1, Vec3{0.0F, 0.4F, 1.0F});
    world.setObstacleSize(1, Vec3{0.8F, 0.8F, 0.8F}); // near - blocks
    world.setObstacleEnabled(1, true);
    ForwardClearanceProbe probe(world);

    EXPECT_FALSE(probe.isForwardCorridorClear());
}

// 15: FacingAwayFromNearbyObstacleCanBeClear
TEST(ForwardClearanceProbeTest, FacingAwayFromNearbyObstacleCanBeClear)
{
    // Obstacle behind the robot's current heading, far enough back that
    // even its clearance-radius-expanded footprint (near/expanded face at
    // Z -0.52) does not reach forward past the robot's center (segment
    // start, Z 0) - the forward corridor is clear. (Any closer than
    // clearanceRadius + halfDepth behind would mean the robot's OWN
    // collision circle already overlaps the obstacle - an invalid,
    // already-penetrating starting position RobotCollision would never
    // allow to begin with, not a meaningful "facing away" case.)
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    world.setObstaclePosition(0, Vec3{0.0F, 0.4F, -1.5F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    ForwardClearanceProbe probe(world);

    ASSERT_LT(-1.5F + 0.4F + kClearanceRadius, 0.0F); // expanded footprint stays behind Z 0
    EXPECT_TRUE(probe.isForwardCorridorClear());
}

// 16: SafetyMarginIsRespected
TEST(ForwardClearanceProbeTest, SafetyMarginIsRespected)
{
    // Positioned so the obstacle's raw AABB expanded by
    // kRobotCollisionRadius ALONE would just clear the centerline (X
    // 0.04 > 0), but expanded by the full clearanceRadius (radius +
    // kSafetyMargin) reaches it (X -0.04 <= 0) - this placement is only
    // caught because the safety margin is actually added on top of the
    // bare collision radius.
    VirtualWorld world;
    disableAllObstacles(world);
    world.setRobotPosition(Vec3{0.0F, 0.125F, 0.0F});
    world.setRobotHeading(0.0F);
    const float obstacleX = kRobotCollisionRadius + 0.4F + (ForwardClearanceProbe::kSafetyMargin / 2.0F);
    world.setObstaclePosition(0, Vec3{obstacleX, 0.4F, 1.0F});
    world.setObstacleSize(0, Vec3{0.8F, 0.8F, 0.8F});
    world.setObstacleEnabled(0, true);
    ForwardClearanceProbe probe(world);

    ASSERT_GT(obstacleX - 0.4F - kRobotCollisionRadius, 0.0F);           // radius alone would miss
    ASSERT_LE(obstacleX - 0.4F - kClearanceRadius, 0.0F);                // radius + margin reaches
    EXPECT_FALSE(probe.isForwardCorridorClear());
}
