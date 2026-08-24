#include <gtest/gtest.h>

#include "robot/visual/CoverageTrail.hpp"

namespace
{
using robot::visual::CoverageTrail;
using robot::visual::RobotPose;
using robot::visual::Vec3;
} // namespace

// 1: StartsEmpty
TEST(CoverageTrailTest, StartsEmpty)
{
    CoverageTrail trail;
    EXPECT_TRUE(trail.points().empty());
}

// 2: FirstPoseRecorded
TEST(CoverageTrailTest, FirstPoseRecorded)
{
    CoverageTrail trail;
    trail.update(RobotPose{Vec3{1.0F, 0.0F, 2.0F}, 0.0F});

    ASSERT_EQ(trail.points().size(), 1U);
    EXPECT_FLOAT_EQ(trail.points()[0].x, 1.0F);
    EXPECT_FLOAT_EQ(trail.points()[0].z, 2.0F);
}

// 3: TinyMovementIgnored
TEST(CoverageTrailTest, TinyMovementIgnored)
{
    CoverageTrail trail;
    trail.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F});
    ASSERT_EQ(trail.points().size(), 1U);

    // Well under kTrailSampleDistanceWorldUnits (0.15F).
    trail.update(RobotPose{Vec3{0.02F, 0.0F, 0.0F}, 0.0F});
    EXPECT_EQ(trail.points().size(), 1U);
}

// 4: ThresholdMovementRecorded
TEST(CoverageTrailTest, ThresholdMovementRecorded)
{
    CoverageTrail trail;
    trail.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F});
    ASSERT_EQ(trail.points().size(), 1U);

    trail.update(RobotPose{Vec3{CoverageTrail::kTrailSampleDistanceWorldUnits, 0.0F, 0.0F}, 0.0F});
    EXPECT_EQ(trail.points().size(), 2U);
}

// 5: MultiplePointsPreserveOrder
TEST(CoverageTrailTest, MultiplePointsPreserveOrder)
{
    CoverageTrail trail;
    trail.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F});
    trail.update(RobotPose{Vec3{0.5F, 0.0F, 0.0F}, 0.0F});
    trail.update(RobotPose{Vec3{1.0F, 0.0F, 0.0F}, 0.0F});

    ASSERT_EQ(trail.points().size(), 3U);
    EXPECT_FLOAT_EQ(trail.points()[0].x, 0.0F);
    EXPECT_FLOAT_EQ(trail.points()[1].x, 0.5F);
    EXPECT_FLOAT_EQ(trail.points()[2].x, 1.0F);
}

// 6: TurningWithoutTranslationDoesNotSpam
TEST(CoverageTrailTest, TurningWithoutTranslationDoesNotSpam)
{
    CoverageTrail trail;
    trail.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F});
    ASSERT_EQ(trail.points().size(), 1U);

    // Position unchanged, only heading changes, across many update()
    // calls - must never append.
    for (float heading = 0.0F; heading < 360.0F; heading += 5.0F)
    {
        trail.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, heading});
    }
    EXPECT_EQ(trail.points().size(), 1U);
}

// 7: ClearWorks
TEST(CoverageTrailTest, ClearWorks)
{
    CoverageTrail trail;
    trail.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F});
    trail.update(RobotPose{Vec3{1.0F, 0.0F, 0.0F}, 0.0F});
    ASSERT_FALSE(trail.points().empty());

    trail.clear();
    EXPECT_TRUE(trail.points().empty());
}

// 8: TrailContinuesAcrossTaskChanges
//
// CoverageTrail has no MissionTask/task-lifecycle knowledge at all - this
// test documents that fact directly: interleaving update() calls exactly
// like Start Explore -> Stop Task -> Start Explore again would (main3d.cpp
// calls trail.update() unconditionally every frame regardless of task
// state) never resets anything, since there is no task-aware code path to
// trigger a reset in the first place.
TEST(CoverageTrailTest, TrailContinuesAcrossTaskChanges)
{
    CoverageTrail trail;
    trail.update(RobotPose{Vec3{0.0F, 0.0F, 0.0F}, 0.0F});    // "Start Explore"
    trail.update(RobotPose{Vec3{0.5F, 0.0F, 0.0F}, 0.0F});    // moving
    // "Stop Task" - no trail API call exists for this; nothing happens.
    trail.update(RobotPose{Vec3{1.0F, 0.0F, 0.0F}, 0.0F});    // "Start Explore" again, moving

    ASSERT_EQ(trail.points().size(), 3U);
    EXPECT_FLOAT_EQ(trail.points().front().x, 0.0F);
    EXPECT_FLOAT_EQ(trail.points().back().x, 1.0F);
}

// 9: ReturnHomeMovementIsRecorded
//
// CoverageTrail is driven purely by RobotPose, so Return Home movement
// (or Safety recovery, or avoidance, or manual drive) is recorded
// identically to Explore movement - there is no separate "which policy is
// driving" input to this class at all.
TEST(CoverageTrailTest, ReturnHomeMovementIsRecorded)
{
    CoverageTrail trail;
    trail.update(RobotPose{Vec3{-3.0F, 0.0F, 1.0F}, 0.0F}); // Explore start
    trail.update(RobotPose{Vec3{-2.0F, 0.0F, 1.0F}, 0.0F}); // Explore movement
    trail.update(RobotPose{Vec3{0.0F, 0.0F, 2.0F}, 45.0F}); // Return Home movement
    trail.update(RobotPose{Vec3{4.0F, 0.0F, 4.0F}, 45.0F}); // arriving at base

    EXPECT_GE(trail.points().size(), 4U);
    EXPECT_FLOAT_EQ(trail.points().back().x, 4.0F);
    EXPECT_FLOAT_EQ(trail.points().back().z, 4.0F);
}
