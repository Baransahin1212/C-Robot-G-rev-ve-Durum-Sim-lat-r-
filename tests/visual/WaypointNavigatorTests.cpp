#include <cmath>

#include <gtest/gtest.h>

#include "robot/visual/ExplorationMap.hpp"
#include "robot/visual/NavigationProgressTracker.hpp"
#include "robot/visual/WaypointNavigator.hpp"

namespace
{

using robot::visual::ExplorationMap;
using robot::visual::NavigationProgressTracker;
using robot::visual::RobotPose;
using robot::visual::TableSurface;
using robot::visual::Vec3;
using robot::visual::WaypointNavigator;
using robot::visual::WaypointNavigatorOutput;
using robot::visual::WaypointNavigatorState;

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

RobotPose poseAt(const Vec3& position)
{
    return RobotPose{position, 0.0F};
}

// ============================================================
// NavigationProgressTracker
// ============================================================

TEST(NavigationProgressTrackerTest, NotStuckBeforeWindowElapses)
{
    NavigationProgressTracker tracker;
    RobotPose pose = poseAt(Vec3{0.0F, 0.0F, 0.0F});
    for (int i = 0; i < NavigationProgressTracker::kStuckSampleWindow - 5; ++i)
    {
        pose.headingDegrees += 90.0F;
        tracker.update(pose);
    }
    EXPECT_FALSE(tracker.isStuck());
}

TEST(NavigationProgressTrackerTest, NotStuckWhenTranslating)
{
    NavigationProgressTracker tracker;
    RobotPose pose = poseAt(Vec3{0.0F, 0.0F, 0.0F});
    for (int i = 0; i < NavigationProgressTracker::kStuckSampleWindow + 10; ++i)
    {
        pose.position.x += 0.05F; // genuine net translation every frame
        pose.headingDegrees += 90.0F;
        tracker.update(pose);
    }
    EXPECT_FALSE(tracker.isStuck());
}

TEST(NavigationProgressTrackerTest, StuckWhenRotatingWithoutTranslating)
{
    NavigationProgressTracker tracker;
    RobotPose pose = poseAt(Vec3{1.0F, 0.0F, 1.0F}); // position never changes below
    for (int i = 0; i < NavigationProgressTracker::kStuckSampleWindow + 10; ++i)
    {
        pose.headingDegrees += 45.0F; // accumulates well past 720 degrees
        tracker.update(pose);
    }
    EXPECT_TRUE(tracker.isStuck());
}

TEST(NavigationProgressTrackerTest, ResetReanchorsAndClearsStuckState)
{
    NavigationProgressTracker tracker;
    RobotPose pose = poseAt(Vec3{1.0F, 0.0F, 1.0F});
    for (int i = 0; i < NavigationProgressTracker::kStuckSampleWindow + 10; ++i)
    {
        pose.headingDegrees += 45.0F;
        tracker.update(pose);
    }
    ASSERT_TRUE(tracker.isStuck());

    tracker.reset();
    EXPECT_FALSE(tracker.isStuck());

    tracker.update(pose); // re-anchors on this call
    EXPECT_FALSE(tracker.isStuck());
}

// ============================================================
// WaypointNavigator
// ============================================================

TEST(WaypointNavigatorTest, InactiveWhenDisabled)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    WaypointNavigator navigator;

    const RobotPose pose = poseAt(map.cellToWorld(10, 10));
    const WaypointNavigatorOutput output =
        navigator.update(pose, map, map.cellToWorld(30, 30), false, false);

    EXPECT_EQ(output.state, WaypointNavigatorState::Inactive);
    EXPECT_TRUE(output.route.empty());
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, 0.0F);
}

TEST(WaypointNavigatorTest, FreshEnableStartsFollowingWithNonEmptyRoute)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    WaypointNavigator navigator;

    const Vec3 start = map.cellToWorld(8, 8);
    const Vec3 goal = map.cellToWorld(30, 30);
    const WaypointNavigatorOutput output = navigator.update(poseAt(start), map, goal, true, false);

    EXPECT_EQ(output.state, WaypointNavigatorState::Following);
    EXPECT_FALSE(output.route.empty());
}

TEST(WaypointNavigatorTest, FailedWhenNoPathExists)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    markOccupied(map, 0, map.width() - 1, 20, 20); // full-width wall, no gap
    WaypointNavigator navigator;

    const Vec3 start = map.cellToWorld(8, 10);
    const Vec3 goal = map.cellToWorld(8, 30);
    const WaypointNavigatorOutput output = navigator.update(poseAt(start), map, goal, true, false);

    EXPECT_EQ(output.state, WaypointNavigatorState::Failed);
    EXPECT_TRUE(output.failed);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.left, 0.0F);
    EXPECT_FLOAT_EQ(output.wheelSpeeds.right, 0.0F);
}

TEST(WaypointNavigatorTest, ReachingFinalWaypointReportsArrived)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    WaypointNavigator navigator;

    const Vec3 start = map.cellToWorld(8, 8);
    const Vec3 goal = map.cellToWorld(10, 8); // close, open map -> simple 2-point route
    const WaypointNavigatorOutput first = navigator.update(poseAt(start), map, goal, true, false);
    ASSERT_NE(first.state, WaypointNavigatorState::Failed);

    // Simulate the robot having physically reached the goal (teleport for
    // this unit test - real physical movement is covered by the
    // integration tests) and feed that pose back in.
    bool everArrived = first.arrived;
    WaypointNavigatorOutput output = first;
    for (int i = 0; i < 5 && !everArrived; ++i)
    {
        output = navigator.update(poseAt(goal), map, goal, true, false);
        everArrived = output.arrived;
    }

    EXPECT_TRUE(everArrived);
    EXPECT_EQ(navigator.state(), WaypointNavigatorState::Arrived);
}

TEST(WaypointNavigatorTest, ReplansWhenGoalChangesMaterially)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    WaypointNavigator navigator;

    const Vec3 start = map.cellToWorld(8, 8);
    const Vec3 goalA = map.cellToWorld(30, 8);
    navigator.update(poseAt(start), map, goalA, true, false);
    const std::vector<Vec3> routeToA = navigator.currentRoute();
    ASSERT_FALSE(routeToA.empty());

    const Vec3 goalB = map.cellToWorld(8, 30);
    navigator.update(poseAt(start), map, goalB, true, false);
    const std::vector<Vec3> routeToB = navigator.currentRoute();

    ASSERT_FALSE(routeToB.empty());
    EXPECT_GT(std::fabs(routeToB.back().z - routeToA.back().z), ExplorationMap::kCellSizeWorldUnits);
}

TEST(WaypointNavigatorTest, RouteInvalidatedByNewObstacleTriggersReplan)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    WaypointNavigator navigator;

    const Vec3 start = map.cellToWorld(8, 20);
    const Vec3 goal = map.cellToWorld(32, 20);
    const WaypointNavigatorOutput first = navigator.update(poseAt(start), map, goal, true, false);
    ASSERT_EQ(first.state, WaypointNavigatorState::Following);
    ASSERT_FALSE(first.route.empty());

    // Block a cell that the direct straight-line route would have used.
    map.markOccupied(20, 20);

    const WaypointNavigatorOutput second = navigator.update(poseAt(start), map, goal, true, false);
    for (const Vec3& waypoint : second.route)
    {
        int col = -1;
        int row = -1;
        if (map.worldToCell(waypoint, col, row))
        {
            EXPECT_NE(map.cellAt(col, row), robot::visual::MapCell::Occupied);
        }
    }
}

TEST(WaypointNavigatorTest, ResetReturnsToInactive)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    WaypointNavigator navigator;

    navigator.update(poseAt(map.cellToWorld(8, 8)), map, map.cellToWorld(30, 30), true, false);
    ASSERT_NE(navigator.state(), WaypointNavigatorState::Inactive);

    navigator.reset();
    EXPECT_EQ(navigator.state(), WaypointNavigatorState::Inactive);
    EXPECT_TRUE(navigator.currentRoute().empty());
}

TEST(WaypointNavigatorTest, DisablingThenReenablingReplansFresh)
{
    ExplorationMap map(bigBounds());
    markAllFree(map);
    WaypointNavigator navigator;

    const Vec3 start = map.cellToWorld(8, 8);
    const Vec3 goal = map.cellToWorld(30, 30);
    navigator.update(poseAt(start), map, goal, true, false);
    navigator.update(poseAt(start), map, goal, false, false);
    ASSERT_EQ(navigator.state(), WaypointNavigatorState::Inactive);

    const WaypointNavigatorOutput reenabled = navigator.update(poseAt(start), map, goal, true, false);
    EXPECT_EQ(reenabled.state, WaypointNavigatorState::Following);
    EXPECT_FALSE(reenabled.route.empty());
}

} // namespace
