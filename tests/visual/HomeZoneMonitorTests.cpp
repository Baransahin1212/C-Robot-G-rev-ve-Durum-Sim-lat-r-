#include <gtest/gtest.h>

#include "robot/visual/HomeZoneMonitor.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::visual::BasePlatform;
using robot::visual::HomeZoneMonitor;
using robot::visual::RobotPose;
using robot::visual::Vec3;

BasePlatform MakeBase(float x, float z)
{
    return BasePlatform{Vec3{x, 0.025F, z}, Vec3{1.5F, 0.05F, 1.5F}};
}

RobotPose MakePose(float x, float z, float headingDegrees)
{
    return RobotPose{Vec3{x, 0.125F, z}, headingDegrees};
}

} // namespace

// 1: RobotInsideZoneDoesNotTrigger
TEST(HomeZoneMonitorTest, RobotInsideZoneDoesNotTrigger)
{
    // Arrange
    HomeZoneMonitor monitor;
    const BasePlatform base = MakeBase(0.0F, 0.0F);

    // Act / Assert: well inside kHomeZoneExitRadius (9.0F), repeatedly.
    for (int i = 0; i < 10; ++i)
    {
        EXPECT_FALSE(monitor.update(MakePose(3.0F, 0.0F, 0.0F), base, true));
    }
    EXPECT_TRUE(monitor.armed());
}

// 2: CrossingExitRadiusTriggersExactlyOnce
TEST(HomeZoneMonitorTest, CrossingExitRadiusTriggersExactlyOnce)
{
    // Arrange
    HomeZoneMonitor monitor;
    const BasePlatform base = MakeBase(0.0F, 0.0F);
    ASSERT_FALSE(monitor.update(MakePose(5.0F, 0.0F, 0.0F), base, true)); // inside

    // Act
    const bool triggered = monitor.update(MakePose(10.0F, 0.0F, 0.0F), base, true); // outside

    // Assert
    EXPECT_TRUE(triggered);
    EXPECT_FALSE(monitor.armed());
}

// 3: RemainingOutsideDoesNotSpam
TEST(HomeZoneMonitorTest, RemainingOutsideDoesNotSpam)
{
    // Arrange
    HomeZoneMonitor monitor;
    const BasePlatform base = MakeBase(0.0F, 0.0F);
    ASSERT_TRUE(monitor.update(MakePose(10.0F, 0.0F, 0.0F), base, true)); // first trigger

    // Act / Assert: stays outside for many more frames - no further
    // triggers.
    for (int i = 0; i < 20; ++i)
    {
        EXPECT_FALSE(monitor.update(MakePose(10.0F, 0.0F, 0.0F), base, true));
    }
}

// 4: EnteringRearmRadiusRearms
TEST(HomeZoneMonitorTest, EnteringRearmRadiusRearms)
{
    // Arrange
    HomeZoneMonitor monitor;
    const BasePlatform base = MakeBase(0.0F, 0.0F);
    ASSERT_TRUE(monitor.update(MakePose(10.0F, 0.0F, 0.0F), base, true));
    ASSERT_FALSE(monitor.armed());

    // Act: back within kHomeZoneRearmRadius (6.0F) - re-arms, but does
    // not itself trigger anything.
    const bool triggered = monitor.update(MakePose(5.0F, 0.0F, 0.0F), base, true);

    // Assert
    EXPECT_FALSE(triggered);
    EXPECT_TRUE(monitor.armed());
}

// 5: LeavingAgainTriggersSecondTime
TEST(HomeZoneMonitorTest, LeavingAgainTriggersSecondTime)
{
    // Arrange: trigger once, rearm.
    HomeZoneMonitor monitor;
    const BasePlatform base = MakeBase(0.0F, 0.0F);
    ASSERT_TRUE(monitor.update(MakePose(10.0F, 0.0F, 0.0F), base, true));
    ASSERT_FALSE(monitor.update(MakePose(5.0F, 0.0F, 0.0F), base, true));
    ASSERT_TRUE(monitor.armed());

    // Act: leaves the zone a second time.
    const bool triggeredAgain = monitor.update(MakePose(10.0F, 0.0F, 0.0F), base, true);

    // Assert
    EXPECT_TRUE(triggeredAgain);
    EXPECT_FALSE(monitor.armed());
}

// 6: ExitRadiusGreaterThanRearmRadius
TEST(HomeZoneMonitorTest, ExitRadiusGreaterThanRearmRadius)
{
    EXPECT_GT(HomeZoneMonitor::kHomeZoneExitRadius, HomeZoneMonitor::kHomeZoneRearmRadius);
}

// 7: DistanceUsesBaseCenter
TEST(HomeZoneMonitorTest, DistanceUsesBaseCenter)
{
    // Arrange: base far from the world origin - robot exactly at the
    // base center.
    HomeZoneMonitor monitor;
    const BasePlatform base = MakeBase(100.0F, 100.0F);

    // Act: zero distance from the (relocated) base - never outside,
    // regardless of how far both are from the origin.
    const bool triggered = monitor.update(MakePose(100.0F, 100.0F, 0.0F), base, true);

    // Assert
    EXPECT_FALSE(triggered);
}

// 8: HeadingDoesNotAffectZoneDistance
TEST(HomeZoneMonitorTest, HeadingDoesNotAffectZoneDistance)
{
    // Arrange
    const BasePlatform base = MakeBase(0.0F, 0.0F);

    // Act / Assert: identical position, four different headings - all
    // trigger identically (distance-only geometry).
    for (float heading : {0.0F, 90.0F, 180.0F, 270.0F})
    {
        HomeZoneMonitor monitor;
        EXPECT_TRUE(monitor.update(MakePose(10.0F, 0.0F, heading), base, true));
    }
}

// 9: DisabledWhenTaskIsNotRoam
TEST(HomeZoneMonitorTest, DisabledWhenTaskIsNotRoam)
{
    // Arrange
    HomeZoneMonitor monitor;
    const BasePlatform base = MakeBase(0.0F, 0.0F);

    // Act: well outside the exit radius, but roamActive is false.
    const bool triggered = monitor.update(MakePose(10.0F, 0.0F, 0.0F), base, false);

    // Assert
    EXPECT_FALSE(triggered);
    EXPECT_TRUE(monitor.armed()); // untouched, not disarmed
    EXPECT_FALSE(monitor.pollEvent().has_value());
}

// 10: BoundaryHandledDeterministically
TEST(HomeZoneMonitorTest, BoundaryHandledDeterministically)
{
    // Arrange
    HomeZoneMonitor monitor;
    const BasePlatform base = MakeBase(0.0F, 0.0F);

    // Act / Assert: exactly AT the exit radius does not trigger (">" is
    // strict, matching HomeNavigator's own "<=" arrival convention of
    // treating the boundary as still "in").
    EXPECT_FALSE(monitor.update(MakePose(HomeZoneMonitor::kHomeZoneExitRadius, 0.0F, 0.0F), base, true));
    EXPECT_TRUE(monitor.armed());

    // Act / Assert: crossing just past it does trigger.
    ASSERT_TRUE(monitor.update(MakePose(HomeZoneMonitor::kHomeZoneExitRadius + 0.01F, 0.0F, 0.0F), base, true));

    // Act / Assert: exactly AT the rearm radius does re-arm ("<=" is
    // inclusive).
    EXPECT_FALSE(monitor.update(MakePose(HomeZoneMonitor::kHomeZoneRearmRadius, 0.0F, 0.0F), base, true));
    EXPECT_TRUE(monitor.armed());
}

// 11: PendingEventDiscardedWhenRoamStops
TEST(HomeZoneMonitorTest, PendingEventDiscardedWhenRoamStops)
{
    // Arrange: a trigger fires but is not yet polled this frame (matches
    // main3d.cpp's real ordering, where a higher-priority Mission Control
    // command can be consumed by RobotRuntime::step() first).
    HomeZoneMonitor monitor;
    const BasePlatform base = MakeBase(0.0F, 0.0F);
    ASSERT_TRUE(monitor.update(MakePose(10.0F, 0.0F, 0.0F), base, true));

    // Act: the task is no longer Roam before pollEvent() ever consumed
    // the pending trigger (e.g. the user pressed Stop Task, or arrival
    // already completed some other way).
    monitor.update(MakePose(10.0F, 0.0F, 0.0F), base, false);

    // Assert: the stale trigger never fires.
    EXPECT_FALSE(monitor.pollEvent().has_value());
}

// 12: PollEventReturnsReturnHomeRequested
TEST(HomeZoneMonitorTest, PollEventReturnsReturnHomeRequested)
{
    // Arrange
    HomeZoneMonitor monitor;
    const BasePlatform base = MakeBase(0.0F, 0.0F);
    ASSERT_TRUE(monitor.update(MakePose(10.0F, 0.0F, 0.0F), base, true));

    // Act
    const std::optional<Event> event = monitor.pollEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::ReturnHomeRequested);
    EXPECT_FALSE(monitor.pollEvent().has_value()); // exactly once
}
