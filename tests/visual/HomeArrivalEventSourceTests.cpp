#include <gtest/gtest.h>

#include "robot/Event.hpp"
#include "robot/visual/HomeArrivalEventSource.hpp"
#include "robot/visual/HomeNavigator.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::visual::BasePlatform;
using robot::visual::HomeArrivalEventSource;
using robot::visual::HomeNavigator;
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

// 1: NoEventWhenNotArrived
TEST(HomeArrivalEventSourceTest, NoEventWhenNotArrived)
{
    // Arrange
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 10.0F);
    navigator.update(MakePose(0.0F, 0.0F, 0.0F), base, true); // far from base -> Aligning/Driving
    HomeArrivalEventSource source(navigator);

    // Act
    const std::optional<Event> event = source.pollEvent();

    // Assert
    EXPECT_FALSE(event.has_value());
}

// 2: EmitsHomeReachedOnArrival
TEST(HomeArrivalEventSourceTest, EmitsHomeReachedOnArrival)
{
    // Arrange
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 0.0F);
    navigator.update(MakePose(0.0F, 0.0F, 0.0F), base, true); // exactly on base -> Arrived
    HomeArrivalEventSource source(navigator);

    // Act
    const std::optional<Event> event = source.pollEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::HomeReached);
    EXPECT_FALSE(event->value.has_value());
}

// 3: DoesNotRepeatHomeReachedWhileStillArrived
TEST(HomeArrivalEventSourceTest, DoesNotRepeatHomeReachedWhileStillArrived)
{
    // Arrange
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 0.0F);
    navigator.update(MakePose(0.0F, 0.0F, 0.0F), base, true);
    HomeArrivalEventSource source(navigator);
    ASSERT_TRUE(source.pollEvent().has_value());

    // Act: navigator remains Arrived across further update() calls.
    navigator.update(MakePose(0.0F, 0.0F, 0.0F), base, true);
    const std::optional<Event> second = source.pollEvent();
    navigator.update(MakePose(0.0F, 0.0F, 0.0F), base, true);
    const std::optional<Event> third = source.pollEvent();

    // Assert: no spam - only the original rising edge produced an event.
    EXPECT_FALSE(second.has_value());
    EXPECT_FALSE(third.has_value());
}

// 4: NoEventWhenLeavingArrived
TEST(HomeArrivalEventSourceTest, NoEventWhenLeavingArrived)
{
    // Arrange
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 0.0F);
    navigator.update(MakePose(0.0F, 0.0F, 0.0F), base, true);
    HomeArrivalEventSource source(navigator);
    ASSERT_TRUE(source.pollEvent().has_value());

    // Act: mission ends, navigation disabled (mirrors main3d.cpp once
    // RobotController stops requesting ReturnToBase after HomeReached is
    // consumed) - HomeNavigator resets to Inactive.
    navigator.update(MakePose(0.0F, 0.0F, 0.0F), base, false);
    const std::optional<Event> event = source.pollEvent();

    // Assert: no "un-arrival" event exists in the vocabulary.
    EXPECT_FALSE(event.has_value());
}

// 5: ReArmsAfterLeavingAndReturningToArrived
TEST(HomeArrivalEventSourceTest, ReArmsAfterLeavingAndReturningToArrived)
{
    // Arrange: first Return Home mission completes and is consumed.
    HomeNavigator navigator;
    const BasePlatform base = MakeBase(0.0F, 0.0F);
    navigator.update(MakePose(0.0F, 0.0F, 0.0F), base, true);
    HomeArrivalEventSource source(navigator);
    ASSERT_TRUE(source.pollEvent().has_value());
    navigator.update(MakePose(0.0F, 0.0F, 0.0F), base, false); // disabled between missions
    ASSERT_FALSE(source.pollEvent().has_value());

    // Act: a second, distinct Return Home mission also arrives at the base.
    navigator.update(MakePose(0.0F, 0.0F, 0.0F), base, true);
    const std::optional<Event> event = source.pollEvent();

    // Assert: the second arrival produces its own fresh HomeReached edge.
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::HomeReached);
}

// 6: NoEventBeforeAnyUpdate
TEST(HomeArrivalEventSourceTest, NoEventBeforeAnyUpdate)
{
    // Arrange: navigator has never had update() called on it (Inactive).
    HomeNavigator navigator;
    HomeArrivalEventSource source(navigator);

    // Act
    const std::optional<Event> event = source.pollEvent();

    // Assert
    EXPECT_FALSE(event.has_value());
}
