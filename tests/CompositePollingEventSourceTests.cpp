#include <gtest/gtest.h>

#include <cstdint>
#include <deque>
#include <optional>

#include "robot/CompositePollingEventSource.hpp"
#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"

namespace
{

using robot::CompositePollingEventSource;
using robot::Event;
using robot::EventType;
using robot::IPollingEventSource;

// Same shape as the QueuePollingEventSource fakes in
// RobotRuntimeTests.cpp/LiveRuntimeRunnerTests.cpp, redefined locally per
// this project's existing convention of small test-only fakes living
// alongside the tests that use them.
class QueuePollingEventSource : public IPollingEventSource
{
public:
    void push(Event event)
    {
        queue_.push_back(event);
    }

    std::optional<Event> pollEvent() override
    {
        if (queue_.empty())
        {
            return std::nullopt;
        }
        const Event event = queue_.front();
        queue_.pop_front();
        return event;
    }

    std::size_t remaining() const
    {
        return queue_.size();
    }

private:
    std::deque<Event> queue_;
};

Event MakeEvent(EventType type, std::uint64_t timestampMs)
{
    return Event{type, timestampMs, std::nullopt};
}

} // namespace

// A: CommandHasPriorityOverSensorEvent
TEST(CompositePollingEventSourceTest, CommandHasPriorityOverSensorEvent)
{
    // Arrange
    QueuePollingEventSource commandSource;
    QueuePollingEventSource hardwareSource;
    commandSource.push(MakeEvent(EventType::StartMission, 0));
    hardwareSource.push(MakeEvent(EventType::ObstacleDetected, 0));
    CompositePollingEventSource composite(commandSource, hardwareSource);

    // Act
    const std::optional<Event> result = composite.pollEvent();

    // Assert: command event returned; hardware event is untouched (still
    // there when hardwareSource is polled directly).
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, EventType::StartMission);
    EXPECT_EQ(hardwareSource.remaining(), 1u);
}

// B: SensorEventUsedWhenNoCommandPending
TEST(CompositePollingEventSourceTest, SensorEventUsedWhenNoCommandPending)
{
    // Arrange
    QueuePollingEventSource commandSource; // empty
    QueuePollingEventSource hardwareSource;
    hardwareSource.push(MakeEvent(EventType::ObstacleDetected, 0));
    CompositePollingEventSource composite(commandSource, hardwareSource);

    // Act
    const std::optional<Event> result = composite.pollEvent();

    // Assert
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->type, EventType::ObstacleDetected);
}

// C: ReturnsNoEventWhenBothEmpty
TEST(CompositePollingEventSourceTest, ReturnsNoEventWhenBothEmpty)
{
    // Arrange
    QueuePollingEventSource commandSource;
    QueuePollingEventSource hardwareSource;
    CompositePollingEventSource composite(commandSource, hardwareSource);

    // Act
    const std::optional<Event> result = composite.pollEvent();

    // Assert
    EXPECT_FALSE(result.has_value());
}

// D: PendingCommandRemainsBeforeSensor
TEST(CompositePollingEventSourceTest, PendingCommandRemainsBeforeSensor)
{
    // Arrange
    QueuePollingEventSource commandSource;
    QueuePollingEventSource hardwareSource;
    commandSource.push(MakeEvent(EventType::ScenarioLoaded, 0));
    commandSource.push(MakeEvent(EventType::StartMission, 1));
    hardwareSource.push(MakeEvent(EventType::ObstacleDetected, 0));
    CompositePollingEventSource composite(commandSource, hardwareSource);

    // Act / Assert: both queued command events come out before the
    // hardware event is ever touched.
    const std::optional<Event> first = composite.pollEvent();
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->type, EventType::ScenarioLoaded);
    EXPECT_EQ(hardwareSource.remaining(), 1u);

    const std::optional<Event> second = composite.pollEvent();
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->type, EventType::StartMission);
    EXPECT_EQ(hardwareSource.remaining(), 1u);

    // Act: commandSource is now empty - the sensor event finally surfaces.
    const std::optional<Event> third = composite.pollEvent();
    ASSERT_TRUE(third.has_value());
    EXPECT_EQ(third->type, EventType::ObstacleDetected);
}

// E: AtMostOneEventReturnedPerPoll
TEST(CompositePollingEventSourceTest, AtMostOneEventReturnedPerPoll)
{
    // Arrange
    QueuePollingEventSource commandSource;
    QueuePollingEventSource hardwareSource;
    commandSource.push(MakeEvent(EventType::ScenarioLoaded, 0));
    commandSource.push(MakeEvent(EventType::StartMission, 1));
    CompositePollingEventSource composite(commandSource, hardwareSource);

    // Act: a single pollEvent() call must drain exactly one event from
    // commandSource, never both at once.
    const std::optional<Event> result = composite.pollEvent();

    // Assert
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(commandSource.remaining(), 1u);
}
