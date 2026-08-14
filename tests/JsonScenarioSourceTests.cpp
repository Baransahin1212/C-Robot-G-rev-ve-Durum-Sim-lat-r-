#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include "robot/Event.hpp"
#include "robot/JsonScenarioSource.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::JsonScenarioSource;
using robot::ScenarioParseError;

std::string FixturePath(const std::string& fileName)
{
    return std::string(TEST_FIXTURES_DIR) + fileName;
}

std::vector<Event> DrainAll(JsonScenarioSource& source)
{
    std::vector<Event> events;
    while (const std::optional<Event> event = source.nextEvent())
    {
        events.push_back(*event);
    }
    return events;
}

} // namespace

TEST(JsonScenarioSourceTest, ValidScenarioParsesAllEvents)
{
    // Arrange
    JsonScenarioSource source(FixturePath("valid_scenario.json"));

    // Act
    const std::vector<Event> events = DrainAll(source);

    // Assert
    EXPECT_EQ(events.size(), 6u);
}

TEST(JsonScenarioSourceTest, EventsAreReturnedInDeclaredOrder)
{
    // Arrange
    JsonScenarioSource source(FixturePath("valid_scenario.json"));

    // Act
    const std::vector<Event> events = DrainAll(source);

    // Assert
    const std::vector<EventType> expectedOrder{
        EventType::ScenarioLoaded,
        EventType::StartMission,
        EventType::ObstacleDetected,
        EventType::BatteryCritical,
        EventType::ObstacleCleared,
        EventType::MissionCompleted,
    };
    ASSERT_EQ(events.size(), expectedOrder.size());
    for (std::size_t i = 0; i < expectedOrder.size(); ++i)
    {
        EXPECT_EQ(events[i].type, expectedOrder[i]);
    }
}

TEST(JsonScenarioSourceTest, TimestampsArePreserved)
{
    // Arrange
    JsonScenarioSource source(FixturePath("valid_scenario.json"));

    // Act
    const std::vector<Event> events = DrainAll(source);

    // Assert
    ASSERT_EQ(events.size(), 6u);
    EXPECT_EQ(events[0].timestampMs, 0u);
    EXPECT_EQ(events[1].timestampMs, 100u);
    EXPECT_EQ(events[2].timestampMs, 2000u);
    EXPECT_EQ(events[3].timestampMs, 3000u);
    EXPECT_EQ(events[4].timestampMs, 4000u);
    EXPECT_EQ(events[5].timestampMs, 6000u);
}

TEST(JsonScenarioSourceTest, OptionalValueIsPreservedWhenPresent)
{
    // Arrange
    JsonScenarioSource source(FixturePath("valid_scenario.json"));

    // Act
    const std::vector<Event> events = DrainAll(source);

    // Assert
    ASSERT_EQ(events.size(), 6u);
    const Event& batteryCritical = events[3];
    ASSERT_EQ(batteryCritical.type, EventType::BatteryCritical);
    ASSERT_TRUE(batteryCritical.value.has_value());
    EXPECT_DOUBLE_EQ(*batteryCritical.value, 8.5);
}

TEST(JsonScenarioSourceTest, EventWithoutValueProducesNullopt)
{
    // Arrange
    JsonScenarioSource source(FixturePath("valid_scenario.json"));

    // Act
    const std::vector<Event> events = DrainAll(source);

    // Assert
    ASSERT_FALSE(events.empty());
    const Event& scenarioLoaded = events[0];
    ASSERT_EQ(scenarioLoaded.type, EventType::ScenarioLoaded);
    EXPECT_FALSE(scenarioLoaded.value.has_value());
}

TEST(JsonScenarioSourceTest, NextEventReturnsNulloptOnceExhausted)
{
    // Arrange
    JsonScenarioSource source(FixturePath("valid_scenario.json"));
    DrainAll(source);

    // Act
    const std::optional<Event> event = source.nextEvent();

    // Assert
    EXPECT_FALSE(event.has_value());
}

TEST(JsonScenarioSourceTest, UnknownEventTypeIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(JsonScenarioSource source(FixturePath("unknown_event_type.json")), ScenarioParseError);
}

TEST(JsonScenarioSourceTest, MalformedJsonIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(JsonScenarioSource source(FixturePath("malformed.json")), ScenarioParseError);
}

TEST(JsonScenarioSourceTest, MissingEventsArrayIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(JsonScenarioSource source(FixturePath("missing_events.json")), ScenarioParseError);
}

TEST(JsonScenarioSourceTest, EventsFieldNotArrayIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(JsonScenarioSource source(FixturePath("events_not_array.json")), ScenarioParseError);
}

TEST(JsonScenarioSourceTest, EventMissingTypeIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(JsonScenarioSource source(FixturePath("missing_type.json")), ScenarioParseError);
}

TEST(JsonScenarioSourceTest, EventMissingTimestampIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(JsonScenarioSource source(FixturePath("missing_timestamp.json")), ScenarioParseError);
}

TEST(JsonScenarioSourceTest, WrongTimestampTypeIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(JsonScenarioSource source(FixturePath("wrong_timestamp_type.json")), ScenarioParseError);
}

TEST(JsonScenarioSourceTest, WrongValueTypeIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(JsonScenarioSource source(FixturePath("wrong_value_type.json")), ScenarioParseError);
}

TEST(JsonScenarioSourceTest, NonexistentFileIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(JsonScenarioSource source(FixturePath("does_not_exist.json")), ScenarioParseError);
}
