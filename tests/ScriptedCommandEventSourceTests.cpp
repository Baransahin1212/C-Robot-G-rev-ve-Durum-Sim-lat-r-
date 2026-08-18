#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#include "robot/CommandScript.hpp"
#include "robot/Event.hpp"
#include "robot/ScriptedCommandEventSource.hpp"

namespace
{

using robot::CommandScript;
using robot::Event;
using robot::EventType;
using robot::ScriptedCommandEventSource;

std::filesystem::path WriteScript(const std::string& contents, const std::string& name)
{
    const std::filesystem::path dir = std::filesystem::path(TEST_OUTPUT_DIR) / "command_event_source";
    std::filesystem::create_directories(dir);
    const std::filesystem::path path = dir / name;
    std::ofstream file(path);
    file << contents;
    return path;
}

} // namespace

// 13: FutureCommandIsNotEmittedEarly
TEST(ScriptedCommandEventSourceTest, FutureCommandIsNotEmittedEarly)
{
    // Arrange
    const CommandScript script(WriteScript("5 start_mission\n", "future_command.txt"));
    ScriptedCommandEventSource source(script);

    // Act / Assert: not eligible at cycles 0..4.
    for (std::size_t cycle = 0; cycle < 5; ++cycle)
    {
        source.setCurrentCycle(cycle);
        EXPECT_FALSE(source.pollEvent().has_value());
    }

    // Act: becomes eligible exactly at cycle 5.
    source.setCurrentCycle(5);
    const std::optional<Event> event = source.pollEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::StartMission);
}

// 14: CurrentCycleCommandIsEmitted
TEST(ScriptedCommandEventSourceTest, CurrentCycleCommandIsEmitted)
{
    // Arrange
    const CommandScript script(WriteScript("2 scenario_loaded\n", "current_cycle_command.txt"));
    ScriptedCommandEventSource source(script);

    // Act
    source.setCurrentCycle(2);
    const std::optional<Event> event = source.pollEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::ScenarioLoaded);
}

// 15: SameCycleCommandsAreEmittedInOrder
TEST(ScriptedCommandEventSourceTest, SameCycleCommandsAreEmittedInOrder)
{
    // Arrange
    const CommandScript script(WriteScript(
        "0 scenario_loaded\n"
        "0 start_mission\n"
        "0 mission_completed\n",
        "same_cycle_commands.txt"));
    ScriptedCommandEventSource source(script);
    source.setCurrentCycle(0);

    // Act
    const std::optional<Event> first = source.pollEvent();
    const std::optional<Event> second = source.pollEvent();
    const std::optional<Event> third = source.pollEvent();
    const std::optional<Event> fourth = source.pollEvent();

    // Assert: file order preserved, one event per call, exhausted after 3.
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    ASSERT_TRUE(third.has_value());
    EXPECT_EQ(first->type, EventType::ScenarioLoaded);
    EXPECT_EQ(second->type, EventType::StartMission);
    EXPECT_EQ(third->type, EventType::MissionCompleted);
    EXPECT_FALSE(fourth.has_value());
}

// 16: EligibleUnconsumedCommandRemainsPending
TEST(ScriptedCommandEventSourceTest, EligibleUnconsumedCommandRemainsPending)
{
    // Arrange
    const CommandScript script(WriteScript("3 start_mission\n", "pending_command.txt"));
    ScriptedCommandEventSource source(script);

    // Act: cycle advances past 3 without ever polling.
    source.setCurrentCycle(3);
    source.setCurrentCycle(4);
    source.setCurrentCycle(5);
    const std::optional<Event> event = source.pollEvent();

    // Assert: still delivered - eligibility, once reached, is not lost by
    // later cycles advancing without a poll.
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::StartMission);
}

// 17: EventsDoNotRepeatAfterConsumption
TEST(ScriptedCommandEventSourceTest, EventsDoNotRepeatAfterConsumption)
{
    // Arrange
    const CommandScript script(WriteScript("0 scenario_loaded\n", "no_repeat.txt"));
    ScriptedCommandEventSource source(script);
    source.setCurrentCycle(0);
    ASSERT_TRUE(source.pollEvent().has_value());

    // Act: poll again at the same cycle, and after advancing further.
    const std::optional<Event> secondPollSameCycle = source.pollEvent();
    source.setCurrentCycle(10);
    const std::optional<Event> pollAfterAdvancing = source.pollEvent();

    // Assert: consumed once, never replayed.
    EXPECT_FALSE(secondPollSameCycle.has_value());
    EXPECT_FALSE(pollAfterAdvancing.has_value());
}
