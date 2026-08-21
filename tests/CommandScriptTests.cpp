#include <gtest/gtest.h>

#include <string>

#include "robot/CommandScript.hpp"
#include "robot/Event.hpp"

namespace
{

using robot::CommandScript;
using robot::CommandScriptEntry;
using robot::CommandScriptParseError;
using robot::EventType;

std::string FixturePath(const std::string& fileName)
{
    return std::string(TEST_FIXTURES_DIR) + fileName;
}

} // namespace

// 1: ParsesScenarioLoaded
TEST(CommandScriptTest, ParsesScenarioLoaded)
{
    // Act
    CommandScript script(FixturePath("command_script_scenario_loaded.txt"));

    // Assert
    ASSERT_EQ(script.entries().size(), 1u);
    EXPECT_EQ(script.entries()[0].cycle, 0u);
    EXPECT_EQ(script.entries()[0].eventType, EventType::ScenarioLoaded);
}

// 2: ParsesStartMission
TEST(CommandScriptTest, ParsesStartMission)
{
    // Act
    CommandScript script(FixturePath("command_script_start_mission.txt"));

    // Assert
    ASSERT_EQ(script.entries().size(), 1u);
    EXPECT_EQ(script.entries()[0].eventType, EventType::StartMission);
}

// 3: ParsesMissionCompleted
TEST(CommandScriptTest, ParsesMissionCompleted)
{
    // Act
    CommandScript script(FixturePath("command_script_mission_completed.txt"));

    // Assert
    ASSERT_EQ(script.entries().size(), 1u);
    EXPECT_EQ(script.entries()[0].eventType, EventType::MissionCompleted);
}

// 3b: ParsesReturnHome (Phase 13T)
TEST(CommandScriptTest, ParsesReturnHome)
{
    // Act
    CommandScript script(FixturePath("command_script_return_home.txt"));

    // Assert
    ASSERT_EQ(script.entries().size(), 1u);
    EXPECT_EQ(script.entries()[0].eventType, EventType::ReturnHomeRequested);
}

// 4: ParsesOtherSupportedCommandTypes (home_reached, reset)
TEST(CommandScriptTest, ParsesOtherSupportedCommandTypes)
{
    // Act
    CommandScript script(FixturePath("command_script_other_commands.txt"));

    // Assert
    ASSERT_EQ(script.entries().size(), 2u);
    EXPECT_EQ(script.entries()[0].eventType, EventType::HomeReached);
    EXPECT_EQ(script.entries()[1].eventType, EventType::Reset);
}

// 5: BlankLinesAllowed
TEST(CommandScriptTest, BlankLinesAllowed)
{
    // Act
    CommandScript script(FixturePath("command_script_blank_lines.txt"));

    // Assert: blank lines between/around real entries are skipped, not
    // treated as parse errors.
    ASSERT_EQ(script.entries().size(), 2u);
    EXPECT_EQ(script.entries()[0].eventType, EventType::ScenarioLoaded);
    EXPECT_EQ(script.entries()[1].eventType, EventType::StartMission);
}

// 6: CommentsAllowed
TEST(CommandScriptTest, CommentsAllowed)
{
    // Act
    CommandScript script(FixturePath("command_script_comments.txt"));

    // Assert: lines starting with '#' are skipped entirely.
    ASSERT_EQ(script.entries().size(), 2u);
    EXPECT_EQ(script.entries()[0].eventType, EventType::ScenarioLoaded);
    EXPECT_EQ(script.entries()[1].eventType, EventType::StartMission);
}

// 7: RejectsUnknownCommand
TEST(CommandScriptTest, RejectsUnknownCommand)
{
    // Arrange / Act / Assert
    EXPECT_THROW(CommandScript script(FixturePath("invalid_command_name.txt")), CommandScriptParseError);
}

// 8: RejectsInvalidCycle
TEST(CommandScriptTest, RejectsInvalidCycle)
{
    // Arrange / Act / Assert
    EXPECT_THROW(CommandScript script(FixturePath("command_script_invalid_cycle.txt")), CommandScriptParseError);
}

// 9: RejectsExtraTokens
TEST(CommandScriptTest, RejectsExtraTokens)
{
    // Arrange / Act / Assert
    EXPECT_THROW(CommandScript script(FixturePath("command_script_extra_tokens.txt")), CommandScriptParseError);
}

// Also covers the "too few tokens" half of "wrong token count" alongside
// RejectsExtraTokens above.
TEST(CommandScriptTest, RejectsTooFewTokens)
{
    // Arrange / Act / Assert
    EXPECT_THROW(CommandScript script(FixturePath("command_script_too_few_tokens.txt")), CommandScriptParseError);
}

// 10: ReportsLineNumber
TEST(CommandScriptTest, ReportsLineNumber)
{
    // Arrange
    bool threw = false;
    std::string message;

    // Act
    try
    {
        CommandScript script(FixturePath("command_script_line_number_error.txt"));
    }
    catch (const CommandScriptParseError& e)
    {
        threw = true;
        message = e.what();
    }

    // Assert: the error is on line 3 ("2 fly").
    ASSERT_TRUE(threw);
    EXPECT_NE(message.find("line 3"), std::string::npos);
}

// 11: StableSameCycleOrdering
TEST(CommandScriptTest, StableSameCycleOrdering)
{
    // Act
    CommandScript script(FixturePath("command_script_same_cycle_order.txt"));

    // Assert: all three entries share cycle 5 and must retain file order.
    ASSERT_EQ(script.entries().size(), 3u);
    EXPECT_EQ(script.entries()[0].eventType, EventType::ScenarioLoaded);
    EXPECT_EQ(script.entries()[1].eventType, EventType::StartMission);
    EXPECT_EQ(script.entries()[2].eventType, EventType::MissionCompleted);
}

// 12: UnsortedInputIsOrdered
TEST(CommandScriptTest, UnsortedInputIsOrdered)
{
    // Act
    CommandScript script(FixturePath("command_script_unsorted_cycles.txt"));

    // Assert: file order is 10, 2, 5 - entries() must come back sorted 2, 5, 10.
    ASSERT_EQ(script.entries().size(), 3u);
    EXPECT_EQ(script.entries()[0].cycle, 2u);
    EXPECT_EQ(script.entries()[1].cycle, 5u);
    EXPECT_EQ(script.entries()[2].cycle, 10u);
}

TEST(CommandScriptTest, NonexistentFileIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(CommandScript script(FixturePath("does_not_exist.txt")), CommandScriptParseError);
}
