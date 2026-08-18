#include <gtest/gtest.h>

#include <string>

#include "robot/SensorScript.hpp"

namespace
{

using robot::SensorKind;
using robot::SensorScript;
using robot::SensorScriptEntry;
using robot::SensorScriptParseError;

std::string FixturePath(const std::string& fileName)
{
    return std::string(TEST_FIXTURES_DIR) + fileName;
}

} // namespace

// 1: ParsesObstacleTrue
TEST(SensorScriptTest, ParsesObstacleTrue)
{
    // Act
    SensorScript script(FixturePath("sensor_script_obstacle_true.txt"));

    // Assert
    ASSERT_EQ(script.entries().size(), 1u);
    const SensorScriptEntry& entry = script.entries()[0];
    EXPECT_EQ(entry.cycle, 0u);
    EXPECT_EQ(entry.sensor, SensorKind::Obstacle);
    EXPECT_EQ(entry.value, 1);
}

// 2: ParsesObstacleFalse
TEST(SensorScriptTest, ParsesObstacleFalse)
{
    // Act
    SensorScript script(FixturePath("sensor_script_obstacle_false.txt"));

    // Assert
    ASSERT_EQ(script.entries().size(), 1u);
    const SensorScriptEntry& entry = script.entries()[0];
    EXPECT_EQ(entry.sensor, SensorKind::Obstacle);
    EXPECT_EQ(entry.value, 0);
}

// 3: ParsesBattery
TEST(SensorScriptTest, ParsesBattery)
{
    // Act
    SensorScript script(FixturePath("sensor_script_battery.txt"));

    // Assert
    ASSERT_EQ(script.entries().size(), 1u);
    const SensorScriptEntry& entry = script.entries()[0];
    EXPECT_EQ(entry.sensor, SensorKind::Battery);
    EXPECT_EQ(entry.value, 42);
}

// 4: ParsesEmergency
TEST(SensorScriptTest, ParsesEmergency)
{
    // Act
    SensorScript script(FixturePath("sensor_script_emergency.txt"));

    // Assert
    ASSERT_EQ(script.entries().size(), 1u);
    const SensorScriptEntry& entry = script.entries()[0];
    EXPECT_EQ(entry.sensor, SensorKind::EmergencyStop);
    EXPECT_EQ(entry.value, 1);
}

// 5: AllowsBlankLines
TEST(SensorScriptTest, AllowsBlankLines)
{
    // Act
    SensorScript script(FixturePath("sensor_script_blank_lines.txt"));

    // Assert: blank lines between/around real entries are skipped, not
    // treated as parse errors.
    ASSERT_EQ(script.entries().size(), 2u);
    EXPECT_EQ(script.entries()[0].cycle, 0u);
    EXPECT_EQ(script.entries()[1].cycle, 2u);
}

// 6: AllowsCommentLines
TEST(SensorScriptTest, AllowsCommentLines)
{
    // Act
    SensorScript script(FixturePath("sensor_script_comments.txt"));

    // Assert: lines starting with '#' are skipped entirely.
    ASSERT_EQ(script.entries().size(), 2u);
    EXPECT_EQ(script.entries()[0].cycle, 0u);
    EXPECT_EQ(script.entries()[1].cycle, 3u);
}

// 7: RejectsUnknownSensor
TEST(SensorScriptTest, RejectsUnknownSensor)
{
    // Arrange / Act / Assert
    EXPECT_THROW(SensorScript script(FixturePath("invalid_sensor_name.txt")), SensorScriptParseError);
}

// 8: RejectsInvalidBoolean
TEST(SensorScriptTest, RejectsInvalidBoolean)
{
    // Arrange / Act / Assert
    EXPECT_THROW(SensorScript script(FixturePath("invalid_boolean.txt")), SensorScriptParseError);
}

// 9: RejectsBatteryBelowZero
TEST(SensorScriptTest, RejectsBatteryBelowZero)
{
    // Arrange / Act / Assert
    EXPECT_THROW(SensorScript script(FixturePath("sensor_script_battery_below_zero.txt")), SensorScriptParseError);
}

// 10: RejectsBatteryAboveHundred
TEST(SensorScriptTest, RejectsBatteryAboveHundred)
{
    // Arrange / Act / Assert
    EXPECT_THROW(SensorScript script(FixturePath("invalid_battery.txt")), SensorScriptParseError);
}

// 11: RejectsInvalidCycle
TEST(SensorScriptTest, RejectsInvalidCycle)
{
    // Arrange / Act / Assert
    EXPECT_THROW(SensorScript script(FixturePath("sensor_script_invalid_cycle.txt")), SensorScriptParseError);
}

// 12: RejectsExtraTokens
TEST(SensorScriptTest, RejectsExtraTokens)
{
    // Arrange / Act / Assert
    EXPECT_THROW(SensorScript script(FixturePath("sensor_script_extra_tokens.txt")), SensorScriptParseError);
}

// Also covers the "too few tokens" half of "wrong token count" alongside
// RejectsExtraTokens above.
TEST(SensorScriptTest, RejectsTooFewTokens)
{
    // Arrange / Act / Assert
    EXPECT_THROW(SensorScript script(FixturePath("invalid_format.txt")), SensorScriptParseError);
}

// 13: ReportsLineNumber
TEST(SensorScriptTest, ReportsLineNumber)
{
    // Arrange
    bool threw = false;
    std::string message;

    // Act
    try
    {
        SensorScript script(FixturePath("sensor_script_line_number_error.txt"));
    }
    catch (const SensorScriptParseError& e)
    {
        threw = true;
        message = e.what();
    }

    // Assert: the error is on line 3 ("2 battery 999").
    ASSERT_TRUE(threw);
    EXPECT_NE(message.find("line 3"), std::string::npos);
}

// 14: StableOrderingForSameCycle
TEST(SensorScriptTest, StableOrderingForSameCycle)
{
    // Act
    SensorScript script(FixturePath("sensor_script_same_cycle_order.txt"));

    // Assert: all three entries share cycle 5 and must retain file order.
    ASSERT_EQ(script.entries().size(), 3u);
    EXPECT_EQ(script.entries()[0].sensor, SensorKind::Obstacle);
    EXPECT_EQ(script.entries()[1].sensor, SensorKind::Battery);
    EXPECT_EQ(script.entries()[2].sensor, SensorKind::EmergencyStop);
}

// 15: AcceptsUnsortedCyclesAndOrdersThemCorrectly
TEST(SensorScriptTest, AcceptsUnsortedCyclesAndOrdersThemCorrectly)
{
    // Act
    SensorScript script(FixturePath("sensor_script_unsorted_cycles.txt"));

    // Assert: file order is 10, 2, 5 - entries() must come back sorted 2, 5, 10.
    ASSERT_EQ(script.entries().size(), 3u);
    EXPECT_EQ(script.entries()[0].cycle, 2u);
    EXPECT_EQ(script.entries()[1].cycle, 5u);
    EXPECT_EQ(script.entries()[2].cycle, 10u);
}

TEST(SensorScriptTest, NonexistentFileIsRejected)
{
    // Arrange / Act / Assert
    EXPECT_THROW(SensorScript script(FixturePath("does_not_exist.txt")), SensorScriptParseError);
}
