#include <gtest/gtest.h>

#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "robot/Application.hpp"

namespace
{

using robot::app::kExitScenarioError;
using robot::app::kExitSuccess;
using robot::app::kExitUsageError;
using robot::app::runApplication;

std::string ScenarioPath(const std::string& fileName)
{
    return std::string(SCENARIOS_DIR) + fileName;
}

std::string FixturePath(const std::string& fileName)
{
    return std::string(TEST_FIXTURES_DIR) + fileName;
}

// Each test gets its own subdirectory under the build tree so tests never
// touch the real logs/ or reports/ directories and never collide with
// each other's output files.
std::filesystem::path TestOutputDir(const std::string& subdir)
{
    return std::filesystem::path(TEST_OUTPUT_DIR) / subdir;
}

struct RunOutcome
{
    int exitCode;
    std::string stdOut;
    std::string stdErr;
};

RunOutcome Invoke(const std::vector<std::string>& args, const std::filesystem::path& outputSubdir)
{
    std::ostringstream out;
    std::ostringstream err;
    const int exitCode = runApplication(args,
                                         TestOutputDir(outputSubdir.string() + "/logs"),
                                         TestOutputDir(outputSubdir.string() + "/reports"),
                                         out,
                                         err);
    return RunOutcome{exitCode, out.str(), err.str()};
}

} // namespace

// A: --help
TEST(ApplicationTest, HelpLongFlagSucceedsAndPrintsUsage)
{
    // Act
    const RunOutcome outcome = Invoke({"--help"}, "help_long");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Usage"), std::string::npos);
}

// B: -h
TEST(ApplicationTest, HelpShortFlagSucceedsAndPrintsUsage)
{
    // Act
    const RunOutcome outcome = Invoke({"-h"}, "help_short");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Usage"), std::string::npos);
}

// C: no scenario argument
TEST(ApplicationTest, NoScenarioArgumentFailsWithUsage)
{
    // Act
    const RunOutcome outcome = Invoke({}, "missing_argument");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
    EXPECT_NE(outcome.stdErr.find("Usage"), std::string::npos);
}

// D: too many arguments
TEST(ApplicationTest, TooManyArgumentsFails)
{
    // Act
    const RunOutcome outcome = Invoke({"a.json", "b.json"}, "too_many_arguments");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
}

// E: valid normal_mission scenario
TEST(ApplicationTest, ValidNormalMissionScenarioSucceedsWithCompletedReport)
{
    // Arrange
    const std::filesystem::path outputSubdir = "normal_mission";

    // Act
    const RunOutcome outcome = Invoke({ScenarioPath("normal_mission.json")}, outputSubdir);

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Mission outcome: Completed"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Events processed: 3"), std::string::npos);
    EXPECT_TRUE(std::filesystem::exists(TestOutputDir("normal_mission/logs") / "simulation.log"));
    EXPECT_TRUE(std::filesystem::exists(TestOutputDir("normal_mission/reports") / "simulation_report.txt"));
}

// F: valid invalid_transition scenario - the application succeeds even
// though the scenario itself contains one rejected FSM transition.
TEST(ApplicationTest, ValidInvalidTransitionScenarioStillSucceeds)
{
    // Act
    const RunOutcome outcome = Invoke({ScenarioPath("invalid_transition.json")}, "invalid_transition");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Mission outcome: Completed"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Rejected transitions: 1"), std::string::npos);
}

// G: nonexistent scenario file
TEST(ApplicationTest, NonexistentScenarioFileFailsWithClearError)
{
    // Act
    const RunOutcome outcome = Invoke({ScenarioPath("does_not_exist.json")}, "nonexistent_scenario");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitScenarioError);
    EXPECT_NE(outcome.stdErr.find("Error loading scenario"), std::string::npos);
}

// H: malformed JSON
TEST(ApplicationTest, MalformedJsonScenarioFailsWithParseError)
{
    // Act
    const RunOutcome outcome = Invoke({FixturePath("malformed.json")}, "malformed_scenario");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitScenarioError);
    EXPECT_NE(outcome.stdErr.find("Error loading scenario"), std::string::npos);
}
