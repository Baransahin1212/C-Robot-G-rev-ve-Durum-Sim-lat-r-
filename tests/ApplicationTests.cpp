#include <gtest/gtest.h>

#include <filesystem>
#include <sstream>
#include <string>
#include <vector>

#include "robot/Application.hpp"
#include "robot/IRobotHardware.hpp"

namespace
{

using robot::IRobotHardware;
using robot::app::kExitScenarioError;
using robot::app::kExitSuccess;
using robot::app::kExitUsageError;
using robot::app::runApplication;
using robot::app::runSimulation;

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

RunOutcome InvokeWithHardware(const std::string& scenarioPath,
                               const std::filesystem::path& outputSubdir,
                               IRobotHardware& hardware)
{
    std::ostringstream out;
    std::ostringstream err;
    const int exitCode = runSimulation(scenarioPath,
                                        TestOutputDir(outputSubdir.string() + "/logs"),
                                        TestOutputDir(outputSubdir.string() + "/reports"),
                                        out,
                                        err,
                                        hardware);
    return RunOutcome{exitCode, out.str(), err.str()};
}

// Which actuator method was called, in a form local to this test file -
// deliberately not the production RobotCommand enum, so this test double
// stays decoupled from robot_hardware's internals and only proves what
// Application actually wired: that RobotController is driving IRobotHardware
// through the real runSimulation() path, not that any particular production
// type was used to observe it.
enum class RecordedCommand
{
    Stop,
    MoveForward,
    ReturnToBase
};

// Test-only IRobotHardware that records every actuator command it
// receives, in order. Sensor reads return fixed, safe defaults since this
// integration only exercises the actuator side (Application/RobotController
// wiring), never obstacle/battery/e-stop sensor input.
class RecordingRobotHardware : public IRobotHardware
{
public:
    int batteryLevelPercent() const override
    {
        return 100;
    }

    bool obstacleDetected() const override
    {
        return false;
    }

    bool emergencyStopPressed() const override
    {
        return false;
    }

    void moveForward() override
    {
        commands.push_back(RecordedCommand::MoveForward);
    }

    void stop() override
    {
        commands.push_back(RecordedCommand::Stop);
    }

    void returnToBase() override
    {
        commands.push_back(RecordedCommand::ReturnToBase);
    }

    std::vector<RecordedCommand> commands;
};

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

// --- Hardware wiring integration (Phase 13D) ---
//
// These tests drive the real runSimulation(..., IRobotHardware&) path with
// a RecordingRobotHardware, proving Application actually constructs a
// RobotController and injects it into Simulator end-to-end - not just that
// Simulator/RobotController work in isolation (already covered by
// SimulatorTests.cpp and RobotControllerTests.cpp).

// I: normal_mission - Idle -> Ready -> Moving -> Completed
TEST(ApplicationTest, NormalMissionDrivesHardwareThroughExpectedCommandSequence)
{
    // Arrange
    RecordingRobotHardware hardware;

    // Act
    const RunOutcome outcome =
        InvokeWithHardware(ScenarioPath("normal_mission.json"), "hardware_normal_mission", hardware);

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Mission outcome: Completed"), std::string::npos);
    const std::vector<RecordedCommand> expected{
        RecordedCommand::Stop,        // initial sync: Idle
        RecordedCommand::Stop,        // ScenarioLoaded -> Ready
        RecordedCommand::MoveForward, // StartMission -> Moving
        RecordedCommand::Stop,        // MissionCompleted -> Completed
    };
    EXPECT_EQ(hardware.commands, expected);
}

// J: obstacle_resume - Moving -> WaitingForObstacleClear -> Moving -> Completed
TEST(ApplicationTest, ObstacleResumeDrivesHardwareThroughExpectedCommandSequence)
{
    // Arrange
    RecordingRobotHardware hardware;

    // Act
    const RunOutcome outcome =
        InvokeWithHardware(ScenarioPath("obstacle_resume.json"), "hardware_obstacle_resume", hardware);

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Mission outcome: Completed"), std::string::npos);
    const std::vector<RecordedCommand> expected{
        RecordedCommand::Stop,        // initial sync: Idle
        RecordedCommand::Stop,        // ScenarioLoaded -> Ready
        RecordedCommand::MoveForward, // StartMission -> Moving
        RecordedCommand::Stop,        // ObstacleDetected -> WaitingForObstacleClear
        RecordedCommand::MoveForward, // ObstacleCleared -> Moving (resumed)
        RecordedCommand::Stop,        // MissionCompleted -> Completed
    };
    EXPECT_EQ(hardware.commands, expected);
}

// K: low_battery_return - Moving -> ReturningHome -> Aborted
TEST(ApplicationTest, LowBatteryReturnDrivesHardwareToBase)
{
    // Arrange
    RecordingRobotHardware hardware;

    // Act
    const RunOutcome outcome =
        InvokeWithHardware(ScenarioPath("low_battery_return.json"), "hardware_low_battery_return", hardware);

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    const std::vector<RecordedCommand> expected{
        RecordedCommand::Stop,        // initial sync: Idle
        RecordedCommand::Stop,        // ScenarioLoaded -> Ready
        RecordedCommand::MoveForward, // StartMission -> Moving
        RecordedCommand::ReturnToBase, // BatteryCritical -> ReturningHome
        RecordedCommand::Stop,        // HomeReached -> Aborted
    };
    EXPECT_EQ(hardware.commands, expected);
}

// L: emergency_stop - Moving -> EmergencyStopped
TEST(ApplicationTest, EmergencyStopStopsHardware)
{
    // Arrange
    RecordingRobotHardware hardware;

    // Act
    const RunOutcome outcome =
        InvokeWithHardware(ScenarioPath("emergency_stop.json"), "hardware_emergency_stop", hardware);

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    const std::vector<RecordedCommand> expected{
        RecordedCommand::Stop,        // initial sync: Idle
        RecordedCommand::Stop,        // ScenarioLoaded -> Ready
        RecordedCommand::MoveForward, // StartMission -> Moving
        RecordedCommand::Stop,        // EmergencyStop -> EmergencyStopped
    };
    EXPECT_EQ(hardware.commands, expected);
}

// M: invalid_transition - the leading rejected MISSION_COMPLETED (Idle
// doesn't accept it) must not produce any extra actuator command.
TEST(ApplicationTest, RejectedLeadingTransitionProducesNoExtraHardwareCommand)
{
    // Arrange
    RecordingRobotHardware hardware;

    // Act
    const RunOutcome outcome =
        InvokeWithHardware(ScenarioPath("invalid_transition.json"), "hardware_invalid_transition", hardware);

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Rejected transitions: 1"), std::string::npos);
    const std::vector<RecordedCommand> expected{
        RecordedCommand::Stop,        // initial sync: Idle
        // rejected MissionCompleted while Idle -> no controller call
        RecordedCommand::Stop,        // ScenarioLoaded -> Ready
        RecordedCommand::MoveForward, // StartMission -> Moving
        RecordedCommand::Stop,        // MissionCompleted -> Completed
    };
    EXPECT_EQ(hardware.commands, expected);
}
