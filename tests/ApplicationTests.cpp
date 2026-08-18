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

// --- CLI live mode (Phase 13H) ---
//
// These tests drive the real runApplication() path in --live mode, proving
// Application composes SimulatedRobotHardware -> HardwareEventSource ->
// RobotStateMachine -> RobotController -> RobotRuntime -> LiveRuntimeRunner
// end-to-end. Default SimulatedRobotHardware sensors are safe (battery=100,
// no obstacle, no e-stop), so every cycle is expected to be NoEvent and the
// final state stays Idle - a boring result is the correct result.

// N: --live --cycles 5 runs exactly 5 cycles, all NoEvent, final state Idle.
TEST(ApplicationTest, LiveModeRunsRequestedCycles)
{
    // Act
    const RunOutcome outcome = Invoke({"--live", "--cycles", "5"}, "live_five_cycles");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Cycles executed: 5"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("No-event cycles: 5"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Accepted transitions: 0"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Rejected transitions: 0"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Final state: Idle"), std::string::npos);
}

// O: --live --cycles 0 runs no cycles at all, all-zero summary.
TEST(ApplicationTest, LiveModeAcceptsZeroCycles)
{
    // Act
    const RunOutcome outcome = Invoke({"--live", "--cycles", "0"}, "live_zero_cycles");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Cycles executed: 0"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("No-event cycles: 0"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Accepted transitions: 0"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Rejected transitions: 0"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Final state: Idle"), std::string::npos);
}

// P: --live with no --cycles option at all.
TEST(ApplicationTest, LiveModeRejectsMissingCyclesOption)
{
    // Act
    const RunOutcome outcome = Invoke({"--live"}, "live_missing_cycles_option");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
    EXPECT_NE(outcome.stdErr.find("Usage"), std::string::npos);
}

// Q: --live --cycles with no value.
TEST(ApplicationTest, LiveModeRejectsMissingCycleValue)
{
    // Act
    const RunOutcome outcome = Invoke({"--live", "--cycles"}, "live_missing_cycle_value");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
    EXPECT_NE(outcome.stdErr.find("Usage"), std::string::npos);
}

// R: --live --cycles -1
TEST(ApplicationTest, LiveModeRejectsNegativeCycleCount)
{
    // Act
    const RunOutcome outcome = Invoke({"--live", "--cycles", "-1"}, "live_negative_cycle_count");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
    EXPECT_NE(outcome.stdErr.find("Usage"), std::string::npos);
}

// S: --live --cycles abc
TEST(ApplicationTest, LiveModeRejectsNonNumericCycleCount)
{
    // Act
    const RunOutcome outcome = Invoke({"--live", "--cycles", "abc"}, "live_non_numeric_cycle_count");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
    EXPECT_NE(outcome.stdErr.find("Usage"), std::string::npos);
}

// T: --live --cycles 1.5
TEST(ApplicationTest, LiveModeRejectsFloatingPointCycleCount)
{
    // Act
    const RunOutcome outcome = Invoke({"--live", "--cycles", "1.5"}, "live_floating_point_cycle_count");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
    EXPECT_NE(outcome.stdErr.find("Usage"), std::string::npos);
}

// U: --live --cycles 10abc - trailing garbage after a valid numeric prefix.
TEST(ApplicationTest, LiveModeRejectsTrailingGarbage)
{
    // Act
    const RunOutcome outcome = Invoke({"--live", "--cycles", "10abc"}, "live_trailing_garbage");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
    EXPECT_NE(outcome.stdErr.find("Usage"), std::string::npos);
}

// V: --live --cycles 10 extra - unexpected extra argument after a valid pair.
TEST(ApplicationTest, LiveModeRejectsUnexpectedExtraArguments)
{
    // Act
    const RunOutcome outcome = Invoke({"--live", "--cycles", "10", "extra"}, "live_extra_arguments");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
    EXPECT_NE(outcome.stdErr.find("Usage"), std::string::npos);
}

// W: --help documents live mode.
TEST(ApplicationTest, HelpDocumentsLiveMode)
{
    // Act
    const RunOutcome outcome = Invoke({"--help"}, "help_documents_live_mode");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("--live --cycles"), std::string::npos);
}

// --- CLI scripted sensor injection (Phase 13I) ---
//
// These tests drive runApplication() with --sensor-script, proving
// Application wires SensorScript -> ScriptedLiveRuntimeRunner into the
// live path, while --live without --sensor-script and scenario mode both
// remain exactly as they were in Phase 13H.

// X: --live --cycles 20 --sensor-script <valid script> succeeds. Live mode
// starts in Idle, and Idle only accepts ScenarioLoaded, so every scripted
// obstacle/battery/emergency edge in valid_sensor_script.txt (4 edges: two
// obstacle transitions, one battery-critical crossing, one emergency
// stop) is handed to the FSM and rejected - this is expected, not a bug.
TEST(ApplicationTest, LiveModeAcceptsSensorScript)
{
    // Act
    const RunOutcome outcome = Invoke(
        {"--live", "--cycles", "20", "--sensor-script", FixturePath("valid_sensor_script.txt")},
        "live_with_script");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Sensor script: "), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Cycles executed: 20"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Accepted transitions: 0"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Rejected transitions: 4"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Final state: Idle"), std::string::npos);
}

// Y: --live --cycles 20 --sensor-script with no file value.
TEST(ApplicationTest, MissingSensorScriptValueRejected)
{
    // Act
    const RunOutcome outcome = Invoke({"--live", "--cycles", "20", "--sensor-script"}, "live_missing_script_value");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
    EXPECT_NE(outcome.stdErr.find("Usage"), std::string::npos);
}

// Z: trailing token after a syntactically complete --sensor-script pair.
TEST(ApplicationTest, ExtraArgumentsAfterScriptRejected)
{
    // Act
    const RunOutcome outcome = Invoke(
        {"--live", "--cycles", "20", "--sensor-script", "a.txt", "extra"}, "live_extra_after_script");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitUsageError);
    EXPECT_NE(outcome.stdErr.find("Usage"), std::string::npos);
}

// AA: CLI syntax is valid, but the script file does not exist - this must
// not be reported as a generic usage error.
TEST(ApplicationTest, MissingScriptFileReturnsNonZero)
{
    // Act
    const RunOutcome outcome = Invoke(
        {"--live", "--cycles", "5", "--sensor-script", FixturePath("does_not_exist.txt")},
        "live_missing_script_file");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitScenarioError);
    EXPECT_NE(outcome.stdErr.find("Error loading sensor script"), std::string::npos);
}

// BB: CLI syntax is valid, but the script content is malformed - same
// non-usage-error treatment, with a message useful enough to locate the
// bad line.
TEST(ApplicationTest, MalformedScriptReturnsNonZeroAndUsefulError)
{
    // Act
    const RunOutcome outcome = Invoke(
        {"--live", "--cycles", "5", "--sensor-script", FixturePath("invalid_boolean.txt")},
        "live_malformed_script");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitScenarioError);
    EXPECT_NE(outcome.stdErr.find("Error loading sensor script"), std::string::npos);
    EXPECT_NE(outcome.stdErr.find("line 1"), std::string::npos);
}

// CC: --live --cycles without --sensor-script is unchanged from Phase 13H.
TEST(ApplicationTest, ExistingLiveModeWithoutScriptStillWorksUnchanged)
{
    // Act
    const RunOutcome outcome = Invoke({"--live", "--cycles", "5"}, "live_unchanged_no_script");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_EQ(outcome.stdOut.find("Sensor script:"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Cycles executed: 5"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("No-event cycles: 5"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Final state: Idle"), std::string::npos);
}

// DD: scenario mode is untouched by this phase.
TEST(ApplicationTest, ExistingScenarioModeStillWorksUnchanged)
{
    // Act
    const RunOutcome outcome = Invoke({ScenarioPath("normal_mission.json")}, "scenario_mode_unchanged");

    // Assert
    EXPECT_EQ(outcome.exitCode, kExitSuccess);
    EXPECT_NE(outcome.stdOut.find("Mission outcome: Completed"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Successful transitions: 3"), std::string::npos);
    EXPECT_NE(outcome.stdOut.find("Rejected transitions: 0"), std::string::npos);
}
