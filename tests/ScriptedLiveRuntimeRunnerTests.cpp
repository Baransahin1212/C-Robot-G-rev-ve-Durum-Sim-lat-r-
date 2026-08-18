#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <optional>
#include <string>

#include "robot/Event.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/ScriptedLiveRuntimeRunner.hpp"
#include "robot/SensorScript.hpp"
#include "robot/SimulatedRobotHardware.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::HardwareEventSource;
using robot::RobotCommand;
using robot::RobotController;
using robot::RobotRuntime;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::RuntimeRunSummary;
using robot::ScriptedLiveRuntimeRunner;
using robot::SensorScript;
using robot::SimulatedRobotHardware;

// Writes `contents` to a build-tree-local temp file, one per test - each
// test in this file needs cycle-specific script content, so ad hoc temp
// files (rather than a shared fixtures/ file) keep the intent local to
// each test, matching the ApplicationTests convention of a per-test
// TestOutputDir.
std::filesystem::path WriteScript(const std::string& contents, const std::string& name)
{
    const std::filesystem::path dir = std::filesystem::path(TEST_OUTPUT_DIR) / "sensor_script_runner";
    std::filesystem::create_directories(dir);
    const std::filesystem::path path = dir / name;
    std::ofstream file(path);
    file << contents;
    return path;
}

Event MakeEvent(EventType type, std::uint64_t timestampMs)
{
    return Event{type, timestampMs, std::nullopt};
}

// Drives the FSM into Moving via its real API, exactly like
// RobotRuntimeTests/LiveRuntimeRunnerTests do - HardwareEventSource has no
// mission-lifecycle events of its own to reach Moving from Idle.
RobotStateMachine MakeMovingStateMachine()
{
    RobotStateMachine machine;
    machine.processEvent(MakeEvent(EventType::ScenarioLoaded, 0));
    machine.processEvent(MakeEvent(EventType::StartMission, 1));
    return machine;
}

} // namespace

// A: EntryIsAppliedBeforeItsCycleStep
TEST(ScriptedLiveRuntimeRunnerTest, EntryIsAppliedBeforeItsCycleStep)
{
    // Arrange
    const SensorScript script(WriteScript("2 obstacle true\n", "entry_before_cycle.txt"));
    RobotStateMachine machine = MakeMovingStateMachine();
    ASSERT_EQ(machine.currentState(), RobotState::Moving);
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, script);

    // Act: cycles 0, 1, 2 - the obstacle mutation applies immediately
    // before cycle 2's step().
    const RuntimeRunSummary summary = runner.runCycles(3);

    // Assert
    EXPECT_EQ(summary.cyclesExecuted, 3u);
    EXPECT_EQ(summary.acceptedTransitions, 1u);
    EXPECT_EQ(machine.currentState(), RobotState::WaitingForObstacleClear);
}

// B: EntryAtCycleZeroIsAppliedBeforeFirstStep
TEST(ScriptedLiveRuntimeRunnerTest, EntryAtCycleZeroIsAppliedBeforeFirstStep)
{
    // Arrange
    const SensorScript script(WriteScript("0 obstacle true\n", "entry_at_cycle_zero.txt"));
    RobotStateMachine machine = MakeMovingStateMachine();
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, script);

    // Act: the very first step() both performs RobotRuntime's one-time
    // sync and observes the edge applied before it.
    const RuntimeRunSummary summary = runner.runCycles(1);

    // Assert
    EXPECT_EQ(summary.cyclesExecuted, 1u);
    EXPECT_EQ(summary.acceptedTransitions, 1u);
    EXPECT_EQ(machine.currentState(), RobotState::WaitingForObstacleClear);
}

// C: FutureEntryOutsideRunRangeIsNotApplied
TEST(ScriptedLiveRuntimeRunnerTest, FutureEntryOutsideRunRangeIsNotApplied)
{
    // Arrange: cycle 10 is never reached by a 5-cycle run.
    const SensorScript script(WriteScript("10 obstacle true\n", "future_entry.txt"));
    RobotStateMachine machine = MakeMovingStateMachine();
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, script);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(5);

    // Assert: not an error - the entry simply never executes.
    EXPECT_EQ(summary.cyclesExecuted, 5u);
    EXPECT_EQ(summary.noEventCycles, 5u);
    EXPECT_EQ(machine.currentState(), RobotState::Moving);
    EXPECT_FALSE(hardware.obstacleDetected());
}

// D: MultipleSameCycleEntriesAreAppliedInFileOrder
TEST(ScriptedLiveRuntimeRunnerTest, MultipleSameCycleEntriesAreAppliedInFileOrder)
{
    // Arrange
    const SensorScript script(WriteScript(
        "3 obstacle true\n"
        "3 battery 15\n"
        "3 emergency true\n",
        "same_cycle_entries.txt"));
    RobotStateMachine machine = MakeMovingStateMachine();
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, script);

    // Act: cycles 0..3 - all three mutations apply before cycle 3's step().
    const RuntimeRunSummary summary = runner.runCycles(4);

    // Assert: every mutation actually reached SimulatedRobotHardware.
    EXPECT_EQ(summary.cyclesExecuted, 4u);
    EXPECT_TRUE(hardware.obstacleDetected());
    EXPECT_EQ(hardware.batteryLevelPercent(), 15);
    EXPECT_TRUE(hardware.emergencyStopPressed());
}

// E: ScriptedObstacleEdgeProducesExpectedRuntimeResult
TEST(ScriptedLiveRuntimeRunnerTest, ScriptedObstacleEdgeProducesExpectedRuntimeResult)
{
    // Arrange
    const SensorScript script(WriteScript("1 obstacle true\n", "obstacle_edge.txt"));
    RobotStateMachine machine = MakeMovingStateMachine();
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, script);

    // Act: cycle 0 - NoEvent (with the one-time sync); cycle 1 - accepted.
    const RuntimeRunSummary summary = runner.runCycles(2);

    // Assert
    EXPECT_EQ(summary.cyclesExecuted, 2u);
    EXPECT_EQ(summary.noEventCycles, 1u);
    EXPECT_EQ(summary.acceptedTransitions, 1u);
    EXPECT_EQ(summary.rejectedTransitions, 0u);
    EXPECT_EQ(machine.currentState(), RobotState::WaitingForObstacleClear);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

// F: ScriptedObstacleClearCanResumeMoving
TEST(ScriptedLiveRuntimeRunnerTest, ScriptedObstacleClearCanResumeMoving)
{
    // Arrange
    const SensorScript script(WriteScript(
        "0 obstacle true\n"
        "2 obstacle false\n",
        "obstacle_clear_resume.txt"));
    RobotStateMachine machine = MakeMovingStateMachine();
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, script);

    // Act: cycle 0 - obstacle hit (accepted); cycle 1 - NoEvent; cycle 2 -
    // obstacle cleared, resumes Moving (accepted).
    const RuntimeRunSummary summary = runner.runCycles(3);

    // Assert
    EXPECT_EQ(summary.cyclesExecuted, 3u);
    EXPECT_EQ(summary.noEventCycles, 1u);
    EXPECT_EQ(summary.acceptedTransitions, 2u);
    EXPECT_EQ(machine.currentState(), RobotState::Moving);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::MovingForward);
}

// G: ScriptedBatteryCriticalCanDriveExistingValidTransition
TEST(ScriptedLiveRuntimeRunnerTest, ScriptedBatteryCriticalCanDriveExistingValidTransition)
{
    // Arrange
    const SensorScript script(WriteScript("0 battery 15\n", "battery_critical.txt"));
    RobotStateMachine machine = MakeMovingStateMachine();
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, script);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(1);

    // Assert: Moving + BatteryCritical -> ReturningHome is an existing,
    // unmodified FSM rule.
    EXPECT_EQ(summary.acceptedTransitions, 1u);
    EXPECT_EQ(machine.currentState(), RobotState::ReturningHome);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::ReturningToBase);
}

// H: ScriptedEmergencyStopUsesExistingTransition
TEST(ScriptedLiveRuntimeRunnerTest, ScriptedEmergencyStopUsesExistingTransition)
{
    // Arrange
    const SensorScript script(WriteScript("0 emergency true\n", "emergency_stop.txt"));
    RobotStateMachine machine = MakeMovingStateMachine();
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, script);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(1);

    // Assert: Moving + EmergencyStop -> EmergencyStopped is an existing,
    // unmodified FSM rule.
    EXPECT_EQ(summary.acceptedTransitions, 1u);
    EXPECT_EQ(machine.currentState(), RobotState::EmergencyStopped);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

// I: NoScriptEntriesBehavesLikeNormalLiveRunner
TEST(ScriptedLiveRuntimeRunnerTest, NoScriptEntriesBehavesLikeNormalLiveRunner)
{
    // Arrange: an empty script - default SimulatedRobotHardware sensors
    // (battery 100, no obstacle, no e-stop) never change.
    const SensorScript script(WriteScript("", "empty_script.txt"));
    RobotStateMachine machine; // Idle
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, script);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(5);

    // Assert: identical shape to LiveRuntimeRunner's own unscripted result.
    EXPECT_EQ(summary.cyclesExecuted, 5u);
    EXPECT_EQ(summary.noEventCycles, 5u);
    EXPECT_EQ(summary.acceptedTransitions, 0u);
    EXPECT_EQ(summary.rejectedTransitions, 0u);
    EXPECT_EQ(machine.currentState(), RobotState::Idle);
}

// J: ExactlyNCyclesStillExecute
TEST(ScriptedLiveRuntimeRunnerTest, ExactlyNCyclesStillExecute)
{
    // Arrange
    const SensorScript script(WriteScript("2 obstacle true\n", "exact_cycles.txt"));
    RobotStateMachine machine = MakeMovingStateMachine();
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, script);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(7);

    // Assert
    EXPECT_EQ(summary.cyclesExecuted, 7u);
}
