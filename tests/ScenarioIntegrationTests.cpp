#include <gtest/gtest.h>

#include <cstdint>
#include <sstream>
#include <string>

#include "robot/JsonScenarioSource.hpp"
#include "robot/MissionOutcome.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/SimulationReport.hpp"
#include "robot/Simulator.hpp"
#include "robot/StreamSimulationLogger.hpp"

namespace
{

using robot::JsonScenarioSource;
using robot::makeSimulationReport;
using robot::MissionOutcome;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::Simulator;
using robot::SimulationReport;
using robot::SimulationResult;
using robot::StreamSimulationLogger;

std::string ScenarioPath(const std::string& fileName)
{
    return std::string(SCENARIOS_DIR) + fileName;
}

// One end-to-end run of a real scenario file through the actual production
// pipeline: JsonScenarioSource -> Simulator -> RobotStateMachine, with a
// StreamSimulationLogger writing to an in-memory stream so no real log file
// is touched during tests.
struct ScenarioRun
{
    SimulationResult result;
    SimulationReport report;
    std::string logText;
};

ScenarioRun RunScenario(const std::string& fileName)
{
    JsonScenarioSource source(ScenarioPath(fileName));
    RobotStateMachine machine;
    std::ostringstream logStream;
    StreamSimulationLogger logger(logStream);
    Simulator simulator(source, machine, &logger);

    ScenarioRun run;
    run.result = simulator.run();
    run.report = makeSimulationReport(run.result);
    run.logText = logStream.str();
    return run;
}

} // namespace

TEST(ScenarioIntegrationTest, NormalMission)
{
    // Act
    const ScenarioRun run = RunScenario("normal_mission.json");

    // Assert
    EXPECT_EQ(run.result.finalState, RobotState::Completed);
    EXPECT_EQ(run.report.outcome, MissionOutcome::Completed);
    EXPECT_EQ(run.result.eventsProcessed, 3u);
    EXPECT_EQ(run.result.successfulTransitions, 3u);
    EXPECT_EQ(run.result.rejectedTransitions, 0u);
}

TEST(ScenarioIntegrationTest, NormalMissionLastEventTimestampMatchesFinalEventInFile)
{
    // Act
    const ScenarioRun run = RunScenario("normal_mission.json");

    // Assert: normal_mission.json's final event (MISSION_COMPLETED) has
    // timestamp_ms = 5000.
    ASSERT_TRUE(run.result.lastEventTimestampMs.has_value());
    EXPECT_EQ(*run.result.lastEventTimestampMs, 5000u);
}

TEST(ScenarioIntegrationTest, ObstacleResume)
{
    // Act
    const ScenarioRun run = RunScenario("obstacle_resume.json");

    // Assert
    EXPECT_EQ(run.result.finalState, RobotState::Completed);
    EXPECT_EQ(run.report.outcome, MissionOutcome::Completed);
    EXPECT_EQ(run.result.eventsProcessed, 5u);
    EXPECT_EQ(run.result.successfulTransitions, 5u);
    EXPECT_EQ(run.result.rejectedTransitions, 0u);

    EXPECT_NE(run.logText.find("Moving -> WaitingForObstacleClear"), std::string::npos);
    EXPECT_NE(run.logText.find("WaitingForObstacleClear -> Moving"), std::string::npos);
}

TEST(ScenarioIntegrationTest, LowBatteryReturn)
{
    // Act
    const ScenarioRun run = RunScenario("low_battery_return.json");

    // Assert
    EXPECT_EQ(run.result.finalState, RobotState::Aborted);
    EXPECT_EQ(run.report.outcome, MissionOutcome::Aborted);
    EXPECT_EQ(run.result.eventsProcessed, 4u);
    EXPECT_EQ(run.result.successfulTransitions, 4u);
    EXPECT_EQ(run.result.rejectedTransitions, 0u);

    EXPECT_NE(run.logText.find("BatteryCritical"), std::string::npos);
    EXPECT_NE(run.logText.find("value=8"), std::string::npos);
    EXPECT_NE(run.logText.find("Moving -> ReturningHome"), std::string::npos);
    EXPECT_NE(run.logText.find("ReturningHome -> Aborted"), std::string::npos);
}

TEST(ScenarioIntegrationTest, EmergencyStop)
{
    // Act
    const ScenarioRun run = RunScenario("emergency_stop.json");

    // Assert
    EXPECT_EQ(run.result.finalState, RobotState::EmergencyStopped);
    EXPECT_EQ(run.report.outcome, MissionOutcome::EmergencyStopped);
    EXPECT_EQ(run.result.eventsProcessed, 3u);
    EXPECT_EQ(run.result.successfulTransitions, 3u);
    EXPECT_EQ(run.result.rejectedTransitions, 0u);

    EXPECT_NE(run.logText.find("Moving -> EmergencyStopped"), std::string::npos);
}

TEST(ScenarioIntegrationTest, InvalidSensorData)
{
    // Act
    const ScenarioRun run = RunScenario("invalid_sensor_data.json");

    // Assert
    EXPECT_EQ(run.result.finalState, RobotState::Error);
    EXPECT_EQ(run.report.outcome, MissionOutcome::Error);
    EXPECT_EQ(run.result.eventsProcessed, 3u);
    EXPECT_EQ(run.result.successfulTransitions, 3u);
    EXPECT_EQ(run.result.rejectedTransitions, 0u);

    EXPECT_NE(run.logText.find("Moving -> Error"), std::string::npos);
}

TEST(ScenarioIntegrationTest, InvalidTransition)
{
    // Act
    const ScenarioRun run = RunScenario("invalid_transition.json");

    // Assert: the leading Idle + MissionCompleted is rejected, then the
    // rest of the mission still runs to completion.
    EXPECT_EQ(run.result.finalState, RobotState::Completed);
    EXPECT_EQ(run.report.outcome, MissionOutcome::Completed);
    EXPECT_EQ(run.result.eventsProcessed, 4u);
    EXPECT_EQ(run.result.successfulTransitions, 3u);
    EXPECT_EQ(run.result.rejectedTransitions, 1u);

    EXPECT_NE(run.logText.find("WARNING"), std::string::npos);
    EXPECT_NE(run.logText.find("Idle"), std::string::npos);
    EXPECT_NE(run.logText.find("MissionCompleted"), std::string::npos);
}
