#include <gtest/gtest.h>

#include <cstddef>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "robot/Event.hpp"
#include "robot/IEventSource.hpp"
#include "robot/MissionOutcome.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/SimulationReport.hpp"
#include "robot/Simulator.hpp"
#include "robot/StreamReportWriter.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::IEventSource;
using robot::makeSimulationReport;
using robot::MissionOutcome;
using robot::missionOutcomeFromState;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::Simulator;
using robot::SimulationReport;
using robot::SimulationResult;
using robot::StreamReportWriter;

SimulationResult MakeResult(RobotState finalState,
                             std::size_t eventsProcessed = 0,
                             std::size_t successfulTransitions = 0,
                             std::size_t rejectedTransitions = 0,
                             std::optional<std::uint64_t> lastEventTimestampMs = std::nullopt)
{
    SimulationResult result;
    result.finalState = finalState;
    result.eventsProcessed = eventsProcessed;
    result.successfulTransitions = successfulTransitions;
    result.rejectedTransitions = rejectedTransitions;
    result.lastEventTimestampMs = lastEventTimestampMs;
    return result;
}

// In-memory IEventSource, local to this test file (mirrors SimulatorTests.cpp).
class FakeEventSource : public IEventSource
{
public:
    explicit FakeEventSource(std::vector<Event> events)
        : events_(std::move(events))
    {
    }

    std::optional<Event> nextEvent() override
    {
        if (index_ >= events_.size())
        {
            return std::nullopt;
        }
        return events_[index_++];
    }

private:
    std::vector<Event> events_;
    std::size_t index_ = 0;
};

Event MakeEvent(EventType type)
{
    return Event{type, 0, std::nullopt};
}

} // namespace

// --- A-E: RobotState -> MissionOutcome mapping ---

TEST(MissionOutcomeTest, CompletedResultMapsToCompletedOutcome)
{
    EXPECT_EQ(missionOutcomeFromState(RobotState::Completed), MissionOutcome::Completed);
}

TEST(MissionOutcomeTest, AbortedResultMapsToAbortedOutcome)
{
    EXPECT_EQ(missionOutcomeFromState(RobotState::Aborted), MissionOutcome::Aborted);
}

TEST(MissionOutcomeTest, EmergencyStoppedResultMapsToEmergencyStoppedOutcome)
{
    EXPECT_EQ(missionOutcomeFromState(RobotState::EmergencyStopped), MissionOutcome::EmergencyStopped);
}

TEST(MissionOutcomeTest, ErrorResultMapsToErrorOutcome)
{
    EXPECT_EQ(missionOutcomeFromState(RobotState::Error), MissionOutcome::Error);
}

TEST(MissionOutcomeTest, IncompleteResultForMovingStateMapsToIncompleteOutcome)
{
    EXPECT_EQ(missionOutcomeFromState(RobotState::Moving), MissionOutcome::Incomplete);
}

TEST(MissionOutcomeTest, AllOtherNonTerminalStatesMapToIncompleteOutcome)
{
    EXPECT_EQ(missionOutcomeFromState(RobotState::Idle), MissionOutcome::Incomplete);
    EXPECT_EQ(missionOutcomeFromState(RobotState::Ready), MissionOutcome::Incomplete);
    EXPECT_EQ(missionOutcomeFromState(RobotState::WaitingForObstacleClear), MissionOutcome::Incomplete);
    EXPECT_EQ(missionOutcomeFromState(RobotState::ReturningHome), MissionOutcome::Incomplete);
}

// --- F: report content ---

TEST(StreamReportWriterTest, ReportContainsOutcomeStateAndCounts)
{
    // Arrange
    const SimulationResult result = MakeResult(RobotState::Completed, 5, 5, 0);
    const SimulationReport report = makeSimulationReport(result);
    std::ostringstream out;
    StreamReportWriter writer(out);

    // Act
    writer.write(report);

    // Assert
    const std::string text = out.str();
    EXPECT_NE(text.find("Mission outcome: Completed"), std::string::npos);
    EXPECT_NE(text.find("Final state: Completed"), std::string::npos);
    EXPECT_NE(text.find("Events processed: 5"), std::string::npos);
    EXPECT_NE(text.find("Successful transitions: 5"), std::string::npos);
    EXPECT_NE(text.find("Rejected transitions: 0"), std::string::npos);
}

// --- G: simulation end time ---

TEST(StreamReportWriterTest, ReportIncludesSimulationEndTimeWhenAvailable)
{
    // Arrange
    const SimulationResult result = MakeResult(RobotState::Completed, 5, 5, 0, 6000);
    const SimulationReport report = makeSimulationReport(result);
    std::ostringstream out;
    StreamReportWriter writer(out);

    // Act
    writer.write(report);

    // Assert
    EXPECT_NE(out.str().find("Simulation end time: 6000 ms"), std::string::npos);
}

TEST(StreamReportWriterTest, ReportOmitsSimulationEndTimeWhenUnavailable)
{
    // Arrange
    const SimulationResult result = MakeResult(RobotState::Idle);
    const SimulationReport report = makeSimulationReport(result);
    std::ostringstream out;
    StreamReportWriter writer(out);

    // Act
    writer.write(report);

    // Assert
    EXPECT_EQ(out.str().find("Simulation end time"), std::string::npos);
}

// --- Integration: FakeEventSource -> Simulator -> SimulationResult -> ReportWriter ---

TEST(SimulationReportIntegrationTest, NormalMissionProducesCompletedReport)
{
    // Arrange
    FakeEventSource source({
        MakeEvent(EventType::ScenarioLoaded),
        MakeEvent(EventType::StartMission),
        MakeEvent(EventType::MissionCompleted),
    });
    RobotStateMachine machine;
    Simulator simulator(source, machine);

    // Act
    const SimulationResult result = simulator.run();
    const SimulationReport report = makeSimulationReport(result);
    std::ostringstream out;
    StreamReportWriter writer(out);
    writer.write(report);

    // Assert
    EXPECT_EQ(report.outcome, MissionOutcome::Completed);
    const std::string text = out.str();
    EXPECT_NE(text.find("Mission outcome: Completed"), std::string::npos);
    EXPECT_NE(text.find("Events processed: 3"), std::string::npos);
    EXPECT_NE(text.find("Successful transitions: 3"), std::string::npos);
    EXPECT_NE(text.find("Rejected transitions: 0"), std::string::npos);
}
