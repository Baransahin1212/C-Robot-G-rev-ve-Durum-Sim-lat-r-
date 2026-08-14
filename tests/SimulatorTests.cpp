#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "robot/Event.hpp"
#include "robot/IEventSource.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/Simulator.hpp"
#include "robot/StreamSimulationLogger.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::IEventSource;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::Simulator;
using robot::SimulationResult;
using robot::StreamSimulationLogger;

// In-memory IEventSource for testing Simulator independently of JSON
// parsing: it simply replays a fixed list of events.
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

Event MakeEvent(EventType type, std::uint64_t timestampMs = 0)
{
    return Event{type, timestampMs, std::nullopt};
}

} // namespace

TEST(SimulatorTest, NormalMissionReachesCompletedWithAllTransitionsSuccessful)
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

    // Assert
    EXPECT_EQ(result.finalState, RobotState::Completed);
    EXPECT_EQ(result.eventsProcessed, 3u);
    EXPECT_EQ(result.successfulTransitions, 3u);
    EXPECT_EQ(result.rejectedTransitions, 0u);
}

TEST(SimulatorTest, ObstacleMissionResumesAndReachesCompleted)
{
    // Arrange
    FakeEventSource source({
        MakeEvent(EventType::ScenarioLoaded),
        MakeEvent(EventType::StartMission),
        MakeEvent(EventType::ObstacleDetected),
        MakeEvent(EventType::ObstacleCleared),
        MakeEvent(EventType::MissionCompleted),
    });
    RobotStateMachine machine;
    Simulator simulator(source, machine);

    // Act
    const SimulationResult result = simulator.run();

    // Assert
    EXPECT_EQ(result.finalState, RobotState::Completed);
    EXPECT_EQ(result.eventsProcessed, 5u);
    EXPECT_EQ(result.successfulTransitions, 5u);
    EXPECT_EQ(result.rejectedTransitions, 0u);
}

TEST(SimulatorTest, LowBatteryReturnReachesAborted)
{
    // Arrange
    FakeEventSource source({
        MakeEvent(EventType::ScenarioLoaded),
        MakeEvent(EventType::StartMission),
        MakeEvent(EventType::BatteryCritical),
        MakeEvent(EventType::HomeReached),
    });
    RobotStateMachine machine;
    Simulator simulator(source, machine);

    // Act
    const SimulationResult result = simulator.run();

    // Assert
    EXPECT_EQ(result.finalState, RobotState::Aborted);
    EXPECT_EQ(result.eventsProcessed, 4u);
    EXPECT_EQ(result.successfulTransitions, 4u);
    EXPECT_EQ(result.rejectedTransitions, 0u);
}

TEST(SimulatorTest, InvalidTransitionIsCountedAndProcessingContinues)
{
    // Arrange: MissionCompleted is invalid from Idle, ScenarioLoaded is not.
    FakeEventSource source({
        MakeEvent(EventType::MissionCompleted),
        MakeEvent(EventType::ScenarioLoaded),
    });
    RobotStateMachine machine;
    Simulator simulator(source, machine);

    // Act
    const SimulationResult result = simulator.run();

    // Assert
    EXPECT_EQ(result.finalState, RobotState::Ready);
    EXPECT_EQ(result.eventsProcessed, 2u);
    EXPECT_EQ(result.successfulTransitions, 1u);
    EXPECT_EQ(result.rejectedTransitions, 1u);
}

TEST(SimulatorTest, EmptyEventSourceLeavesStateAtIdle)
{
    // Arrange
    FakeEventSource source({});
    RobotStateMachine machine;
    Simulator simulator(source, machine);

    // Act
    const SimulationResult result = simulator.run();

    // Assert
    EXPECT_EQ(result.finalState, RobotState::Idle);
    EXPECT_EQ(result.eventsProcessed, 0u);
    EXPECT_EQ(result.successfulTransitions, 0u);
    EXPECT_EQ(result.rejectedTransitions, 0u);
}

TEST(SimulatorTest, EmergencyStopScenarioReachesEmergencyStopped)
{
    // Arrange
    FakeEventSource source({
        MakeEvent(EventType::ScenarioLoaded),
        MakeEvent(EventType::StartMission),
        MakeEvent(EventType::EmergencyStop),
    });
    RobotStateMachine machine;
    Simulator simulator(source, machine);

    // Act
    const SimulationResult result = simulator.run();

    // Assert
    EXPECT_EQ(result.finalState, RobotState::EmergencyStopped);
    EXPECT_EQ(result.eventsProcessed, 3u);
    EXPECT_EQ(result.successfulTransitions, 3u);
    EXPECT_EQ(result.rejectedTransitions, 0u);
}

TEST(SimulatorTest, NormalMissionProducesEventAndTransitionLogEntries)
{
    // Arrange
    FakeEventSource source({
        MakeEvent(EventType::ScenarioLoaded),
        MakeEvent(EventType::StartMission),
        MakeEvent(EventType::MissionCompleted),
    });
    RobotStateMachine machine;
    std::ostringstream out;
    StreamSimulationLogger logger(out);
    Simulator simulator(source, machine, &logger);

    // Act
    simulator.run();

    // Assert
    const std::string logText = out.str();
    EXPECT_NE(logText.find("ScenarioLoaded"), std::string::npos);
    EXPECT_NE(logText.find("StartMission"), std::string::npos);
    EXPECT_NE(logText.find("MissionCompleted"), std::string::npos);
    EXPECT_NE(logText.find("Idle -> Ready"), std::string::npos);
    EXPECT_NE(logText.find("Ready -> Moving"), std::string::npos);
    EXPECT_NE(logText.find("Moving -> Completed"), std::string::npos);
    EXPECT_EQ(logText.find("WARNING"), std::string::npos);
}

TEST(SimulatorTest, InvalidTransitionProducesWarningLog)
{
    // Arrange: MissionCompleted is invalid from Idle.
    FakeEventSource source({
        MakeEvent(EventType::MissionCompleted),
    });
    RobotStateMachine machine;
    std::ostringstream out;
    StreamSimulationLogger logger(out);
    Simulator simulator(source, machine, &logger);

    // Act
    simulator.run();

    // Assert
    const std::string logText = out.str();
    EXPECT_NE(logText.find("WARNING"), std::string::npos);
    EXPECT_NE(logText.find("Idle"), std::string::npos);
    EXPECT_NE(logText.find("MissionCompleted"), std::string::npos);
}

TEST(SimulatorTest, LoggingDoesNotAlterSimulationResultCounts)
{
    // Arrange: same scenario run once with a logger and once without.
    auto makeEvents = []() {
        return std::vector<Event>{
            MakeEvent(EventType::ScenarioLoaded),
            MakeEvent(EventType::StartMission),
            MakeEvent(EventType::ObstacleDetected),
            MakeEvent(EventType::ObstacleCleared),
            MakeEvent(EventType::MissionCompleted),
        };
    };

    FakeEventSource sourceWithoutLogger(makeEvents());
    RobotStateMachine machineWithoutLogger;
    Simulator simulatorWithoutLogger(sourceWithoutLogger, machineWithoutLogger);

    FakeEventSource sourceWithLogger(makeEvents());
    RobotStateMachine machineWithLogger;
    std::ostringstream out;
    StreamSimulationLogger logger(out);
    Simulator simulatorWithLogger(sourceWithLogger, machineWithLogger, &logger);

    // Act
    const SimulationResult resultWithoutLogger = simulatorWithoutLogger.run();
    const SimulationResult resultWithLogger = simulatorWithLogger.run();

    // Assert
    EXPECT_EQ(resultWithoutLogger.finalState, resultWithLogger.finalState);
    EXPECT_EQ(resultWithoutLogger.eventsProcessed, resultWithLogger.eventsProcessed);
    EXPECT_EQ(resultWithoutLogger.successfulTransitions, resultWithLogger.successfulTransitions);
    EXPECT_EQ(resultWithoutLogger.rejectedTransitions, resultWithLogger.rejectedTransitions);
}

TEST(SimulatorTest, LastEventTimestampReflectsMostRecentEvent)
{
    // Arrange
    FakeEventSource source({
        MakeEvent(EventType::ScenarioLoaded, 0),
        MakeEvent(EventType::StartMission, 100),
        MakeEvent(EventType::MissionCompleted, 6000),
    });
    RobotStateMachine machine;
    Simulator simulator(source, machine);

    // Act
    const SimulationResult result = simulator.run();

    // Assert
    ASSERT_TRUE(result.lastEventTimestampMs.has_value());
    EXPECT_EQ(*result.lastEventTimestampMs, 6000u);
}

TEST(SimulatorTest, LastEventTimestampIsNulloptWhenNoEventsProcessed)
{
    // Arrange
    FakeEventSource source({});
    RobotStateMachine machine;
    Simulator simulator(source, machine);

    // Act
    const SimulationResult result = simulator.run();

    // Assert
    EXPECT_FALSE(result.lastEventTimestampMs.has_value());
}
