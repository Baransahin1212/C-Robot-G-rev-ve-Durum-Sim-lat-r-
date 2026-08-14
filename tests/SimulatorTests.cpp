#include <gtest/gtest.h>

#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

#include "robot/Event.hpp"
#include "robot/IEventSource.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/Simulator.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::IEventSource;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::Simulator;
using robot::SimulationResult;

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

Event MakeEvent(EventType type)
{
    return Event{type, 0, std::nullopt};
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
