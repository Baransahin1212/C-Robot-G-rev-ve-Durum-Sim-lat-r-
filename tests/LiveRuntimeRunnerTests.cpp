#include <gtest/gtest.h>

#include <cstddef>
#include <deque>
#include <optional>
#include <vector>

#include "robot/Event.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/IPollingEventSource.hpp"
#include "robot/LiveRuntimeRunner.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/SimulatedRobotHardware.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::HardwareEventSource;
using robot::IPollingEventSource;
using robot::LiveRuntimeRunner;
using robot::RobotCommand;
using robot::RobotController;
using robot::RobotRuntime;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::RuntimeRunSummary;
using robot::SimulatedRobotHardware;

// Same shape as the QueuePollingEventSource in RobotRuntimeTests.cpp,
// redefined locally per this project's existing convention of small
// test-only fakes living alongside the tests that use them. Returns
// std::nullopt once its queue is drained.
class QueuePollingEventSource : public IPollingEventSource
{
public:
    void push(Event event)
    {
        queue_.push_back(event);
    }

    std::optional<Event> pollEvent() override
    {
        if (queue_.empty())
        {
            return std::nullopt;
        }
        const Event event = queue_.front();
        queue_.pop_front();
        return event;
    }

private:
    std::deque<Event> queue_;
};

// Replays an exact, caller-specified sequence of poll results (including
// explicit std::nullopt "gaps"), one per pollEvent() call - used where a
// test needs NoEvent cycles interspersed between specific events, which a
// plain FIFO queue of only-real-events cannot express.
class SequencedPollingEventSource : public IPollingEventSource
{
public:
    void push(std::optional<Event> result)
    {
        sequence_.push_back(result);
    }

    std::optional<Event> pollEvent() override
    {
        if (index_ >= sequence_.size())
        {
            return std::nullopt;
        }
        return sequence_[index_++];
    }

private:
    std::vector<std::optional<Event>> sequence_;
    std::size_t index_ = 0;
};

Event MakeEvent(EventType type, std::uint64_t timestampMs, std::optional<double> value = std::nullopt)
{
    return Event{type, timestampMs, value};
}

} // namespace

// A: ZeroCyclesReturnsZeroSummary
TEST(LiveRuntimeRunnerTest, ZeroCyclesReturnsZeroSummary)
{
    // Arrange: if runCycles(0) ever called step() even once, the one-time
    // sync would reset this to Stopped - it must not.
    SimulatedRobotHardware hardware;
    hardware.moveForward();
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotStateMachine machine;
    RobotRuntime runtime(source, machine, controller);
    LiveRuntimeRunner runner(runtime);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(0);

    // Assert
    EXPECT_EQ(summary.cyclesExecuted, 0u);
    EXPECT_EQ(summary.noEventCycles, 0u);
    EXPECT_EQ(summary.acceptedTransitions, 0u);
    EXPECT_EQ(summary.rejectedTransitions, 0u);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::MovingForward);
}

// B: RunsExactlyRequestedNumberOfCycles
TEST(LiveRuntimeRunnerTest, RunsExactlyRequestedNumberOfCycles)
{
    // Arrange
    RobotStateMachine machine;
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);
    QueuePollingEventSource source; // empty: every poll is NoEvent
    RobotRuntime runtime(source, machine, controller);
    LiveRuntimeRunner runner(runtime);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(7);

    // Assert
    EXPECT_EQ(summary.cyclesExecuted, 7u);
}

// C: NoEventDoesNotStopRunner
TEST(LiveRuntimeRunnerTest, NoEventDoesNotStopRunner)
{
    // Arrange
    RobotStateMachine machine;
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);
    QueuePollingEventSource source; // empty: every poll is NoEvent
    RobotRuntime runtime(source, machine, controller);
    LiveRuntimeRunner runner(runtime);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(3);

    // Assert
    EXPECT_EQ(summary.cyclesExecuted, 3u);
    EXPECT_EQ(summary.noEventCycles, 3u);
}

// D: CountsAcceptedTransitions
TEST(LiveRuntimeRunnerTest, CountsAcceptedTransitions)
{
    // Arrange
    RobotStateMachine machine; // Idle
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);
    QueuePollingEventSource source;
    source.push(MakeEvent(EventType::ScenarioLoaded, 0)); // valid from Idle
    RobotRuntime runtime(source, machine, controller);
    LiveRuntimeRunner runner(runtime);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(1);

    // Assert
    EXPECT_EQ(summary.acceptedTransitions, 1u);
    EXPECT_EQ(summary.rejectedTransitions, 0u);
    EXPECT_EQ(machine.currentState(), RobotState::Ready);
}

// E: CountsRejectedTransitions
TEST(LiveRuntimeRunnerTest, CountsRejectedTransitions)
{
    // Arrange
    RobotStateMachine machine; // Idle
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);
    QueuePollingEventSource source;
    source.push(MakeEvent(EventType::MissionCompleted, 0)); // invalid from Idle
    RobotRuntime runtime(source, machine, controller);
    LiveRuntimeRunner runner(runtime);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(1);

    // Assert
    EXPECT_EQ(summary.rejectedTransitions, 1u);
    EXPECT_EQ(summary.acceptedTransitions, 0u);
    EXPECT_EQ(machine.currentState(), RobotState::Idle);
}

// F: CountsMixedResultsCorrectly
TEST(LiveRuntimeRunnerTest, CountsMixedResultsCorrectly)
{
    // Arrange: NoEvent, Accepted (Idle->Ready), NoEvent, Rejected
    // (MissionCompleted invalid from Ready), Accepted (Ready->Moving).
    RobotStateMachine machine; // Idle
    SimulatedRobotHardware hardware;
    RobotController controller(hardware);
    SequencedPollingEventSource source;
    source.push(std::nullopt);
    source.push(MakeEvent(EventType::ScenarioLoaded, 0));
    source.push(std::nullopt);
    source.push(MakeEvent(EventType::MissionCompleted, 1));
    source.push(MakeEvent(EventType::StartMission, 2));
    RobotRuntime runtime(source, machine, controller);
    LiveRuntimeRunner runner(runtime);

    // Act
    const RuntimeRunSummary summary = runner.runCycles(5);

    // Assert
    EXPECT_EQ(summary.cyclesExecuted, 5u);
    EXPECT_EQ(summary.noEventCycles, 2u);
    EXPECT_EQ(summary.acceptedTransitions, 2u);
    EXPECT_EQ(summary.rejectedTransitions, 1u);
    EXPECT_EQ(machine.currentState(), RobotState::Moving);
}

// G: RunnerCanObserveEventAfterEarlierNoEventCycles
TEST(LiveRuntimeRunnerTest, RunnerCanObserveEventAfterEarlierNoEventCycles)
{
    // Arrange: real HardwareEventSource/SimulatedRobotHardware, driven into
    // Moving via the FSM's real API first.
    RobotStateMachine machine;
    machine.processEvent(MakeEvent(EventType::ScenarioLoaded, 0));
    machine.processEvent(MakeEvent(EventType::StartMission, 1));
    ASSERT_EQ(machine.currentState(), RobotState::Moving);

    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);
    LiveRuntimeRunner runner(runtime);

    // Act: an earlier batch of cycles sees no sensor edges at all.
    const RuntimeRunSummary firstSummary = runner.runCycles(2);

    // Assert: nothing happened yet, but the runner kept going regardless.
    EXPECT_EQ(firstSummary.cyclesExecuted, 2u);
    EXPECT_EQ(firstSummary.noEventCycles, 2u);
    EXPECT_EQ(firstSummary.acceptedTransitions, 0u);

    // Act: a sensor edge appears only after those earlier NoEvent cycles.
    hardware.setObstacleDetected(true);
    const RuntimeRunSummary secondSummary = runner.runCycles(2);

    // Assert: a later batch of cycles still picks it up.
    EXPECT_EQ(secondSummary.cyclesExecuted, 2u);
    EXPECT_EQ(secondSummary.acceptedTransitions, 1u);
    EXPECT_EQ(secondSummary.noEventCycles, 1u);
    EXPECT_EQ(machine.currentState(), RobotState::WaitingForObstacleClear);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}
