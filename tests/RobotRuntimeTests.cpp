#include <gtest/gtest.h>

#include <deque>
#include <optional>

#include "robot/Event.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/IPollingEventSource.hpp"
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
using robot::RobotCommand;
using robot::RobotController;
using robot::RobotRuntime;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::RuntimeStepResult;
using robot::SimulatedRobotHardware;

// Small test-only IPollingEventSource that returns exactly the events it
// was given, one per pollEvent() call - used where the test wants precise
// control over what RobotRuntime receives on each step(), decoupled from
// HardwareEventSource's own already-tested edge/priority/queue logic
// (see HardwareEventSourceTests.cpp).
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

Event MakeEvent(EventType type, std::uint64_t timestampMs, std::optional<double> value = std::nullopt)
{
    return Event{type, timestampMs, value};
}

} // namespace

// A: SafeStepReturnsNoEvent
TEST(RobotRuntimeTest, SafeStepReturnsNoEvent)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotStateMachine machine;
    RobotRuntime runtime(source, machine, controller);

    // Act
    const RuntimeStepResult result = runtime.step();

    // Assert
    EXPECT_EQ(result, RuntimeStepResult::NoEvent);
}

// B: FirstStepSynchronizesHardwareWithCurrentFSMState
TEST(RobotRuntimeTest, FirstStepSynchronizesHardwareWithCurrentFSMState)
{
    // Arrange: hardware starts in a command inconsistent with the FSM's
    // initial Idle state, so the synchronization is meaningful.
    SimulatedRobotHardware hardware;
    hardware.moveForward();
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotStateMachine machine;
    RobotRuntime runtime(source, machine, controller);

    // Act
    const RuntimeStepResult result = runtime.step();

    // Assert
    EXPECT_EQ(result, RuntimeStepResult::NoEvent);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

// C: InitialSynchronizationOccursOnlyOnce
TEST(RobotRuntimeTest, InitialSynchronizationOccursOnlyOnce)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotStateMachine machine;
    RobotRuntime runtime(source, machine, controller);
    ASSERT_EQ(runtime.step(), RuntimeStepResult::NoEvent); // consumes the one-time sync

    // Act: manually move the command away from what the FSM state implies,
    // then run another event-free step.
    hardware.moveForward();
    const RuntimeStepResult result = runtime.step();

    // Assert: a second synchronization would have reset this to Stopped
    // (Idle's mapped command) - it did not, proving sync is one-time.
    EXPECT_EQ(result, RuntimeStepResult::NoEvent);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::MovingForward);
}

// D: NoEventIsNotPermanentExhaustion
TEST(RobotRuntimeTest, NoEventIsNotPermanentExhaustion)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotStateMachine machine;
    RobotRuntime runtime(source, machine, controller);
    ASSERT_EQ(runtime.step(), RuntimeStepResult::NoEvent);

    // Act: a new sensor edge appears after the prior NoEvent step.
    hardware.setEmergencyStopPressed(true);
    const RuntimeStepResult result = runtime.step();

    // Assert: the edge was handed to the FSM (rejected from Idle, but
    // processed, not silently dropped) - proving the earlier NoEvent did
    // not terminate the runtime the way IEventSource exhaustion would.
    EXPECT_NE(result, RuntimeStepResult::NoEvent);
}

// E: ObstacleEdgeCanDriveMovingFSMToWaiting
TEST(RobotRuntimeTest, ObstacleEdgeCanDriveMovingFSMToWaiting)
{
    // Arrange: drive the FSM into Moving via its real API before the live
    // runtime exists - HardwareEventSource has no mission-lifecycle events
    // of its own.
    RobotStateMachine machine;
    machine.processEvent(MakeEvent(EventType::ScenarioLoaded, 0));
    machine.processEvent(MakeEvent(EventType::StartMission, 1));
    ASSERT_EQ(machine.currentState(), RobotState::Moving);

    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);

    // Act: the obstacle edge is set before the first step(), so that one
    // step both performs the initial Moving -> moveForward() sync and then
    // polls straight into the rising edge.
    hardware.setObstacleDetected(true);
    const RuntimeStepResult result = runtime.step();

    // Assert
    EXPECT_EQ(result, RuntimeStepResult::TransitionAccepted);
    EXPECT_EQ(machine.currentState(), RobotState::WaitingForObstacleClear);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

// F: ObstacleClearCanResumeMoving
TEST(RobotRuntimeTest, ObstacleClearCanResumeMoving)
{
    // Arrange: reach WaitingForObstacleClear the same way as the previous
    // test (obstacle hit while Moving, so resumeState_ is Moving).
    RobotStateMachine machine;
    machine.processEvent(MakeEvent(EventType::ScenarioLoaded, 0));
    machine.processEvent(MakeEvent(EventType::StartMission, 1));
    ASSERT_EQ(machine.currentState(), RobotState::Moving);

    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);

    hardware.setObstacleDetected(true);
    ASSERT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    ASSERT_EQ(machine.currentState(), RobotState::WaitingForObstacleClear);

    // Act
    hardware.setObstacleDetected(false);
    const RuntimeStepResult result = runtime.step();

    // Assert
    EXPECT_EQ(result, RuntimeStepResult::TransitionAccepted);
    EXPECT_EQ(machine.currentState(), RobotState::Moving);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::MovingForward);
}

// G: RejectedEventDoesNotChangeActuatorCommand
TEST(RobotRuntimeTest, RejectedEventDoesNotChangeActuatorCommand)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotStateMachine machine; // Idle
    RobotRuntime runtime(source, machine, controller);
    ASSERT_EQ(runtime.step(), RuntimeStepResult::NoEvent); // one-time sync -> Stopped

    // Set a distinctive command unrelated to the FSM, so a later
    // accidental controller call would be obvious.
    hardware.moveForward();
    ASSERT_EQ(hardware.currentCommand(), RobotCommand::MovingForward);

    // Act: EmergencyStop is not accepted from Idle.
    hardware.setEmergencyStopPressed(true);
    const RuntimeStepResult result = runtime.step();

    // Assert
    EXPECT_EQ(result, RuntimeStepResult::TransitionRejected);
    EXPECT_EQ(machine.currentState(), RobotState::Idle);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::MovingForward);
}

// H: EmergencyEventUsesExistingFSM
TEST(RobotRuntimeTest, EmergencyEventUsesExistingFSM)
{
    // Arrange
    RobotStateMachine machine;
    machine.processEvent(MakeEvent(EventType::ScenarioLoaded, 0));
    machine.processEvent(MakeEvent(EventType::StartMission, 1));
    ASSERT_EQ(machine.currentState(), RobotState::Moving);

    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    RobotController controller(hardware);
    RobotRuntime runtime(source, machine, controller);

    // Act
    hardware.setEmergencyStopPressed(true);
    const RuntimeStepResult result = runtime.step();

    // Assert
    EXPECT_EQ(result, RuntimeStepResult::TransitionAccepted);
    EXPECT_EQ(machine.currentState(), RobotState::EmergencyStopped);
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::Stopped);
}

// I: PendingEventsAreProcessedAcrossSeparateSteps
TEST(RobotRuntimeTest, PendingEventsAreProcessedAcrossSeparateSteps)
{
    // Arrange: a focused QueuePollingEventSource stands in for
    // HardwareEventSource here, so this test can control exactly which
    // three events are queued (in the same emergency/battery/obstacle
    // priority order HardwareEventSource would produce) without depending
    // on its own sensor-edge sampling - that priority ordering is already
    // covered by HardwareEventSourceTests.cpp. The focus here is purely
    // RobotRuntime's one-event-per-step draining.
    RobotStateMachine machine;
    machine.processEvent(MakeEvent(EventType::ScenarioLoaded, 0));
    machine.processEvent(MakeEvent(EventType::StartMission, 1));
    ASSERT_EQ(machine.currentState(), RobotState::Moving);

    SimulatedRobotHardware hardware;
    RobotController controller(hardware);
    QueuePollingEventSource source;
    source.push(MakeEvent(EventType::EmergencyStop, 2));
    source.push(MakeEvent(EventType::BatteryCritical, 3, 15.0));
    source.push(MakeEvent(EventType::ObstacleDetected, 4));

    RobotRuntime runtime(source, machine, controller);

    // Act / Assert: step 1 - emergency stop is accepted from Moving.
    EXPECT_EQ(runtime.step(), RuntimeStepResult::TransitionAccepted);
    EXPECT_EQ(machine.currentState(), RobotState::EmergencyStopped);

    // Step 2 - battery-critical is handed to the FSM but rejected, since
    // EmergencyStopped only accepts Reset. It is still consumed from the
    // queue, not skipped.
    EXPECT_EQ(runtime.step(), RuntimeStepResult::TransitionRejected);
    EXPECT_EQ(machine.currentState(), RobotState::EmergencyStopped);

    // Step 3 - obstacle-detected, rejected for the same reason.
    EXPECT_EQ(runtime.step(), RuntimeStepResult::TransitionRejected);
    EXPECT_EQ(machine.currentState(), RobotState::EmergencyStopped);

    // Step 4 - the queue is now empty; nothing was silently dropped.
    EXPECT_EQ(runtime.step(), RuntimeStepResult::NoEvent);
}
