#include <gtest/gtest.h>

#include "robot/Event.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::TransitionResult;

Event MakeEvent(EventType type)
{
    return Event{type, 0, std::nullopt};
}

// Test-only helpers that drive a machine through a sequence of valid
// transitions to reach a starting state for a later test. These do not
// touch production code.
RobotStateMachine MakeReadyMachine()
{
    RobotStateMachine machine;
    machine.processEvent(MakeEvent(EventType::ScenarioLoaded));
    return machine;
}

RobotStateMachine MakeMovingMachine()
{
    RobotStateMachine machine = MakeReadyMachine();
    machine.processEvent(MakeEvent(EventType::StartMission));
    return machine;
}

RobotStateMachine MakeReturningHomeMachine()
{
    RobotStateMachine machine = MakeMovingMachine();
    machine.processEvent(MakeEvent(EventType::BatteryCritical));
    return machine;
}

RobotStateMachine MakeWaitingFromMovingMachine()
{
    RobotStateMachine machine = MakeMovingMachine();
    machine.processEvent(MakeEvent(EventType::ObstacleDetected));
    return machine;
}

RobotStateMachine MakeWaitingFromReturningHomeMachine()
{
    RobotStateMachine machine = MakeReturningHomeMachine();
    machine.processEvent(MakeEvent(EventType::ObstacleDetected));
    return machine;
}

RobotStateMachine MakeEmergencyStoppedMachine()
{
    RobotStateMachine machine = MakeMovingMachine();
    machine.processEvent(MakeEvent(EventType::EmergencyStop));
    return machine;
}

RobotStateMachine MakeErrorMachine()
{
    RobotStateMachine machine = MakeMovingMachine();
    machine.processEvent(MakeEvent(EventType::InvalidSensorData));
    return machine;
}

RobotStateMachine MakeCompletedMachine()
{
    RobotStateMachine machine = MakeMovingMachine();
    machine.processEvent(MakeEvent(EventType::MissionCompleted));
    return machine;
}

RobotStateMachine MakeAbortedMachine()
{
    RobotStateMachine machine = MakeReturningHomeMachine();
    machine.processEvent(MakeEvent(EventType::HomeReached));
    return machine;
}

} // namespace

TEST(RobotStateMachineTest, InitialStateIsIdle)
{
    // Arrange / Act
    RobotStateMachine machine;

    // Assert
    EXPECT_EQ(machine.currentState(), RobotState::Idle);
}

TEST(RobotStateMachineTest, IdleScenarioLoadedTransitionsToReady)
{
    // Arrange
    RobotStateMachine machine;

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ScenarioLoaded));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Ready);
}

TEST(RobotStateMachineTest, ReadyStartMissionTransitionsToMoving)
{
    // Arrange
    RobotStateMachine machine = MakeReadyMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::StartMission));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Moving);
}

TEST(RobotStateMachineTest, MovingMissionCompletedTransitionsToCompleted)
{
    // Arrange
    RobotStateMachine machine = MakeMovingMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::MissionCompleted));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Completed);
}

TEST(RobotStateMachineTest, MovingObstacleDetectedTransitionsToWaitingForObstacleClear)
{
    // Arrange
    RobotStateMachine machine = MakeMovingMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ObstacleDetected));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::WaitingForObstacleClear);
}

TEST(RobotStateMachineTest, ObstacleClearedAfterMovingResumesMoving)
{
    // Arrange
    RobotStateMachine machine = MakeWaitingFromMovingMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ObstacleCleared));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Moving);
}

TEST(RobotStateMachineTest, MovingBatteryCriticalTransitionsToReturningHome)
{
    // Arrange
    RobotStateMachine machine = MakeMovingMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::BatteryCritical));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::ReturningHome);
}

TEST(RobotStateMachineTest, ReturningHomeHomeReachedTransitionsToAborted)
{
    // Arrange
    RobotStateMachine machine = MakeReturningHomeMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::HomeReached));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Aborted);
}

TEST(RobotStateMachineTest, ReturningHomeObstacleDetectedTransitionsToWaitingForObstacleClear)
{
    // Arrange
    RobotStateMachine machine = MakeReturningHomeMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ObstacleDetected));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::WaitingForObstacleClear);
}

TEST(RobotStateMachineTest, ObstacleClearedAfterReturningHomeResumesReturningHome)
{
    // Arrange: obstacle hit while ReturningHome, so resumeState_ must be
    // ReturningHome, not Moving, when the obstacle clears.
    RobotStateMachine machine = MakeWaitingFromReturningHomeMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ObstacleCleared));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::ReturningHome);
}

TEST(RobotStateMachineTest, EmergencyStopFromMovingTransitionsToEmergencyStopped)
{
    // Arrange
    RobotStateMachine machine = MakeMovingMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::EmergencyStop));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::EmergencyStopped);
}

TEST(RobotStateMachineTest, EmergencyStopFromWaitingForObstacleClearTransitionsToEmergencyStopped)
{
    // Arrange
    RobotStateMachine machine = MakeWaitingFromMovingMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::EmergencyStop));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::EmergencyStopped);
}

TEST(RobotStateMachineTest, EmergencyStopFromReturningHomeTransitionsToEmergencyStopped)
{
    // Arrange
    RobotStateMachine machine = MakeReturningHomeMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::EmergencyStop));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::EmergencyStopped);
}

TEST(RobotStateMachineTest, EmergencyStoppedResetTransitionsToIdle)
{
    // Arrange
    RobotStateMachine machine = MakeEmergencyStoppedMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::Reset));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Idle);
}

TEST(RobotStateMachineTest, ErrorResetTransitionsToIdle)
{
    // Arrange
    RobotStateMachine machine = MakeErrorMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::Reset));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Idle);
}

TEST(RobotStateMachineTest, InvalidSensorDataFromMovingTransitionsToError)
{
    // Arrange
    RobotStateMachine machine = MakeMovingMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::InvalidSensorData));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Error);
}

TEST(RobotStateMachineTest, InvalidSensorDataFromWaitingForObstacleClearTransitionsToError)
{
    // Arrange
    RobotStateMachine machine = MakeWaitingFromMovingMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::InvalidSensorData));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Error);
}

TEST(RobotStateMachineTest, InvalidSensorDataFromReturningHomeTransitionsToError)
{
    // Arrange
    RobotStateMachine machine = MakeReturningHomeMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::InvalidSensorData));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Error);
}

TEST(RobotStateMachineTest, IdleMissionCompletedIsRejected)
{
    // Arrange
    RobotStateMachine machine;

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::MissionCompleted));

    // Assert
    EXPECT_EQ(result, TransitionResult::InvalidTransition);
    EXPECT_EQ(machine.currentState(), RobotState::Idle);
}

TEST(RobotStateMachineTest, CompletedRejectsFurtherEventsAndStateIsUnchanged)
{
    // Arrange
    RobotStateMachine machine = MakeCompletedMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::StartMission));

    // Assert
    EXPECT_EQ(result, TransitionResult::InvalidTransition);
    EXPECT_EQ(machine.currentState(), RobotState::Completed);
}

TEST(RobotStateMachineTest, AbortedRejectsFurtherEventsAndStateIsUnchanged)
{
    // Arrange
    RobotStateMachine machine = MakeAbortedMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ScenarioLoaded));

    // Assert
    EXPECT_EQ(result, TransitionResult::InvalidTransition);
    EXPECT_EQ(machine.currentState(), RobotState::Aborted);
}
