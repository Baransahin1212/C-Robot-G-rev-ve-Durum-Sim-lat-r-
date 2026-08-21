#include <gtest/gtest.h>

#include "robot/Event.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::ReturnHomeReason;
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

// Manual-validation bugfix: same destination state (ReturningHome) as
// MakeReturningHomeMachine() above, but reached via the explicit
// ReturnHomeRequested command instead of the automatic BatteryCritical
// trigger, so returnHomeReason() reads UserRequest, not MissionAbort.
RobotStateMachine MakeUserRequestedReturningHomeMachine()
{
    RobotStateMachine machine = MakeMovingMachine();
    machine.processEvent(MakeEvent(EventType::ReturnHomeRequested));
    return machine;
}

// Manual-validation bugfix: a full user-requested round trip - Moving ->
// ReturnHomeRequested -> ReturningHome -> HomeReached -> Ready - the
// "arrived home after an explicit R command" starting point a second R
// press should work from.
RobotStateMachine MakeReadyAfterUserRequestedReturnMachine()
{
    RobotStateMachine machine = MakeUserRequestedReturningHomeMachine();
    machine.processEvent(MakeEvent(EventType::HomeReached));
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

TEST(RobotStateMachineTest, MovingReturnHomeRequestedTransitionsToReturningHome)
{
    // Arrange
    RobotStateMachine machine = MakeMovingMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ReturnHomeRequested));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::ReturningHome);
}

TEST(RobotStateMachineTest, IdleReturnHomeRequestedIsRejected)
{
    // Arrange: ReturnHomeRequested is only meaningful from Moving - it
    // must not be accepted from every state (Phase 13T brief: "Do not add
    // broad transitions from every state without justification").
    RobotStateMachine machine;

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ReturnHomeRequested));

    // Assert
    EXPECT_EQ(result, TransitionResult::InvalidTransition);
    EXPECT_EQ(machine.currentState(), RobotState::Idle);
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

// --- Manual-validation bugfix: repeated Return Home / return-reason
// semantics (Phase 13T follow-up) ---
//
// Root cause: ReturningHome + HomeReached always went to Aborted, a
// terminal state with no outgoing transitions at all (not even Reset) -
// so after one user-requested Return Home completed, a second `R` press
// (Aborted + ReturnHomeRequested) was rejected, silently, forever. The
// fix distinguishes WHY the robot is returning home
// (ReturnHomeReason::MissionAbort vs. UserRequest) so only the automatic
// mission-abort path (e.g. BatteryCritical) still lands in Aborted; an
// explicit user request lands in the reusable Ready state instead, which
// itself now also accepts a further ReturnHomeRequested.

// 1: MovingReturnHomeRequestedSetsUserReason
TEST(RobotStateMachineTest, MovingReturnHomeRequestedSetsUserReason)
{
    // Arrange
    RobotStateMachine machine = MakeMovingMachine();

    // Act
    machine.processEvent(MakeEvent(EventType::ReturnHomeRequested));

    // Assert
    ASSERT_EQ(machine.currentState(), RobotState::ReturningHome);
    EXPECT_EQ(machine.returnHomeReason(), ReturnHomeReason::UserRequest);
}

// 2: ReadyReturnHomeRequestedTransitionsToReturningHome
TEST(RobotStateMachineTest, ReadyReturnHomeRequestedTransitionsToReturningHome)
{
    // Arrange: Ready reached the ordinary way (ScenarioLoaded) - this
    // must also work from the "arrived home via user request" Ready, but
    // this test isolates the transition rule itself from that specific
    // history.
    RobotStateMachine machine = MakeReadyMachine();

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ReturnHomeRequested));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::ReturningHome);
    EXPECT_EQ(machine.returnHomeReason(), ReturnHomeReason::UserRequest);
}

// 3: UserRequestedHomeReachedTransitionsToReusableState
TEST(RobotStateMachineTest, UserRequestedHomeReachedTransitionsToReusableState)
{
    // Arrange
    RobotStateMachine machine = MakeUserRequestedReturningHomeMachine();
    ASSERT_EQ(machine.returnHomeReason(), ReturnHomeReason::UserRequest);

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::HomeReached));

    // Assert: Ready, not Aborted - the whole point of this bugfix.
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Ready);
    EXPECT_EQ(machine.returnHomeReason(), ReturnHomeReason::None);
}

// 4: MissionAbortHomeReachedStillTransitionsToAborted
TEST(RobotStateMachineTest, MissionAbortHomeReachedStillTransitionsToAborted)
{
    // Arrange: the pre-existing BatteryCritical path - must be completely
    // unaffected by this bugfix.
    RobotStateMachine machine = MakeReturningHomeMachine();
    ASSERT_EQ(machine.returnHomeReason(), ReturnHomeReason::MissionAbort);

    // Act
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::HomeReached));

    // Assert
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Aborted);
    EXPECT_EQ(machine.returnHomeReason(), ReturnHomeReason::None);
}

// 5: ReturnReasonSurvivesObstacleWaitResume
TEST(RobotStateMachineTest, ReturnReasonSurvivesObstacleWaitResume)
{
    // Arrange: obstacle interrupts a user-requested Return Home.
    RobotStateMachine machine = MakeUserRequestedReturningHomeMachine();
    machine.processEvent(MakeEvent(EventType::ObstacleDetected));
    ASSERT_EQ(machine.currentState(), RobotState::WaitingForObstacleClear);

    // Act: obstacle clears, resuming ReturningHome.
    machine.processEvent(MakeEvent(EventType::ObstacleCleared));
    ASSERT_EQ(machine.currentState(), RobotState::ReturningHome);

    // Assert: the reason survived the interruption - resuming still
    // eventually lands in Ready, not Aborted.
    EXPECT_EQ(machine.returnHomeReason(), ReturnHomeReason::UserRequest);
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::HomeReached));
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::Ready);
}

// 6: ResetClearsReturnReason
TEST(RobotStateMachineTest, ResetClearsReturnReason)
{
    // Arrange: EmergencyStop interrupts a user-requested Return Home
    // before HomeReached is ever consumed, leaving returnHomeReason_ set.
    RobotStateMachine machine = MakeUserRequestedReturningHomeMachine();
    machine.processEvent(MakeEvent(EventType::EmergencyStop));
    ASSERT_EQ(machine.currentState(), RobotState::EmergencyStopped);
    ASSERT_EQ(machine.returnHomeReason(), ReturnHomeReason::UserRequest);

    // Act
    machine.processEvent(MakeEvent(EventType::Reset));

    // Assert: no stale reason leaks into whatever mission comes next.
    ASSERT_EQ(machine.currentState(), RobotState::Idle);
    EXPECT_EQ(machine.returnHomeReason(), ReturnHomeReason::None);
}

// 7: InvalidEmergencyOrErrorReturnHomeRequestRejected
TEST(RobotStateMachineTest, InvalidEmergencyOrErrorReturnHomeRequestRejected)
{
    // Arrange / Act / Assert: EmergencyStopped rejects ReturnHomeRequested.
    RobotStateMachine emergencyMachine = MakeEmergencyStoppedMachine();
    const TransitionResult emergencyResult =
        emergencyMachine.processEvent(MakeEvent(EventType::ReturnHomeRequested));
    EXPECT_EQ(emergencyResult, TransitionResult::InvalidTransition);
    EXPECT_EQ(emergencyMachine.currentState(), RobotState::EmergencyStopped);

    // Arrange / Act / Assert: Error rejects ReturnHomeRequested.
    RobotStateMachine errorMachine = MakeErrorMachine();
    const TransitionResult errorResult = errorMachine.processEvent(MakeEvent(EventType::ReturnHomeRequested));
    EXPECT_EQ(errorResult, TransitionResult::InvalidTransition);
    EXPECT_EQ(errorMachine.currentState(), RobotState::Error);
}

// 8: RepeatedRequestWhileReturningHomeHandledDeterministically
TEST(RobotStateMachineTest, RepeatedRequestWhileReturningHomeHandledDeterministically)
{
    // Arrange
    RobotStateMachine machine = MakeUserRequestedReturningHomeMachine();

    // Act: a second ReturnHomeRequested arrives while already
    // ReturningHome (e.g. the user presses R again mid-navigation).
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ReturnHomeRequested));

    // Assert: rejected, not restarted/duplicated - navigation is left
    // completely undisturbed (still ReturningHome, still UserRequest).
    EXPECT_EQ(result, TransitionResult::InvalidTransition);
    EXPECT_EQ(machine.currentState(), RobotState::ReturningHome);
    EXPECT_EQ(machine.returnHomeReason(), ReturnHomeReason::UserRequest);
}

// 9: SecondReturnHomeRequestFromReusedReadyIsAccepted
TEST(RobotStateMachineTest, SecondReturnHomeRequestFromReusedReadyIsAccepted)
{
    // Arrange: Ready reached specifically via a completed user-requested
    // Return Home (not the ScenarioLoaded path) - the exact scenario the
    // human validation report described.
    RobotStateMachine machine = MakeReadyAfterUserRequestedReturnMachine();
    ASSERT_EQ(machine.currentState(), RobotState::Ready);

    // Act: press R again.
    const TransitionResult result = machine.processEvent(MakeEvent(EventType::ReturnHomeRequested));

    // Assert: accepted - this is the fix for the reported defect.
    EXPECT_EQ(result, TransitionResult::Success);
    EXPECT_EQ(machine.currentState(), RobotState::ReturningHome);
    EXPECT_EQ(machine.returnHomeReason(), ReturnHomeReason::UserRequest);
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
