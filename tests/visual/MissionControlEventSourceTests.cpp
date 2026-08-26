#include <gtest/gtest.h>

#include "robot/Event.hpp"
#include "robot/RobotState.hpp"
#include "robot/visual/MissionControlEventSource.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::RobotState;
using robot::visual::MissionControlEventSource;

} // namespace

// 1: DefaultsWithNoPendingEvent
TEST(MissionControlEventSourceTest, DefaultsWithNoPendingEvent)
{
    // Arrange
    MissionControlEventSource source;

    // Act
    const std::optional<Event> event = source.pollEvent();

    // Assert
    EXPECT_FALSE(event.has_value());
}

// 2: IdleStartRoamQueuesScenarioLoadedFirst
TEST(MissionControlEventSourceTest, IdleStartRoamQueuesScenarioLoadedFirst)
{
    // Arrange
    MissionControlEventSource source;

    // Act
    source.requestStartRoam(RobotState::Idle);
    const std::optional<Event> first = source.pollEvent();

    // Assert
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->type, EventType::ScenarioLoaded);
}

// 3: IdleStartRoamQueuesStartMissionSecond
TEST(MissionControlEventSourceTest, IdleStartRoamQueuesStartMissionSecond)
{
    // Arrange
    MissionControlEventSource source;
    source.requestStartRoam(RobotState::Idle);
    ASSERT_TRUE(source.pollEvent().has_value()); // consume ScenarioLoaded

    // Act
    const std::optional<Event> second = source.pollEvent();

    // Assert
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->type, EventType::StartMission);
}

// 4: OnlyOneEventReturnedPerPoll
TEST(MissionControlEventSourceTest, OnlyOneEventReturnedPerPoll)
{
    // Arrange
    MissionControlEventSource source;
    source.requestStartRoam(RobotState::Idle);

    // Act
    const std::optional<Event> first = source.pollEvent();

    // Assert: only ScenarioLoaded came back - StartMission is still
    // queued, not merged/returned together.
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->type, EventType::ScenarioLoaded);
    EXPECT_NE(first->type, EventType::StartMission);
}

// 5: ReadyStartRoamQueuesOnlyStartMission
TEST(MissionControlEventSourceTest, ReadyStartRoamQueuesOnlyStartMission)
{
    // Arrange
    MissionControlEventSource source;

    // Act
    source.requestStartRoam(RobotState::Ready);
    const std::optional<Event> first = source.pollEvent();
    const std::optional<Event> second = source.pollEvent();

    // Assert: exactly one event, StartMission - no ScenarioLoaded.
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->type, EventType::StartMission);
    EXPECT_FALSE(second.has_value());
}

// 5b: AlreadyMovingStartRoamIsNoOp
TEST(MissionControlEventSourceTest, AlreadyMovingStartRoamIsNoOp)
{
    // Arrange
    MissionControlEventSource source;

    // Act: a fresh Start Roam request while already Moving - deterministic
    // no-op, per the Phase 13U brief ("Do not restart the FSM
    // unnecessarily").
    source.requestStartRoam(RobotState::Moving);

    // Assert
    EXPECT_FALSE(source.pollEvent().has_value());
}

// 6: ReturnHomeQueuesReturnHomeRequested
TEST(MissionControlEventSourceTest, ReturnHomeQueuesReturnHomeRequested)
{
    // Arrange
    MissionControlEventSource source;

    // Act
    source.requestReturnHome();
    const std::optional<Event> event = source.pollEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::ReturnHomeRequested);
}

// 7: StopTaskQueuesStopTaskRequested
TEST(MissionControlEventSourceTest, StopTaskQueuesStopTaskRequested)
{
    // Arrange
    MissionControlEventSource source;

    // Act
    source.requestStopTask();
    const std::optional<Event> event = source.pollEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::StopTaskRequested);
}

// 8: RepeatedButtonRequestHandledDeterministically
TEST(MissionControlEventSourceTest, RepeatedButtonRequestHandledDeterministically)
{
    // Arrange: mashing `2` five times before any frame consumes it.
    MissionControlEventSource source;

    // Act
    source.requestReturnHome();
    source.requestReturnHome();
    source.requestReturnHome();
    source.requestReturnHome();
    source.requestReturnHome();

    // Assert: exactly one ReturnHomeRequested is ever delivered - not
    // five, matching ReturnHomeRequestSource's own established
    // idempotent-pending precedent (Phase 13T).
    const std::optional<Event> first = source.pollEvent();
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->type, EventType::ReturnHomeRequested);
    EXPECT_FALSE(source.pollEvent().has_value());

    // Same guarantee for mashing `1` while Idle - the sequence is queued
    // exactly once, never duplicated/corrupted.
    source.requestStartRoam(RobotState::Idle);
    source.requestStartRoam(RobotState::Idle);
    source.requestStartRoam(RobotState::Idle);
    const std::optional<Event> roamFirst = source.pollEvent();
    const std::optional<Event> roamSecond = source.pollEvent();
    ASSERT_TRUE(roamFirst.has_value());
    EXPECT_EQ(roamFirst->type, EventType::ScenarioLoaded);
    ASSERT_TRUE(roamSecond.has_value());
    EXPECT_EQ(roamSecond->type, EventType::StartMission);
    EXPECT_FALSE(source.pollEvent().has_value());
}

// 9: QueueOrderIsPreserved
TEST(MissionControlEventSourceTest, QueueOrderIsPreserved)
{
    // Arrange: a Start Roam request queues two events - order must be
    // ScenarioLoaded strictly before StartMission, never reversed.
    MissionControlEventSource source;
    source.requestStartRoam(RobotState::Idle);

    // Act
    const std::optional<Event> first = source.pollEvent();
    const std::optional<Event> second = source.pollEvent();

    // Assert
    ASSERT_TRUE(first.has_value());
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(first->type, EventType::ScenarioLoaded);
    EXPECT_EQ(second->type, EventType::StartMission);
}

// 10: CanStartAnotherRoamAfterPreviousQueueCompletes
TEST(MissionControlEventSourceTest, CanStartAnotherRoamAfterPreviousQueueCompletes)
{
    // Arrange: drain a full Idle Start Roam sequence.
    MissionControlEventSource source;
    source.requestStartRoam(RobotState::Idle);
    ASSERT_TRUE(source.pollEvent().has_value()); // ScenarioLoaded
    ASSERT_TRUE(source.pollEvent().has_value()); // StartMission
    ASSERT_FALSE(source.pollEvent().has_value());

    // Act: a later Start Roam request, now that the FSM would genuinely
    // be Ready (StartMission already consumed) - only StartMission is
    // needed this time.
    source.requestStartRoam(RobotState::Ready);
    const std::optional<Event> event = source.pollEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::StartMission);
    EXPECT_FALSE(source.pollEvent().has_value());
}

// ============================================================
// Phase 13X quick fix (Bug C) - TESTS: requestReturnHomeFromIdle()
// ============================================================
// Human GUI validation reproduced: launch (Idle) -> M -> manually drive
// away -> 2. Manual cleared correctly, but plain requestReturnHome() is
// unconditional and RobotStateMachine has NO Idle + ReturnHomeRequested
// transition at all (only Ready does) - the event was silently rejected
// forever, leaving "Durum: Bekliyor / Görev: YOK". requestReturnHomeFromIdle()
// composes the existing, unmodified Idle->Ready (ScenarioLoaded) and
// Ready->ReturningHome (ReturnHomeRequested) transitions, mirroring
// requestStartRoam()'s own Idle branch shape exactly.

// 11: ReturnHomeFromIdleQueuesScenarioLoadedFirst
TEST(MissionControlEventSourceTest, ReturnHomeFromIdleQueuesScenarioLoadedFirst)
{
    // Arrange
    MissionControlEventSource source;

    // Act
    source.requestReturnHomeFromIdle();
    const std::optional<Event> first = source.pollEvent();

    // Assert
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->type, EventType::ScenarioLoaded);
}

// 12: ReturnHomeFromIdleQueuesReturnHomeSecond
TEST(MissionControlEventSourceTest, ReturnHomeFromIdleQueuesReturnHomeSecond)
{
    // Arrange
    MissionControlEventSource source;
    source.requestReturnHomeFromIdle();
    ASSERT_TRUE(source.pollEvent().has_value()); // consume ScenarioLoaded

    // Act
    const std::optional<Event> second = source.pollEvent();

    // Assert
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->type, EventType::ReturnHomeRequested);
    EXPECT_FALSE(source.pollEvent().has_value());
}

// 13: ReturnHomeFromIdleSequenceNotDuplicatedOnRepeatedPress
TEST(MissionControlEventSourceTest, ReturnHomeFromIdleSequenceNotDuplicatedOnRepeatedPress)
{
    // Arrange: a sequence already in flight (mirrors mashing 2/R twice
    // before the first ScenarioLoaded has even been delivered).
    MissionControlEventSource source;
    source.requestReturnHomeFromIdle();

    // Act
    source.requestReturnHomeFromIdle();

    // Assert: exactly the original two events, never four.
    ASSERT_TRUE(source.pollEvent().has_value());
    ASSERT_TRUE(source.pollEvent().has_value());
    EXPECT_FALSE(source.pollEvent().has_value());
}
