#include <gtest/gtest.h>

#include <optional>
#include <vector>

#include "robot/Event.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/IPollingEventSource.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/SimulatedRobotHardware.hpp"
#include "robot/Simulator.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::HardwareEventSource;
using robot::IPollingEventSource;
using robot::RobotCommand;
using robot::RobotState;
using robot::RobotStateMachine;
using robot::SimulatedRobotHardware;
using robot::Simulator;
using robot::SimulationResult;

} // namespace

TEST(HardwareEventSourceTest, SafeInitialHardwareProducesNoEvent)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);

    // Act
    const std::optional<Event> event = source.nextEvent();

    // Assert
    EXPECT_FALSE(event.has_value());
}

TEST(HardwareEventSourceTest, ObstacleRisingEdgeProducesObstacleDetected)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    hardware.setObstacleDetected(true);

    // Act
    const std::optional<Event> event = source.nextEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::ObstacleDetected);
}

TEST(HardwareEventSourceTest, StableObstacleDoesNotRepeatEvent)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    hardware.setObstacleDetected(true);
    ASSERT_TRUE(source.nextEvent().has_value()); // consume the rising edge

    // Act: obstacle remains true, nothing changed
    const std::optional<Event> event = source.nextEvent();

    // Assert
    EXPECT_FALSE(event.has_value());
}

TEST(HardwareEventSourceTest, ObstacleClearProducesClearEvent)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    hardware.setObstacleDetected(true);
    ASSERT_TRUE(source.nextEvent().has_value()); // consume the rising edge

    // Act
    hardware.setObstacleDetected(false);
    const std::optional<Event> event = source.nextEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::ObstacleCleared);
}

TEST(HardwareEventSourceTest, CriticalBatteryThresholdCrossingProducesBatteryEvent)
{
    // Arrange: default battery is 100, well above the 20% threshold.
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    hardware.setBatteryLevelPercent(15);

    // Act
    const std::optional<Event> event = source.nextEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::BatteryCritical);
    ASSERT_TRUE(event->value.has_value());
    EXPECT_DOUBLE_EQ(*event->value, 15.0);
}

TEST(HardwareEventSourceTest, CriticalBatteryDoesNotRepeatWhileStillCritical)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    hardware.setBatteryLevelPercent(15);
    ASSERT_TRUE(source.nextEvent().has_value()); // consume the crossing edge

    // Act: still below threshold, even though the exact value changed
    hardware.setBatteryLevelPercent(10);
    const std::optional<Event> event = source.nextEvent();

    // Assert
    EXPECT_FALSE(event.has_value());
}

TEST(HardwareEventSourceTest, EmergencyStopPressProducesEmergencyEvent)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    hardware.setEmergencyStopPressed(true);

    // Act
    const std::optional<Event> event = source.nextEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::EmergencyStop);
}

TEST(HardwareEventSourceTest, EmergencyStopDoesNotRepeatWhileHeld)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    hardware.setEmergencyStopPressed(true);
    ASSERT_TRUE(source.nextEvent().has_value()); // consume the rising edge

    // Act: still pressed, nothing changed
    const std::optional<Event> event = source.nextEvent();

    // Assert
    EXPECT_FALSE(event.has_value());
}

TEST(HardwareEventSourceTest, MultipleSimultaneousEdgesRespectPriority)
{
    // Arrange: all three hazards become active between samples.
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    hardware.setEmergencyStopPressed(true);
    hardware.setBatteryLevelPercent(15);
    hardware.setObstacleDetected(true);

    // Act / Assert: emergency stop, then battery, then obstacle.
    const std::optional<Event> first = source.nextEvent();
    ASSERT_TRUE(first.has_value());
    EXPECT_EQ(first->type, EventType::EmergencyStop);

    const std::optional<Event> second = source.nextEvent();
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(second->type, EventType::BatteryCritical);

    const std::optional<Event> third = source.nextEvent();
    ASSERT_TRUE(third.has_value());
    EXPECT_EQ(third->type, EventType::ObstacleDetected);

    const std::optional<Event> fourth = source.nextEvent();
    EXPECT_FALSE(fourth.has_value());
}

TEST(HardwareEventSourceTest, PendingEventsAreNotLost)
{
    // Arrange: same simultaneous-edge setup as the priority test, but this
    // test's focus is that draining fully yields exactly the three queued
    // events - none silently dropped by a later re-sample.
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    hardware.setEmergencyStopPressed(true);
    hardware.setBatteryLevelPercent(15);
    hardware.setObstacleDetected(true);

    // Act
    std::vector<EventType> observed;
    while (const std::optional<Event> event = source.nextEvent())
    {
        observed.push_back(event->type);
    }

    // Assert
    const std::vector<EventType> expected{
        EventType::EmergencyStop,
        EventType::BatteryCritical,
        EventType::ObstacleDetected,
    };
    EXPECT_EQ(observed, expected);
}

TEST(HardwareEventSourceTest, HardwareEventSourceNeverChangesActuatorCommand)
{
    // Arrange
    SimulatedRobotHardware hardware;
    hardware.moveForward();
    HardwareEventSource source(hardware);

    // Act: drive several sensor edges through the source.
    hardware.setEmergencyStopPressed(true);
    hardware.setBatteryLevelPercent(15);
    hardware.setObstacleDetected(true);
    while (source.nextEvent().has_value())
    {
    }

    // Assert
    EXPECT_EQ(hardware.currentCommand(), RobotCommand::MovingForward);
}

TEST(HardwareEventSourceTest, SafeSensorChangesDoNotProduceEvents)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);

    // Act: battery drops but stays well above the 20% threshold.
    hardware.setBatteryLevelPercent(80);
    const std::optional<Event> event = source.nextEvent();

    // Assert
    EXPECT_FALSE(event.has_value());
}

// --- IPollingEventSource conformance (Phase 13F) ---
//
// Compile/runtime proof that HardwareEventSource correctly implements the
// new polling interface too, addressed purely through an
// IPollingEventSource& - not just IEventSource&.
TEST(HardwareEventSourceTest, CanBeUsedThroughIPollingEventSourceInterface)
{
    // Arrange
    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    IPollingEventSource& poller = source;
    hardware.setObstacleDetected(true);

    // Act
    const std::optional<Event> event = poller.pollEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::ObstacleDetected);
}

// --- Simulator integration ---
//
// Proves HardwareEventSource satisfies IEventSource end-to-end: driving
// RobotStateMachine into Moving via its existing processEvent() API (not
// through HardwareEventSource, which has no mission-lifecycle events of
// its own), then letting a single obstacle-detected sensor edge reach the
// FSM through Simulator exactly the way a JsonScenarioSource event would.
TEST(HardwareEventSourceTest, DrivesSimulatorFromObstacleSensorEdge)
{
    // Arrange
    RobotStateMachine machine;
    machine.processEvent(Event{EventType::ScenarioLoaded, 0, std::nullopt});
    machine.processEvent(Event{EventType::StartMission, 1, std::nullopt});
    ASSERT_EQ(machine.currentState(), RobotState::Moving);

    SimulatedRobotHardware hardware;
    HardwareEventSource source(hardware);
    hardware.setObstacleDetected(true);

    Simulator simulator(source, machine);

    // Act
    const SimulationResult result = simulator.run();

    // Assert
    EXPECT_EQ(result.finalState, RobotState::WaitingForObstacleClear);
    EXPECT_EQ(result.eventsProcessed, 1u);
    EXPECT_EQ(result.successfulTransitions, 1u);
    EXPECT_EQ(result.rejectedTransitions, 0u);
}
