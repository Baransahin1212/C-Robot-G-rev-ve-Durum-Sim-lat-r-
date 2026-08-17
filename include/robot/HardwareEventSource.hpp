#pragma once

#include <cstdint>
#include <deque>
#include <optional>

#include "robot/Event.hpp"
#include "robot/IEventSource.hpp"
#include "robot/IRobotHardware.hpp"

namespace robot
{

// Converts IRobotHardware sensor state into the same Event vocabulary
// JsonScenarioSource produces (ObstacleDetected/ObstacleCleared/
// BatteryCritical/EmergencyStop), so Simulator/RobotStateMachine can be
// driven by either source interchangeably through IEventSource.
// HardwareEventSource only reads sensor-like IRobotHardware methods
// (batteryLevelPercent/obstacleDetected/emergencyStopPressed) - it never
// calls an actuator method. It produces Events, it does not command the
// robot; that remains RobotController's job.
//
// Edge-triggered: each nextEvent() call compares the current sensor
// snapshot against the previously observed one and emits an event only for
// a state *change*, never for a condition that merely persists. There is
// no EventType for "emergency stop released" or "battery recovered" in the
// existing vocabulary, so those transitions are tracked internally (to
// correctly detect the *next* rising edge) but produce no Event -
// ObstacleCleared is the one recovery-direction event that does exist, and
// is emitted when obstacleDetected() goes true -> false.
//
// When more than one sensor condition becomes newly active between calls,
// all of them are queued (never silently dropped) and drained one at a
// time in a fixed safety priority: emergency stop, then critical battery,
// then obstacle. A fresh sensor snapshot is only taken once the queue is
// empty.
//
// This is a finite, snapshot-driven adapter, not a continuous polling
// service: nextEvent() returns std::nullopt whenever there is currently
// nothing new to report, which - per IEventSource's contract - looks like
// source exhaustion to Simulator::run(). That is intentional for Phase
// 13E; see docs/technical-decisions.md for why continuous real-time
// polling is a deliberately separate, later concern.
class HardwareEventSource : public IEventSource
{
public:
    explicit HardwareEventSource(IRobotHardware& hardware);

    std::optional<Event> nextEvent() override;

private:
    // No existing documented threshold; 20% is this project's first
    // definition of "critical" battery, kept as a named constant rather
    // than a magic number so it is easy to find and change.
    static constexpr int kCriticalBatteryPercent = 20;

    void sampleAndEnqueueEdges();
    Event makeEvent(EventType type, std::optional<double> value = std::nullopt);

    IRobotHardware& hardware_;
    std::deque<Event> pendingEvents_;
    std::uint64_t nextTimestampMs_ = 0;

    // Defaulting these to false means the very first sample is compared
    // against an implicit "all clear" baseline, so an already-active
    // hazard at startup (e.g. emergency stop already pressed) is treated
    // as a rising edge and surfaced immediately, rather than silently
    // ignored - without needing any separate first-call special case.
    bool wasObstacleDetected_ = false;
    bool wasBatteryCritical_ = false;
    bool wasEmergencyStopPressed_ = false;
};

} // namespace robot
