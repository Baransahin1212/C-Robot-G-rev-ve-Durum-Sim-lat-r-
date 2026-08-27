#pragma once

#include <optional>

#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"

namespace robot::visual
{

// Phase 13X final-approach fix ("FINAL APPROACH HAZARD CONTRACT"): the
// narrowest possible dock-specific exception to obstacle-triggered
// RobotStateMachine pausing - never a change to obstacle SENSING itself
// (VirtualRobotHardware::obstacleDetected() keeps reading the real,
// unmodified geometry), never a change to ReactiveObstacleAvoidance,
// Safety, or any other task's obstacle handling, and never a new
// RobotStateMachine transition.
//
// THE PROBLEM this closes: correctly docking necessarily brings the robot
// physically close to the dock's own rear housing - a real, permanent
// obstacle. The instant the forward sensor detects it (an entirely
// accurate reading), HardwareEventSource's ObstacleDetected would move
// RobotStateMachine into WaitingForObstacleClear, PAUSING the ReturnHome
// task. That pause has no path back out on its own here: RobotStateMachine
// only leaves WaitingForObstacleClear via ObstacleCleared (the robot
// physically moving away from what it detected) or an explicit user
// command - but a robot correctly parked at the dock is never going to
// move away on its own, and DockApproachController's own avoidance-trigger
// suppression (see main3d.cpp's own `dockApproachActive` docs) only ever
// prevented a reactive TURN, not the FSM pause itself, since HardwareEventSource
// has composite-polling PRIORITY over the completion-arrival group (see
// main3d.cpp's own event-source composition comment) - the ObstacleDetected
// edge reaches RobotStateMachine before this fix, a genuine traced deadlock
// (ReturningHome/WaitingForObstacleClear, HomeReached perpetually rejected,
// zero wheels forever).
//
// THE FIX: wraps an inner IPollingEventSource (production: HardwareEventSource)
// and, while `suppressed` is true, silently discards ONLY an
// EventType::ObstacleDetected result - every other event (ObstacleCleared,
// BatteryCritical, EmergencyStop, and anything else this source ever
// produces) passes through completely unchanged. The wrapped source's own
// internal edge-tracking is untouched by this (HardwareEventSource updates
// its own "was detected" bookkeeping the moment pollEvent() is CALLED,
// regardless of what the caller does with the result - see that class's
// own sampleAndEnqueueEdges()), so a genuine, later obstacle encounter
// (e.g. after `M` ends the dock session and the robot drives elsewhere) is
// still correctly detected - this filter only ever discards the exact
// edge(s) raised while the caller has explicitly marked the robot as
// currently executing the validated docking lane.
class DockLaneObstacleFilter : public IPollingEventSource
{
public:
    explicit DockLaneObstacleFilter(IPollingEventSource& inner) noexcept;

    // Set once per frame by the caller (main3d.cpp), BEFORE RobotRuntime::step()
    // - the same sticky-read-from-end-of-previous-frame convention this
    // file's own `dockApproachActive` already uses elsewhere, since
    // DockApproachController's own state for the CURRENT frame is not
    // computed until later in the frame.
    void setSuppressed(bool suppressed) noexcept;

    std::optional<Event> pollEvent() override;

private:
    IPollingEventSource& inner_;
    bool suppressed_ = false;
};

} // namespace robot::visual
