#pragma once

#include <optional>

#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"

namespace robot
{

// Combines two IPollingEventSource instances into one, so RobotRuntime -
// which accepts exactly one IPollingEventSource - can be driven by both a
// command source and a sensor source at once (Phase 13J). Deterministic
// priority: a pending command event is always returned before a pending
// sensor event, within the same pollEvent() call.
//
// Rationale for command-before-sensor priority: at cycle 0, a scripted
// ScenarioLoaded command may need to move Idle -> Ready before any sensor
// edge from that same cycle is considered; at cycle 1, StartMission may
// need to move Ready -> Moving before a sensor event scheduled at cycle 1
// is processed. Consistently favoring the command source avoids an
// ordering-dependent race between the two independently-scheduled scripts.
//
// This class has no knowledge of cycles, IRobotHardware, CommandScript, or
// SensorScript - it only ever calls pollEvent() on the two sources it was
// given and forwards the first non-nullopt result, exactly once per call
// of its own pollEvent() (RobotRuntime still processes at most one Event
// per step() - this class never drains more than one event from either
// source in a single call).
class CompositePollingEventSource : public IPollingEventSource
{
public:
    // commandSource and hardwareSource must both outlive this object -
    // the same non-owning-reference pattern used throughout this codebase.
    CompositePollingEventSource(IPollingEventSource& commandSource, IPollingEventSource& hardwareSource);

    // Returns commandSource's next event if it has one pending; otherwise
    // hardwareSource's next event if it has one pending; otherwise
    // std::nullopt. Never polls hardwareSource when commandSource already
    // produced an event for this call.
    std::optional<Event> pollEvent() override;

private:
    IPollingEventSource& commandSource_;
    IPollingEventSource& hardwareSource_;
};

} // namespace robot
