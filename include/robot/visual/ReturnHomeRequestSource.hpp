#pragma once

#include <optional>

#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"

namespace robot::visual
{

// Tiny, visual-simulator-only IPollingEventSource (Phase 13T) that turns
// an external "R was pressed" signal into exactly one ReturnHomeRequested
// Event, then reports std::nullopt again until requested a second time -
// mirrors DemoCommandSource's own pollEvent() shape/simplicity. Exists so
// RobotSimulator3D's R key never mutates RobotStateMachine directly, nor
// calls VirtualRobotHardware::returnToBase() itself: main3d.cpp calls
// requestReturnHome() from IsKeyPressed(KEY_R), and the resulting Event is
// consumed by RobotRuntime::step() through the real FSM transition path
// (Moving + ReturnHomeRequested -> ReturningHome) exactly like every other
// Event source. Deliberately isolated to visual code - production
// CommandScript/ScriptedCommandEventSource semantics are untouched (the
// same command is also reachable there via the "return_home" script
// token, entirely independently of this class).
class ReturnHomeRequestSource : public IPollingEventSource
{
public:
    // Arms a pending ReturnHomeRequested event, to be delivered by the
    // next pollEvent() call. Calling this again before the pending event
    // is consumed has no additional effect (still exactly one queued
    // event) - matches IsKeyPressed()'s own single-frame-edge semantics,
    // so a held key can never enqueue more than one request per press.
    void requestReturnHome() { pending_ = true; }

    std::optional<Event> pollEvent() override
    {
        if (!pending_)
        {
            return std::nullopt;
        }
        pending_ = false;
        return Event{EventType::ReturnHomeRequested, 0, std::nullopt};
    }

private:
    bool pending_ = false;
};

} // namespace robot::visual
