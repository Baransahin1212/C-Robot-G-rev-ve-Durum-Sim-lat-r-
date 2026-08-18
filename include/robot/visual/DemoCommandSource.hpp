#pragma once

#include <optional>

#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"

namespace robot::visual
{

// Tiny, visual-simulator-only IPollingEventSource that delivers exactly
// two mission-lifecycle events, once each - ScenarioLoaded, then
// StartMission - then std::nullopt forever after. Exists solely so
// RobotSimulator3D's RobotRuntime has a real, valid way to drive
// RobotStateMachine from Idle through Ready into Moving via its actual
// transition rules (see docs/technical-decisions.md, Phase 13N), without
// depending on an external --command-script file or fabricating FSM
// state directly. Deliberately isolated to visual code - production
// CommandScript/ScriptedCommandEventSource semantics are untouched.
class DemoCommandSource : public IPollingEventSource
{
public:
    std::optional<Event> pollEvent() override
    {
        switch (nextEventIndex_)
        {
            case 0:
                ++nextEventIndex_;
                return Event{EventType::ScenarioLoaded, 0, std::nullopt};
            case 1:
                ++nextEventIndex_;
                return Event{EventType::StartMission, 1, std::nullopt};
            default:
                return std::nullopt;
        }
    }

private:
    int nextEventIndex_ = 0;
};

} // namespace robot::visual
