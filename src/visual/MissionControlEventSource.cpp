#include "robot/visual/MissionControlEventSource.hpp"

#include <algorithm>

namespace robot::visual
{

void MissionControlEventSource::requestStartRoam(RobotState currentState)
{
    if (alreadyQueued(EventType::ScenarioLoaded) || alreadyQueued(EventType::StartMission))
    {
        // A Start Roam sequence is already in flight - do not restart or
        // duplicate it (Phase 13U brief: "Do not restart the FSM
        // unnecessarily").
        return;
    }

    switch (currentState)
    {
        case RobotState::Idle:
            pendingEvents_.push_back(EventType::ScenarioLoaded);
            pendingEvents_.push_back(EventType::StartMission);
            break;
        case RobotState::Ready:
            pendingEvents_.push_back(EventType::StartMission);
            break;
        default:
            // Already Moving (or any other state) - neither
            // ScenarioLoaded nor StartMission would be accepted from
            // here, so nothing is queued at all: a deterministic no-op,
            // never a doomed-to-be-rejected event.
            break;
    }
}

void MissionControlEventSource::requestScenarioLoadedOnly(RobotState currentState)
{
    if (currentState == RobotState::Idle)
    {
        requestSingle(EventType::ScenarioLoaded);
    }
}

void MissionControlEventSource::requestReturnHome()
{
    requestSingle(EventType::ReturnHomeRequested);
}

void MissionControlEventSource::requestReturnHomeFromIdle()
{
    if (alreadyQueued(EventType::ScenarioLoaded) || alreadyQueued(EventType::ReturnHomeRequested))
    {
        return;
    }
    pendingEvents_.push_back(EventType::ScenarioLoaded);
    pendingEvents_.push_back(EventType::ReturnHomeRequested);
}

void MissionControlEventSource::requestStopTask()
{
    requestSingle(EventType::StopTaskRequested);
}

std::optional<Event> MissionControlEventSource::pollEvent()
{
    if (pendingEvents_.empty())
    {
        return std::nullopt;
    }

    const EventType next = pendingEvents_.front();
    pendingEvents_.pop_front();
    return Event{next, 0, std::nullopt};
}

bool MissionControlEventSource::alreadyQueued(EventType type) const
{
    return std::find(pendingEvents_.begin(), pendingEvents_.end(), type) != pendingEvents_.end();
}

void MissionControlEventSource::requestSingle(EventType type)
{
    if (!alreadyQueued(type))
    {
        pendingEvents_.push_back(type);
    }
}

} // namespace robot::visual
