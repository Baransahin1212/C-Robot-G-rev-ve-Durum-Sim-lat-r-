#pragma once

#include <string_view>

#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"

namespace robot::visual
{

// Phase 13U: user-facing task status - what Mission Control's panel
// displays, and what HomeZoneMonitor uses to decide whether it is
// currently relevant (Home Zone only applies to Roam). This is
// deliberately NOT a second state machine: it carries no state of its
// own and grants no transition authority - RobotStateMachine remains the
// only thing that actually decides what the robot does. MissionTask is
// derived fresh every frame from RobotStateMachine's own already-public
// state (see deriveMissionTask() below), never stored/mutated
// independently, so it can never drift out of sync with the real FSM.
enum class MissionTask
{
    None,
    Roam,
    ReturnHome
};

constexpr std::string_view toString(MissionTask task) noexcept
{
    switch (task)
    {
        case MissionTask::None: return "NONE";
        case MissionTask::Roam: return "ROAM";
        case MissionTask::ReturnHome: return "RETURN HOME";
    }
    return "Unknown";
}

// Derives the current user-facing task from RobotStateMachine's own
// public state - the cleanest option after auditing the available FSM
// API, since it needs no new RobotStateMachine surface at all:
//   Moving                  -> Roam (RobotController maps this to
//                               MoveForward - "wandering" is exactly
//                               what a plain forward FSM command plus
//                               the existing reactive layers already
//                               produces, see docs/technical-decisions.md)
//   ReturningHome            -> ReturnHome
//   WaitingForObstacleClear  -> whichever task the obstacle interrupted.
//                               reason == None can only happen if the
//                               interrupted task was Moving (Moving never
//                               sets returnHomeReason_); reason != None
//                               means the interrupted task was
//                               ReturningHome (both MissionAbort and
//                               UserRequest survive the obstacle pause
//                               unchanged - see
//                               ReturnReasonSurvivesObstacleWaitResume).
//                               This reuses returnHomeReason() instead of
//                               requiring a new resumeState_ getter.
//   everything else          -> None (Idle/Ready/Completed/Aborted/
//                               EmergencyStopped/Error: no task is
//                               actively running)
constexpr MissionTask deriveMissionTask(RobotState state, ReturnHomeReason reason) noexcept
{
    switch (state)
    {
        case RobotState::Moving:
            return MissionTask::Roam;
        case RobotState::ReturningHome:
            return MissionTask::ReturnHome;
        case RobotState::WaitingForObstacleClear:
            return (reason == ReturnHomeReason::None) ? MissionTask::Roam : MissionTask::ReturnHome;
        default:
            return MissionTask::None;
    }
}

} // namespace robot::visual
