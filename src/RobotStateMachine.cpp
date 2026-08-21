#include "robot/RobotStateMachine.hpp"

namespace robot
{

RobotStateMachine::RobotStateMachine()
    : state_(RobotState::Idle)
    , resumeState_(RobotState::Idle)
    , returnHomeReason_(ReturnHomeReason::None)
{
}

RobotState RobotStateMachine::currentState() const noexcept
{
    return state_;
}

ReturnHomeReason RobotStateMachine::returnHomeReason() const noexcept
{
    return returnHomeReason_;
}

TransitionResult RobotStateMachine::processEvent(const Event& event)
{
    switch (state_)
    {
        case RobotState::Idle:
            if (event.type == EventType::ScenarioLoaded)
            {
                state_ = RobotState::Ready;
                return TransitionResult::Success;
            }
            break;

        case RobotState::Ready:
            switch (event.type)
            {
                case EventType::StartMission:
                    state_ = RobotState::Moving;
                    return TransitionResult::Success;
                case EventType::ReturnHomeRequested:
                    // Manual-validation bugfix: Ready is now also the
                    // reusable "arrived home, idling" destination a
                    // UserRequest return leaves the robot in (see the
                    // ReturningHome + HomeReached case below) - so a
                    // second Return Home request must be meaningful from
                    // here too, or the interactive R workflow could only
                    // ever be used once per session.
                    returnHomeReason_ = ReturnHomeReason::UserRequest;
                    state_ = RobotState::ReturningHome;
                    return TransitionResult::Success;
                default:
                    break;
            }
            break;

        case RobotState::Moving:
            switch (event.type)
            {
                case EventType::ObstacleDetected:
                    resumeState_ = RobotState::Moving;
                    state_ = RobotState::WaitingForObstacleClear;
                    return TransitionResult::Success;
                case EventType::StopTaskRequested:
                    // Phase 13U: user-cancelled task, not a mission
                    // failure/completion and not a fault - lands in the
                    // same reusable Ready state a user-requested Return
                    // Home arrival already uses (see
                    // ReturningHome + HomeReached below), so a new task
                    // can be assigned immediately. returnHomeReason_ is
                    // already None here (Moving never sets it) - nothing
                    // to clear.
                    state_ = RobotState::Ready;
                    return TransitionResult::Success;
                case EventType::BatteryCritical:
                    // Automatic mission-abort trigger - see
                    // ReturnHomeReason::MissionAbort's own docs.
                    returnHomeReason_ = ReturnHomeReason::MissionAbort;
                    state_ = RobotState::ReturningHome;
                    return TransitionResult::Success;
                case EventType::ReturnHomeRequested:
                    // Phase 13T: a legitimate user/operator intent to
                    // return home, distinct from BatteryCritical's
                    // automatic low-battery trigger - both lead to the
                    // same physical ReturningHome state (RobotController's
                    // State -> hardware mapping, and everything
                    // ReturningHome already does, e.g. obstacle handling
                    // below, has no notion of *why* the robot is heading
                    // home), but manual-validation bugfix: they must NOT
                    // lead to the same OUTCOME once home is reached - see
                    // returnHomeReason_ and the ReturningHome + HomeReached
                    // case below.
                    returnHomeReason_ = ReturnHomeReason::UserRequest;
                    state_ = RobotState::ReturningHome;
                    return TransitionResult::Success;
                case EventType::MissionCompleted:
                    state_ = RobotState::Completed;
                    return TransitionResult::Success;
                case EventType::EmergencyStop:
                    state_ = RobotState::EmergencyStopped;
                    return TransitionResult::Success;
                case EventType::InvalidSensorData:
                    state_ = RobotState::Error;
                    return TransitionResult::Success;
                default:
                    break;
            }
            break;

        case RobotState::WaitingForObstacleClear:
            switch (event.type)
            {
                case EventType::ObstacleCleared:
                    state_ = resumeState_;
                    return TransitionResult::Success;
                case EventType::StopTaskRequested:
                    // Phase 13U: cancels whichever task was interrupted by
                    // the obstacle pause - Roam (resumeState_ == Moving)
                    // or a Return Home (resumeState_ == ReturningHome).
                    // Either way this lands in Ready, and any in-flight
                    // Return Home reason is discarded (see the
                    // ReturningHome case's own StopTaskRequested docs
                    // below) - a stale resumeState_ is never read again
                    // afterward, since the only reader (ObstacleCleared,
                    // above) is unreachable once state_ is no longer
                    // WaitingForObstacleClear.
                    returnHomeReason_ = ReturnHomeReason::None;
                    state_ = RobotState::Ready;
                    return TransitionResult::Success;
                case EventType::EmergencyStop:
                    state_ = RobotState::EmergencyStopped;
                    return TransitionResult::Success;
                case EventType::InvalidSensorData:
                    state_ = RobotState::Error;
                    return TransitionResult::Success;
                default:
                    break;
            }
            break;

        case RobotState::ReturningHome:
            switch (event.type)
            {
                case EventType::HomeReached:
                    // Manual-validation bugfix: the destination depends on
                    // WHY the robot returned home, not merely that it did.
                    // MissionAbort (e.g. BatteryCritical) preserves the
                    // original, pre-existing "mission ended abnormally"
                    // outcome - Aborted, unchanged. UserRequest (the
                    // explicit `R` command) is not a mission failure - the
                    // robot is simply safely parked at base with no
                    // mission actively running, so it lands in Ready, the
                    // same non-terminal "loaded, idling, can accept a new
                    // StartMission or ReturnHomeRequested" state
                    // ScenarioLoaded already produces - reusing it here
                    // (rather than inventing a new state) means a second
                    // Return Home request, or a fresh mission, both work
                    // immediately with no other code changes. The reason
                    // is cleared on every exit from ReturningHome via
                    // HomeReached so it can never leak into a later,
                    // unrelated mission.
                    state_ = (returnHomeReason_ == ReturnHomeReason::UserRequest) ? RobotState::Ready
                                                                                   : RobotState::Aborted;
                    returnHomeReason_ = ReturnHomeReason::None;
                    return TransitionResult::Success;
                case EventType::ObstacleDetected:
                    resumeState_ = RobotState::ReturningHome;
                    state_ = RobotState::WaitingForObstacleClear;
                    return TransitionResult::Success;
                case EventType::StopTaskRequested:
                    // Phase 13U: cancels an in-progress Return Home
                    // (whether user-requested or an automatic mission-
                    // abort) - the user gets the robot back under their
                    // control immediately. returnHomeReason_ must be
                    // cleared here: otherwise it would stay set (e.g.
                    // UserRequest) while sitting in Ready with no active
                    // task, violating returnHomeReason()'s own documented
                    // invariant ("Valid only while currentState() ==
                    // ReturningHome; None otherwise") and leaving a stale
                    // value for a Full-HUD "Return reason" line to
                    // display incorrectly.
                    returnHomeReason_ = ReturnHomeReason::None;
                    state_ = RobotState::Ready;
                    return TransitionResult::Success;
                case EventType::EmergencyStop:
                    state_ = RobotState::EmergencyStopped;
                    return TransitionResult::Success;
                case EventType::InvalidSensorData:
                    state_ = RobotState::Error;
                    return TransitionResult::Success;
                default:
                    break;
            }
            break;

        case RobotState::EmergencyStopped:
            if (event.type == EventType::Reset)
            {
                // Manual-validation bugfix: EmergencyStop can interrupt an
                // active ReturningHome before HomeReached is ever
                // consumed, leaving returnHomeReason_ set - Reset must not
                // let that stale reason leak into a later, unrelated
                // mission.
                returnHomeReason_ = ReturnHomeReason::None;
                state_ = RobotState::Idle;
                return TransitionResult::Success;
            }
            break;

        case RobotState::Error:
            if (event.type == EventType::Reset)
            {
                // See EmergencyStopped's own Reset case above.
                returnHomeReason_ = ReturnHomeReason::None;
                state_ = RobotState::Idle;
                return TransitionResult::Success;
            }
            break;

        case RobotState::Completed:
        case RobotState::Aborted:
            // Terminal states for now; no transitions defined out of them.
            break;
    }

    return TransitionResult::InvalidTransition;
}

} // namespace robot
