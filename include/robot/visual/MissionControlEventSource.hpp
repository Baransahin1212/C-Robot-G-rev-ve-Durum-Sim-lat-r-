#pragma once

#include <deque>
#include <optional>

#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"
#include "robot/RobotState.hpp"

namespace robot::visual
{

// Phase 13U: the single, raylib-free event source behind every explicit
// Mission Control command (keys 1/2/3/R, and by extension the CommandScript
// tokens they mirror) - turns a high-level user TASK intent into the
// existing/new domain Event(s) RobotRuntime already knows how to consume,
// never a direct RobotStateMachine mutation or a direct IRobotHardware
// call. `2` and `R` are deliberately the SAME intent
// (requestReturnHome()) - there is exactly one Return Home implementation,
// not two competing ones.
//
// Mirrors ReturnHomeRequestSource's own simple, edge-triggered shape for
// the single-event intents; Start Roam is the one intent that can require
// TWO chained events (Idle needs ScenarioLoaded then StartMission - see
// requestStartRoam() below), so this class owns a small internal FIFO
// queue rather than a single pending flag. RobotRuntime::step() still
// only ever consumes at most one Event per call (pollEvent() only ever
// returns one), so a two-event Start Roam request naturally spans two
// separate rendered frames - never forced through in one frame, and never
// requiring more than the mandatory one RobotRuntime::step() per frame.
class MissionControlEventSource : public IPollingEventSource
{
public:
    // Requests the "wander the tabletop" task. `currentState` is the
    // caller's up-to-date RobotStateMachine::currentState() snapshot,
    // read at the moment of the request (main3d.cpp reads it right
    // before calling this, from the KEY_ONE handler) - this class holds
    // no RobotStateMachine reference of its own, matching every other
    // event source in this codebase (none of them read FSM state
    // directly).
    //
    //   Idle    -> queues ScenarioLoaded, then StartMission (two events,
    //              delivered on two separate pollEvent() calls).
    //   Ready   -> queues only StartMission (Ready already accepts it
    //              directly - see RobotStateMachine::processEvent()).
    //   anything else (already Moving, ReturningHome, ...) -> a
    //              deterministic no-op: nothing is queued, since neither
    //              ScenarioLoaded nor StartMission would be accepted from
    //              any of those states anyway - the FSM is never
    //              "restarted" by re-queueing events doomed to be
    //              rejected.
    //
    // A Start Roam sequence already in flight (either event still
    // pending) is never re-queued on top of itself - repeatedly pressing
    // `1` before the previous request finishes delivering is a no-op,
    // not a growing queue.
    void requestStartRoam(RobotState currentState);

    // Requests Return Home - queues ReturnHomeRequested exactly once,
    // regardless of current state (the FSM itself already correctly
    // accepts or rejects it - see RobotStateMachine::processEvent()'s
    // Moving/Ready cases). Shared by both `2` and `R` in main3d.cpp; a
    // request already pending and not yet delivered is not duplicated.
    void requestReturnHome();

    // Requests the current task be cancelled - queues StopTaskRequested
    // exactly once, regardless of current state, for the same reason as
    // requestReturnHome() above. A request already pending is not
    // duplicated.
    void requestStopTask();

    // Returns the next queued Event, or std::nullopt if nothing is
    // pending - never more than one per call, exactly like every other
    // IPollingEventSource in this codebase.
    std::optional<Event> pollEvent() override;

private:
    bool alreadyQueued(EventType type) const;
    void requestSingle(EventType type);

    std::deque<EventType> pendingEvents_;
};

} // namespace robot::visual
