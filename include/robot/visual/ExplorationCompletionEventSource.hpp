#pragma once

#include <optional>

#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"

namespace robot::visual
{

// Phase 13X: the small signal ExplorationCompletionEventSource observes -
// a plain bool the caller (main3d.cpp's exploration loop) updates
// whenever it evaluates frontier availability, never every single frame
// (this phase's own brief: "Do NOT re-run A* every frame" - `complete`
// should be derived from the SAME low-frequency
// FrontierExplorer::selectTarget()/completion() call the exploration loop
// already needs for target selection, never a second, dedicated per-frame
// completion check). True means "actively exploring (MissionTask::Roam)
// AND no reachable frontier remains" - see
// ExplorationCompletionEventSource's own class docs for why this is the
// correct, minimal signal to latch on.
struct ExplorationCompletionSignal
{
    bool complete = false;
};

// Turns "autonomous mapping has nothing reachable left to explore" into
// exactly one ReturnHomeRequested Event (Phase 13X) - edge-triggered on
// ExplorationCompletionSignal::complete going false -> true, mirroring
// HomeArrivalEventSource/WaypointArrivalEventSource's own edge-triggered
// shape exactly.
//
// Deliberately reuses the EXISTING ReturnHomeRequested EventType/
// ReturnHomeReason::UserRequest semantics rather than introducing a new
// Event/reason - see docs/technical-decisions.md (Phase 13X, "mapping-
// complete auto-return semantics") for the full audit: MissionAbort would
// incorrectly classify a successfully COMPLETED mapping session as a
// mission failure (RobotStateMachine's own ReturningHome + HomeReached
// case sends MissionAbort to the terminal Aborted state - wrong for a
// success), while UserRequest's existing "arrive, then land back in the
// reusable Ready state" outcome is EXACTLY the right terminal behavior for
// a completed mapping session (a fresh Start Haritalama/Return Home can
// be issued immediately afterward, precisely like every other UserRequest
// arrival) - achieved with ZERO RobotStateMachine/ReturnHomeReason
// changes, per this phase's own "prefer zero core FSM changes" brief.
//
// Never fires while `signal.complete` is false (e.g. outside Roam, or
// while frontiers remain) - the caller's own ExplorationCompletionSignal
// naturally re-arms this source the next time complete legitimately
// becomes true again (a new mapping session after Stop Task, or a
// resumed/loaded map that still has reachable frontiers) - no manual
// reset needed by any caller, exactly like HomeArrivalEventSource's own
// re-arming contract.
class ExplorationCompletionEventSource : public IPollingEventSource
{
public:
    // `signal` must outlive this object - the same non-owning-reference
    // pattern every other event source in this codebase uses.
    explicit ExplorationCompletionEventSource(const ExplorationCompletionSignal& signal) noexcept;

    std::optional<Event> pollEvent() override;

private:
    const ExplorationCompletionSignal& signal_;
    bool wasComplete_ = false;
};

} // namespace robot::visual
