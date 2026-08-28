#pragma once

#include "robot/RobotState.hpp"

namespace robot::visual
{

// Phase 13Z: what a single fresh `N` ("Yeni Harita") keypress should do
// this frame, decided by MapResetController::requestKeyPress() below -
// main3d.cpp's keyboard handler only ever reacts to this result, it never
// re-derives the confirmation/state-gate logic itself (mirrors
// MissionControlEventSource's own "the small class decides, main3d.cpp
// only reacts" shape).
enum class MapResetRequestOutcome
{
    // First press (or a press after the previous confirmation window
    // expired) while the robot is Idle/Ready - a confirmation is now
    // pending; no reset performed yet.
    ConfirmationRequested,

    // The robot is not currently Idle/Ready (an active mission is
    // running) - the request is refused outright, any previously-pending
    // confirmation is cancelled, and no reset is performed.
    RejectedActiveMission,

    // This is the CONFIRMING second press, within the confirmation
    // window, while the robot is (still) Idle/Ready - the caller must now
    // actually perform the reset (delete the persisted file, reset the
    // in-memory session - see ExplorationSessionReset.hpp).
    Confirmed,
};

// Deterministic, raylib-free two-press confirmation gate for New Map
// (`N`) - owns no ExplorationMap/CoverageTrail/WaypointNavigator state
// itself (see ExplorationSessionReset.hpp for the actual reset operation);
// this class only ever answers "should a reset happen in response to this
// keypress, and does the robot's current state even allow one." Map reset
// is deliberately kept an application/session-level operation, never a
// RobotStateMachine event (this phase's own brief) - hence this class only
// ever READS RobotState, it never mutates or drives the FSM.
class MapResetController
{
public:
    // How long a first `N` press's confirmation stays armed before it is
    // silently cancelled - a named constant per this phase's own brief,
    // never a magic literal at each call site.
    static constexpr float kConfirmationWindowSeconds = 3.0F;

    // Advances the pending-confirmation countdown by one frame's elapsed
    // time - call exactly once per rendered frame, unconditionally
    // (mirrors every other stateful per-frame controller in this
    // codebase, e.g. ReactiveObstacleAvoidance::update()). A no-op while
    // no confirmation is currently pending.
    void update(float deltaSeconds) noexcept;

    // Call exactly once for a fresh, edge-triggered `N` keypress (the
    // caller is responsible for gating on IsKeyPressed(), never calling
    // this every frame a key is merely held - mirrors every other
    // keyboard handler in main3d.cpp). `state` is the robot's CURRENT
    // RobotState. Re-validates the Idle/Ready gate on EVERY call
    // (including the confirming second press), so a mission that starts
    // while a confirmation is pending correctly cancels it, rather than
    // letting a stale confirmation silently reset the map underneath an
    // active mission.
    MapResetRequestOutcome requestKeyPress(RobotState state) noexcept;

    // Whether a confirmation is currently armed, awaiting either a
    // confirming second press or the window expiring - drives the
    // "Onaylamak için N'ye tekrar basın" HUD notice.
    bool confirmationPending() const noexcept;

private:
    bool confirmationPending_ = false;
    float elapsedSinceFirstPressSeconds_ = 0.0F;
};

} // namespace robot::visual
