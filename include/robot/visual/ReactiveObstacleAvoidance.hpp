#pragma once

#include <optional>
#include <string_view>

#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/RobotCollision.hpp"

namespace robot::visual
{

// Phase 13V human-validation fix: the three internal phases one avoidance
// incident moves through - see ReactiveObstacleAvoidance's own class docs
// below for the full "why" (the Phase 13R latch alone was not sufficient:
// releasing the instant the sensor/corridor reads clear let HomeNavigator
// immediately re-aim back into the very obstacle avoidance had only
// ROTATED away from, without ever having TRANSLATED past it). `active()`
// is true for both TurnAway and AdvanceClear - main3d.cpp's own "does
// avoidance currently want the wheels" caller contract is unchanged.
enum class AvoidanceState
{
    Inactive,
    TurnAway,
    AdvanceClear
};

// Visual-only, not part of any FSM/RobotState convention - mirrors every
// other visual-simulation enum's own toString() shape in this codebase.
constexpr std::string_view toString(AvoidanceState state) noexcept
{
    switch (state)
    {
        case AvoidanceState::Inactive: return "Inactive";
        case AvoidanceState::TurnAway: return "TurnAway";
        case AvoidanceState::AdvanceClear: return "AdvanceClear";
    }
    return "Unknown";
}

// Phase 13V human-validation fix: the per-ray hazard snapshot
// ReactiveObstacleAvoidance uses ONLY to choose a turn direction, once,
// when a new incident begins - deliberately a small plain struct (matching
// this codebase's RangeObservation precedent) rather than a dependency on
// VirtualObstacleSensorArray.hpp/ObstacleSensorArrayReadings directly, so
// this class still knows nothing about VirtualDistanceSensor,
// VirtualObstacleSensorArray, ForwardClearanceProbe, or VirtualWorld - the
// caller (main3d.cpp) repackages already-computed
// VirtualObstacleSensorArray::readings() distances into this shape, never
// duplicating the ray/AABB math itself. `std::nullopt` means that ray/side
// reported no hit, matching ObstacleSensorArrayReadings' own
// std::optional<float> convention exactly.
struct ObstacleHazardSample
{
    std::optional<float> leftDistance;
    std::optional<float> centerDistance;
    std::optional<float> rightDistance;
};

// Deterministic, raylib-free reactive obstacle-avoidance policy - a
// stateful three-phase incident lifecycle (Phase 13V human-validation
// fix), replacing the Phase 13R two-phase latch (Inactive/active-with-a-
// single-fixed-turn-direction).
//
// THE BUG THIS FIXES (human GUI validation, Phase 13V): pressing `2`/`R`
// (Return Home) with an obstacle sitting close to the direct line to base
// made the robot oscillate in place indefinitely - turning, "releasing"
// the instant ForwardClearanceProbe reported clear, HomeNavigator
// immediately re-aiming (it recomputes its target from the CURRENT pose
// every single frame - see HomeNavigator.hpp) back toward the still-
// nearby obstacle, re-triggering avoidance, forever, with zero net
// translation. Root cause: "the forward corridor reads clear along the
// robot's CURRENT heading" is a fact about ROTATION, not about whether the
// robot has physically TRANSLATED far enough to no longer be sitting
// right next to the obstacle's footprint - Phase 13R's latch already knew
// the difference between "a ray is clear" and "the body corridor is
// clear," but never distinguished "the corridor is clear" from "the robot
// has actually moved past the obstacle." This class now tracks that third,
// separate fact explicitly (AdvanceClear's own distance-travelled
// bookkeeping below).
//
// THE FIX: TurnAway (rotate only, zero linear velocity, exactly like the
// old single-phase latch) -> AdvanceClear (drive forward once the corridor
// is clear, tracking real displacement from where AdvanceClear began) ->
// Inactive, only once BOTH the corridor is (still) clear AND the robot has
// translated at least kMinimumBypassDistanceWorldUnits. Re-blocked during
// AdvanceClear returns to TurnAway, preserving (never recomputing) the
// turn direction chosen for this incident. Still REACTIVE avoidance only -
// not pathfinding, not A*, not waypoint planning, not SLAM/mapping, not
// full navigation: this remains local, single-obstacle bypass, with no
// memory of any incident once it ends.
//
// PHASE 13X BLOCKER FIX (bounded TurnAway sweep): this class's own prior
// docs acknowledged "no guaranteed solution for two or more obstacles
// forming an actual enclosure" as an accepted V1 limitation - reproduced
// deterministically as a genuine defect once autonomous frontier
// exploration (Phase 13X) started driving the robot into more varied desk
// positions than plain undirected Roam ever visited: TurnAway rotated
// >700 degrees (multiple full turns) with zero net translation, because
// `forwardCorridorClear` never became true at ANY heading sampled during
// the sweep (see docs/technical-decisions.md, Phase 13X blocker fix, for
// the full recorded reproduction - pose, hazard distances, accumulated
// rotation). Root architectural rule: local reactive avoidance must never
// try to solve geometry that actually requires a GLOBAL route change -
// that responsibility now belongs to GridPathPlanner + WaypointNavigator
// (Phase 13X). TurnAway therefore now tracks accumulated ABSOLUTE heading
// rotation for the CURRENT incident (reset only when a new incident
// begins or the incident fully releases - never on an AdvanceClear ->
// TurnAway re-block within the SAME incident, since that is still one
// continuous incident) and, if `kMaximumTurnAwaySweepDegrees` is reached
// without ever finding a clear corridor, releases to Inactive and reports
// `localRouteBlockedThisUpdate()` true for exactly that one update() call
// - a one-frame edge signal, not a persistent state, so this class never
// permanently holds the AutonomousAvoidance drive-authority tier hostage.
// The caller (main3d.cpp) uses that single frame to force WaypointNavigator
// to replan/blacklist the current global target from the CURRENT pose -
// see main3d.cpp's own docs on the post-block handoff, including why the
// caller must ALSO briefly suppress re-arming `triggerAvoidance` until the
// robot's heading has genuinely changed (otherwise, since obstacleDetected()
// reflects a real-time sensor reading and the robot has not moved at all
// the instant after release, a fresh incident would start on the very
// next frame before Navigation's newly-replanned command ever gets a
// chance to actually turn the robot - never resolving in aggregate even
// though each individual incident is now bounded).
//
// Still has no knowledge of RobotStateMachine, Event, IRobotHardware,
// VirtualDistanceSensor, VirtualObstacleSensorArray, or VirtualWorld
// itself - the caller (main3d.cpp) computes ObstacleHazardSample/
// forwardCorridorClear/the current RobotPose and decides *when* to apply
// wheelSpeeds() via VirtualRobotHardware::setAutonomousWheelSpeeds()/
// clearAutonomousWheelOverride(), exactly like every prior phase.
class ReactiveObstacleAvoidance
{
public:
    // Fixed in-place turn wheel-speed magnitude, in world units/second -
    // unchanged from Phase 13Q/13R. Direction is now chosen per-incident
    // (see chooseTurnSign() in the .cpp) rather than always the same
    // fixed sign, but the magnitude itself, and the fact that it produces
    // zero linear velocity (pure rotation), are unchanged.
    static constexpr float kTurnWheelSpeed = 0.6F;

    // Forward wheel speed while AdvanceClear, in world units/second -
    // deliberately less than VirtualRobotHardware::kForwardWheelSpeed
    // (1.0F), matching this codebase's existing convention that a
    // controlled reactive maneuver (HomeNavigator::kNavigationForwardSpeed
    // 0.8F, TableEdgeSafetyController::kRecoveryInwardSpeed 0.6F) reads as
    // deliberate, not a full-speed dash.
    static constexpr float kAdvanceWheelSpeed = 0.8F;

    // Minimum straight-line distance (world units) the robot must
    // translate from the pose where AdvanceClear began before this class
    // will release back to Inactive - the fix for "clear corridor" not
    // implying "physically bypassed." Derived, not blindly hardcoded:
    // twice RobotCollision.hpp's own kRobotCollisionRadius (the same
    // collision-footprint source of truth ForwardClearanceProbe's own
    // clearance margin already uses) - i.e. the robot's full collision
    // DIAMETER, a natural, already-justified-elsewhere V1 measure of "far
    // enough to no longer be straddling roughly the same footprint area
    // it got stuck at." At this project's actual RobotDimensions (0.5F
    // radius), this is 1.0F - inside the brief's own suggested 0.6F-1.0F
    // range. Declared here, defined out-of-line in the .cpp (after
    // kRobotCollisionRadius is guaranteed already constructed within that
    // one translation unit) rather than as an in-class initializer, to
    // avoid any static-initialization-order ambiguity between two inline
    // variables defined in different headers.
    static const float kMinimumBypassDistanceWorldUnits;

    // Phase 13X blocker fix: the maximum accumulated ABSOLUTE heading
    // rotation (degrees) TurnAway may perform for a single incident before
    // giving up and reporting `localRouteBlockedThisUpdate()`. Exactly one
    // full revolution: TurnAway always rotates continuously in one fixed
    // (latched) direction, so a full 360-degree sweep genuinely samples
    // every possible heading exactly once - there is no heading a
    // continuous rotation could reach that a full revolution does not
    // already cover, so no larger bound would ever find a corridor a
    // smaller-than-360 bound could miss, and no amount of EXTRA rotation
    // beyond 360 could discover a clear heading the first 360 degrees did
    // not already sample. 360.0F is therefore not an arbitrary tuning
    // constant - it is the mathematically complete search of this
    // incident's one rotational degree of freedom.
    static constexpr float kMaximumTurnAwaySweepDegrees = 360.0F;

    // Phase 13X blocker fix: true for exactly the one update() call on
    // which TurnAway's bounded sweep was exhausted without ever finding a
    // clear corridor - a one-frame EDGE signal (mirrors this codebase's
    // other edge-triggered signals, e.g. HomeArrivalEventSource), not a
    // persistent state. On that same call, this class has already
    // released itself back to Inactive (see update()'s own docs) - it
    // never continues occupying the AutonomousAvoidance drive-authority
    // tier once local avoidance has given up. The caller (main3d.cpp) is
    // expected to react on this exact frame: force the global navigation
    // layer (WaypointNavigator) to replan/blacklist its current target
    // from the robot's CURRENT pose, and briefly suppress re-arming
    // `triggerAvoidance` until the robot's heading has genuinely changed
    // (see this class's own top-level docs for why that second part is
    // required, not merely the replan itself).
    bool localRouteBlockedThisUpdate() const noexcept;

    // Advances the incident by one frame/step.
    //   - `enabled` false (the `A` toggle off) forces Inactive
    //     immediately, regardless of every other argument - unchanged
    //     from Phase 13R.
    //   - `triggerAvoidance` true while Inactive begins a NEW incident:
    //     enters TurnAway and chooses (once, from `hazard`) the turn
    //     direction latched for the remainder of this incident - see
    //     chooseTurnSign()'s own docs in the .cpp. Ignored while an
    //     incident is already in progress (TurnAway/AdvanceClear) - the
    //     latch, once started, is driven by `forwardCorridorClear` and
    //     displacement alone, exactly like Phase 13R's own "trigger going
    //     false mid-incident does not release" behavior.
    //   - TurnAway -> AdvanceClear the instant `forwardCorridorClear`
    //     becomes true (`pose` is captured as the AdvanceClear start
    //     point).
    //   - AdvanceClear -> TurnAway if `forwardCorridorClear` goes false
    //     again (re-blocked) - the latched turn direction from this same
    //     incident is preserved, never recomputed.
    //   - AdvanceClear -> Inactive once `forwardCorridorClear` is (still)
    //     true AND `pose` has moved at least
    //     kMinimumBypassDistanceWorldUnits from the AdvanceClear start
    //     point.
    //   - TurnAway -> Inactive (Phase 13X blocker fix), reporting
    //     `localRouteBlockedThisUpdate()` true for this one call, once
    //     accumulated rotation for the current incident reaches
    //     kMaximumTurnAwaySweepDegrees without `forwardCorridorClear` ever
    //     having become true - see this class's own top-level docs.
    // `pose` should be the robot's current, real RobotPose every call
    // (world.robotPose() in main3d.cpp) - this class never mutates it,
    // only reads position for its own displacement bookkeeping.
    void update(bool enabled, bool triggerAvoidance, bool forwardCorridorClear, const RobotPose& pose,
                const ObstacleHazardSample& hazard) noexcept;

    // True while TurnAway or AdvanceClear - the caller's cue to hold
    // VirtualRobotHardware's autonomous-avoidance wheel override active
    // (see main3d.cpp), unchanged in meaning from Phase 13R's own
    // active().
    bool active() const noexcept;

    // The current phase - Inactive/TurnAway/AdvanceClear. Exposed for
    // telemetry (Ayrıntılı HUD) and tests; main3d.cpp's own authority-
    // sync logic only ever needs active(), not this.
    AvoidanceState state() const noexcept;

    // Deterministic wheel speeds for the CURRENT state: {0, 0} while
    // Inactive; {-kTurnWheelSpeed, +kTurnWheelSpeed} or the mirrored pair
    // while TurnAway, depending on the latched direction for this
    // incident; {kAdvanceWheelSpeed, kAdvanceWheelSpeed} (straight,
    // deterministic forward motion - no arc/steering) while AdvanceClear.
    // Safe to call regardless of active() - the caller is still
    // responsible for only applying it while active() is true, exactly
    // like Phase 13R's own avoidanceWheelSpeeds() contract.
    WheelSpeeds wheelSpeeds() const noexcept;

private:
    float chooseTurnSign(const ObstacleHazardSample& hazard) const noexcept;

    AvoidanceState state_ = AvoidanceState::Inactive;
    float latchedTurnSign_ = 1.0F;
    Vec3 advanceStartPosition_{};

    // Phase 13X blocker fix: accumulated ABSOLUTE heading rotation for the
    // CURRENT incident (see kMaximumTurnAwaySweepDegrees's own docs) -
    // spans multiple TurnAway phases within one incident (an AdvanceClear
    // re-block does not reset it; only a brand new incident, or a full
    // release, does).
    float accumulatedTurnAwayRotationDegrees_ = 0.0F;
    float turnAwayPreviousHeadingDegrees_ = 0.0F;
    bool turnAwayHeadingSeeded_ = false;
    bool localRouteBlockedThisUpdate_ = false;
};

} // namespace robot::visual
