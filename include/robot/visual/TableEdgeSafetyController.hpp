#pragma once

#include <string_view>

#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/VirtualCliffSensor.hpp"

namespace robot::visual
{

// Deterministic, raylib-free, stateful V1 table-edge emergency-recovery
// policy (Phase 13S; heading-aware release condition added in the Phase
// 13S manual-validation bugfix): converts CliffSensorReadings plus the
// robot's own pose/table geometry into emergency wheel speeds. Entirely
// separate from ReactiveObstacleAvoidance (solid-obstacle avoidance) and
// RobotCollision (solid-obstacle penetration guard) - a table edge is not
// a solid obstacle, so neither of those components is reused or extended
// for this; see docs/technical-decisions.md (Phase 13S) for the full
// responsibility split. Has no knowledge of RobotStateMachine, Event, or
// IRobotHardware - the caller (main3d.cpp) feeds it this frame's
// CliffSensorReadings/RobotPose/TableSurface and applies
// recoveryWheelSpeeds() via VirtualRobotHardware's safety override API,
// exactly matching ReactiveObstacleAvoidance's own caller contract.
//
// BUGFIX CONTEXT (human manual validation, Phase 13S): the original
// version released Turning the instant CliffSensorReadings::anyCliff()
// was false, with no requirement that the robot had turned toward the
// table interior at all. On a straight edge, BackingAway typically clears
// the triggering corner with only a small margin, so a single simulation
// step's rotation was frequently already enough to satisfy "all sensors
// safe" - releasing safety with an almost-unchanged, still-edge-facing
// heading, so the robot immediately re-approached the same edge
// (repeated back-and-forth). At table corners this was less visible
// because clearing sensors relative to TWO edges naturally demands more
// rotation. The fix: Turning now also tracks a geometry-derived target
// heading (toward the table center) and requires the robot to be
// genuinely aligned with it - not just "sensors happen to read safe this
// instant" - before releasing. See update()'s docs below and
// docs/technical-decisions.md (Phase 13S) for the full before/after
// comparison.
//
// BUGFIX #2 CONTEXT (human manual validation): once the heading-alignment
// requirement above existed, a second, distinct defect surfaced: heading
// alignment does NOT imply the robot's full footprint is safely back on
// the table. A robot can reach its target heading (error within
// tolerance) while one corner - most often a REAR corner, since Turning
// only ever rotates, it never translates the robot's center - is still
// off the table. The old Turning exit condition required
// `!anyCliff() && headingSafe` together, so it correctly refused to
// release, but Turning's own recoveryWheelSpeeds() only ever offers an
// in-place turn - which cannot move the robot's center at all, so it
// could spin in place indefinitely with the heading already correct and
// one corner permanently unsupported (reproduced deterministically by
// TableEdgeSafetyControllerTests.cpp's
// TurningTransitionsToAdvancingInwardWhenHeadingSafeButCornerStillEdge
// before this fix was written). The fix: heading completion and physical
// support completion are now two separate conditions
// (`kRecoveryHeadingToleranceDegrees` vs. `kRecoverySupportMargin`/
// `areAllCornersSafelyInsideTable()`, VirtualCliffSensor.hpp) - once the
// heading is safe but support is not yet robust, `Turning` hands off to
// the new `AdvancingInward` state, which drives forward (translating the
// center, the one thing pure rotation cannot do) until the WHOLE
// footprint is safely inside the table by a small margin. See
// docs/technical-decisions.md (table-edge recovery bugfix #2) for the
// full four-concept breakdown (detect / separate / orient / translate).
class TableEdgeSafetyController
{
public:
    // Internal recovery states - NOT RobotStateMachine states, and never
    // exposed to/consumed by RobotStateMachine/RobotController in any
    // way. RobotState can legitimately still read Moving while this
    // controller is in BackingAway/Turning/etc. - the same command-vs-
    // effective-actuator distinction Phase 13Q/13R already established
    // for DriveAuthority (see docs/technical-decisions.md, Phase 13S).
    enum class RecoveryState
    {
        Inactive,
        BackingAway,
        MovingForwardFromRearEdge,
        Turning,
        AdvancingInward
    };

    // Straight recovery speed (reverse/forward), in world units/second -
    // deliberately the same magnitude as VirtualRobotHardware's own
    // kForwardWheelSpeed (1.0F) so recovery reads as a normal-speed
    // maneuver, not hesitant creeping - this is an emergency response,
    // not a cautious one.
    static constexpr float kRecoveryLinearSpeed = 1.0F;

    // In-place recovery turn speed MAGNITUDE - matches
    // ReactiveObstacleAvoidance::kTurnWheelSpeed (0.6F) so both
    // "autonomous-authority turning" maneuvers read the same in the HUD/
    // telemetry. Unlike ReactiveObstacleAvoidance, the SIGN is no longer
    // fixed - see recoveryWheelSpeeds()'s docs below: this controller
    // picks whichever in-place turn direction is the shorter path to
    // targetRecoveryHeadingDegrees_, computed fresh from
    // shortestSignedHeadingErrorDegrees() every update() call while
    // Turning.
    static constexpr float kRecoveryTurnSpeed = 0.6F;

    // Heading-alignment tolerance for release, in degrees - the maximum
    // |shortestSignedHeadingErrorDegrees()| from the recovery target
    // heading allowed before Turning may release, in ADDITION to all
    // cliff sensors being safe (see update()'s docs). 10.0F: tight enough
    // that "recovered" means a genuinely different, inward-facing
    // heading (not the ~5.7 degrees of a single simulation step that
    // previously let Turning exit almost immediately on some straight-
    // edge approaches - see the bugfix context above), loose enough that
    // exact floating-point heading equality is never required.
    static constexpr float kRecoveryHeadingToleranceDegrees = 10.0F;

    // Straight recovery speed while AdvancingInward, in world units/
    // second (table-edge recovery bugfix #2) - deliberately SLOWER than
    // kRecoveryLinearSpeed (1.0F, used for the initial emergency back-
    // away/forward-away move): by the time AdvancingInward engages, the
    // robot is no longer in the initial emergency - it is already
    // pointed at a safe heading and just needs a controlled, precise
    // approach to close the remaining support margin, not another full-
    // speed escape maneuver. 0.6F matches kRecoveryTurnSpeed's own
    // magnitude, so every "controlled maneuver" phase of recovery reads
    // consistently in the HUD/telemetry.
    static constexpr float kRecoveryInwardSpeed = 0.6F;

    // Recovery-release support buffer, in world units (table-edge
    // recovery bugfix #2) - see
    // VirtualCliffSensor.hpp's areAllCornersSafelyInsideTable(). A corner
    // that is merely on-table AT the exact inclusive boundary is not
    // robust: the next frame's motion (or even floating-point noise)
    // could push it back off. 0.15F sits inside the brief's suggested
    // 0.10F-0.20F range - small relative to the table's own ~12-unit
    // width, comfortably larger than a single simulation step's typical
    // positional drift at kRecoveryInwardSpeed.
    static constexpr float kRecoverySupportMargin = 0.15F;

    // Proposed-motion lookahead distance, in world units (table-edge
    // recovery bugfix #3 - see update()'s own docs for the full defect
    // this fixes): a small, fixed step used ONLY to ask "if I keep
    // translating this way, does the overall situation get better or
    // worse" - never tied to any specific delta-time, since update()
    // itself receives none. Small enough to be a genuine one-step probe
    // (comparable to a single simulation frame's worth of motion at
    // kRecoveryLinearSpeed with a typical ~0.05s step), never a lookahead
    // far enough to function as path planning.
    static constexpr float kSupportCheckLookaheadDistance = 0.05F;

    // Phase 13X blocker fix (deadlock repair): `translatingWouldNotHelp()`
    // (bugfix #3, above) only ever reasons about TABLE geometry - it has
    // no knowledge of solid obstacles (by design; see this class's own
    // top-level docs on the deliberate separation from
    // ReactiveObstacleAvoidance/RobotCollision). If a translating state's
    // (BackingAway/MovingForwardFromRearEdge/AdvancingInward) commanded
    // motion is being externally vetoed every frame - in practice, by
    // VirtualRobotHardware's independent obstacle-collision guard, e.g. a
    // desk object sitting between the robot and this incident's fixed
    // recovery target heading - the bugfix #3 probe cannot see that,
    // since it only ever evaluates a HYPOTHETICAL projected pose against
    // table bounds, never the ACTUAL pose update the collision guard just
    // rejected. Without a bound, the state can hold Safety's own always-
    // highest drive authority forever, commanding the same doomed
    // translation every single call - starving AutonomousAvoidance and
    // Navigation of the wheels indefinitely, since Safety unconditionally
    // outranks both regardless of which of ITS OWN sub-states is engaged
    // (see docs/technical-decisions.md, Phase 13X blocker fix, "safety
    // recovery stall").
    //
    // kMaxRecoveryStallFrames bounds this the same way
    // ReactiveObstacleAvoidance::kMaximumTurnAwaySweepDegrees bounds
    // TurnAway: if the robot's ACTUAL position (never a hypothetical
    // projection) has not moved more than kStallProgressEpsilon for this
    // many CONSECUTIVE update() calls while in a translating state, update()
    // releases straight to Inactive and reports
    // recoveryBlockedThisUpdate() true for exactly that one call - a one-
    // frame edge signal, mirroring
    // ReactiveObstacleAvoidance::localRouteBlockedThisUpdate() exactly
    // (see that class's own docs). This never weakens the actual safety
    // guarantee: VirtualRobotHardware's own unconditional last-resort
    // guards (obstacle-collision rejection, all-four-corners-off-table
    // rejection) remain fully active regardless of this controller's
    // active()/Inactive state, exactly as they already do for every other
    // drive authority (Manual/AutonomousAvoidance/Navigation/Fsm) - this
    // bound only ever gives up on the PROACTIVE, comfortable-margin
    // recovery maneuver once it is PROVABLY not making progress, never on
    // the hard backstop. 15 frames (0.75s of simulated time at this
    // project's ~0.05s/frame convention) is small enough not to leave the
    // robot pinned for long, large enough that a single transient frame
    // (e.g. one collision-rejected step immediately followed by real
    // progress) is never mistaken for a genuine stall, and comfortably
    // clear of TableEdgeSafetyControllerTests.cpp's own
    // TurningTransitionsToAdvancingInwardWhenHeadingSafeButCornerStillEdge
    // (which deliberately calls update() 50 times with a fixed pose to
    // exercise the pure decision-transition logic in isolation from
    // physical integration - never itself a real stall).
    static constexpr int kMaxRecoveryStallFrames = 60;

    // Minimum positional change, in world units, between consecutive
    // update() calls for a translating state to be considered "making
    // progress" (see kMaxRecoveryStallFrames above) - well below either
    // translating speed's smallest plausible per-frame step
    // (kRecoveryInwardSpeed * a typical ~0.05s frame is 0.03F), so any
    // frame with a genuinely committed translation resets the stall
    // counter, while floating-point noise on an actually-frozen position
    // never falsely resets it.
    static constexpr float kStallProgressEpsilon = 0.005F;

    // Advances the recovery state machine by one frame/step, given this
    // frame's real cliff-sensor readings and the robot's current pose/
    // table geometry (needed to compute and track the recovery target
    // heading - see beginRecovery()'s role below). Transition table:
    //
    //   Inactive                   -> BackingAway (any front cliff) -
    //                                  captures a fresh recovery target
    //                                  heading pointing from the robot's
    //                                  CURRENT position toward
    //                                  table's center (see below)
    //   Inactive                   -> MovingForwardFromRearEdge (any rear
    //                                  cliff, and no front cliff - front
    //                                  takes precedence when both trigger
    //                                  simultaneously; see "known
    //                                  limitations" in
    //                                  docs/technical-decisions.md, Phase
    //                                  13S, for the simultaneous-both-
    //                                  edges corner case) - captures the
    //                                  same kind of target heading
    //   BackingAway                -> Turning (once no front cliff, OR the
    //                                  proposed-motion safety check below
    //                                  finds one more reverse step would
    //                                  not reduce the aggregate overhang -
    //                                  see bugfix #3)
    //   MovingForwardFromRearEdge  -> Turning (once no rear cliff, OR the
    //                                  same proposed-motion safety check)
    //   Turning                    -> Inactive (once heading is safe AND
    //                                  the whole footprint is already
    //                                  safely inside the table by
    //                                  kRecoverySupportMargin - see
    //                                  update()'s release-condition docs)
    //   Turning                    -> AdvancingInward (once heading is
    //                                  safe, but the support margin is
    //                                  NOT yet satisfied - pure rotation
    //                                  cannot fix this on its own, since
    //                                  it never translates the robot's
    //                                  center; see the bugfix #2 context
    //                                  above)
    //   AdvancingInward            -> Turning (if heading drifts back
    //                                  outside tolerance while driving
    //                                  forward - realign before
    //                                  continuing, per the brief's
    //                                  robustness requirement - OR if the
    //                                  proposed-motion safety check finds
    //                                  one more forward step would not
    //                                  reduce the aggregate overhang - see
    //                                  bugfix #3)
    //   AdvancingInward            -> Inactive (once heading is (still)
    //                                  safe AND the support margin is now
    //                                  satisfied)
    //   BackingAway/
    //   MovingForwardFromRearEdge/
    //   AdvancingInward            -> Inactive (Phase 13X blocker fix:
    //                                  bounded recovery-stall escape - see
    //                                  kMaxRecoveryStallFrames's own docs.
    //                                  Reports recoveryBlockedThisUpdate()
    //                                  true for exactly this one call,
    //                                  same one-frame-edge shape as
    //                                  ReactiveObstacleAvoidance's own
    //                                  localRouteBlockedThisUpdate())
    //
    // BUGFIX #3 CONTEXT (human manual validation, Phase 13W after the
    // robot/table rescale): BackingAway/MovingForwardFromRearEdge/
    // AdvancingInward all translate along a direction derived purely from
    // the robot's CURRENT heading (reverse/forward respectively) - never
    // validated against which axis the actual triggering overhang is on.
    // "Front"/"rear" are ROBOT-relative labels, not TABLE-relative ones:
    // a table edge encountered at a shallow/lateral angle (the robot's
    // heading roughly PARALLEL to that edge, not perpendicular to it) can
    // trip a front-corner cliff whose overhang is almost entirely along
    // the axis the robot's current heading barely moves it on - backing
    // away then does essentially nothing to fix it, and can just as
    // easily carry the robot toward or past a DIFFERENT edge, growing
    // the problem instead of solving it, potentially reaching
    // `VirtualRobotHardware`'s own ALL-four-corners-off-table hard
    // fail-safe (which then rejects every further translation - a
    // genuine, reproducible permanent freeze; see
    // TableEdgeSafetyControllerTests.cpp's own
    // BackingAwayAtShallowAngleDoesNotDriveOffADifferentEdge and
    // docs/technical-decisions.md for the sweep that found it). Fixed by
    // a small "does this actually help" probe
    // (`VirtualCliffSensor::aggregateTableOverhang()` before vs. after one
    // `kSupportCheckLookaheadDistance`-sized step in the state's own
    // travel direction) applied to all three translating states: if one
    // more step would not reduce the aggregate overhang, the state
    // reorients toward the already-known-safe target heading (Turning)
    // instead of continuing to blindly translate. This is a lookahead
    // PROBE, not a planner - one fixed-distance step, never search/
    // iteration over multiple candidate directions.
    //
    // The target heading is captured ONCE per incident (on the Inactive
    // -> BackingAway/MovingForwardFromRearEdge transition) and held fixed
    // for the remainder of that incident, even as the robot's position
    // continues to change during BackingAway/MovingForwardFromRearEdge/
    // AdvancingInward - continuously re-aiming at a moving target as the
    // robot moves would risk unstable steering for no benefit here (the
    // target is a fixed point, the table center, so recomputing from a
    // position a small distance away rarely changes it by much, and
    // holding it fixed guarantees Turning/AdvancingInward are always
    // chasing one stationary target).
    //
    // Once active (any state other than Inactive), stays active across
    // calls until a Turning/AdvancingInward -> Inactive transition - the
    // caller does not need to track "was this already triggered," exactly
    // like ReactiveObstacleAvoidance's own latch.
    void update(const CliffSensorReadings& readings, const RobotPose& pose, const TableSurface& table) noexcept;

    // True whenever state() != Inactive - the caller's cue to hold
    // VirtualRobotHardware's safety wheel override active (see
    // main3d.cpp).
    bool active() const noexcept;

    RecoveryState state() const noexcept;

    // Phase 13X blocker fix: true for exactly the one update() call on
    // which a translating state's bounded stall (kMaxRecoveryStallFrames)
    // was exhausted without the robot's actual position ever making
    // progress - see that constant's own docs. On that same call, this
    // class has already released itself to Inactive. Mirrors
    // ReactiveObstacleAvoidance::localRouteBlockedThisUpdate() exactly -
    // callers (main3d.cpp) should react on this exact frame the same way:
    // force the global navigation layer to replan from the robot's
    // CURRENT pose, and briefly suppress this controller's own re-arming
    // until the robot's position has genuinely changed (see
    // docs/technical-decisions.md, Phase 13X blocker fix, "safety
    // recovery stall").
    bool recoveryBlockedThisUpdate() const noexcept;

    // Deterministic wheel speeds for the CURRENT state - {0, 0} while
    // Inactive (never applied by a correctly-written caller, which
    // should check active() first, but always a safe, deterministic
    // value regardless). While Turning, the sign of both wheels is
    // derived from currentHeadingErrorDegrees() (updated every update()
    // call): a positive error turns the same direction Phase 13Q/13R's
    // ReactiveObstacleAvoidance always uses (left = -kRecoveryTurnSpeed,
    // right = +kRecoveryTurnSpeed, omega > 0); a negative error turns the
    // opposite way (left = +kRecoveryTurnSpeed, right =
    // -kRecoveryTurnSpeed, omega < 0) - always the SHORTER of the two
    // directions toward the target heading, per
    // shortestSignedHeadingErrorDegrees()'s own (-180, 180] range. While
    // AdvancingInward, equal positive speeds (kRecoveryInwardSpeed) drive
    // straight forward - the robot is already pointed approximately
    // toward the table interior, so this translates the center toward
    // safety, the one thing Turning's pure rotation cannot do.
    WheelSpeeds recoveryWheelSpeeds() const noexcept;

    // The recovery target heading captured for the CURRENT incident (see
    // update()'s docs) - meaningless while Inactive (returns whatever the
    // previous incident last held, or 0.0F if none has ever occurred);
    // callers should only read this while active(). Visual-simulator-only
    // telemetry getter (HUD "Edge target heading").
    float targetRecoveryHeadingDegrees() const noexcept;

    // shortestSignedHeadingErrorDegrees(pose.headingDegrees,
    // targetRecoveryHeadingDegrees()) as of the most recent update() call
    // - live while any recovery state is active, meaningless while
    // Inactive (same caveat as targetRecoveryHeadingDegrees() above).
    // Visual-simulator-only telemetry getter (HUD "Edge heading error").
    float currentHeadingErrorDegrees() const noexcept;

private:
    void beginRecovery(const RobotPose& pose, const TableSurface& table) noexcept;

    RecoveryState state_ = RecoveryState::Inactive;
    float targetRecoveryHeadingDegrees_ = 0.0F;
    float currentHeadingErrorDegrees_ = 0.0F;

    // Phase 13X blocker fix: bounded recovery-stall bookkeeping - see
    // kMaxRecoveryStallFrames's own docs.
    Vec3 stallAnchorPosition_{};
    int framesSinceStallProgress_ = 0;
    bool recoveryBlockedThisUpdate_ = false;
};

// Visual-only, not part of any FSM/RobotState convention - mirrors
// VirtualDriveCommand/DriveAuthority's own toString() shape (Phase 13S).
constexpr std::string_view toString(TableEdgeSafetyController::RecoveryState state) noexcept
{
    switch (state)
    {
        case TableEdgeSafetyController::RecoveryState::Inactive: return "Inactive";
        case TableEdgeSafetyController::RecoveryState::BackingAway: return "BackingAway";
        case TableEdgeSafetyController::RecoveryState::MovingForwardFromRearEdge: return "MovingForwardFromRearEdge";
        case TableEdgeSafetyController::RecoveryState::Turning: return "Turning";
        case TableEdgeSafetyController::RecoveryState::AdvancingInward: return "AdvancingInward";
    }
    return "Unknown";
}

} // namespace robot::visual
