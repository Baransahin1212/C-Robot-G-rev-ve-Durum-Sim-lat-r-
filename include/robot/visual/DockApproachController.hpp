#pragma once

#include <string_view>

#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/DockChargingContacts.hpp"
#include "robot/visual/HomeNavigator.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Phase 13Y (precision reverse docking): global A* route planning
// (GridPathPlanner/WaypointNavigator) is deliberately NOT responsible for
// precision docking - it only ever needs to bring the robot to a docking
// STAGING point comfortably in front of the dock, through known Free
// space, respecting the same clearance/avoidance rules every other route
// does. The last short stretch is a different kind of maneuver entirely:
// rotate so the robot's REAR faces the dock, then reverse slowly along the
// dock's own centerline until the two rear charging receivers physically
// meet the two dock pins - see DockChargingContacts.hpp for that contact
// geometry, and this header's own computeDockStagingPoint()/
// computeDockReverseHeadingDegrees() docs for how the staging point/
// reverse heading are derived, and DockApproachController's own class
// docs for the full state machine.
//
// Phase 13X's earlier "drive forward into the literal dock position"
// design (Aligning/FinalApproach/Arrived) is gone entirely - the human
// product requirement is now realistic reverse parking with physical
// contact-pin alignment, never a nose-first drive-through.
enum class DockApproachState
{
    Inactive,

    // Enabled, but the caller has not yet reported the global route has
    // reached the staging point - this controller produces zero wheel
    // speeds and defers entirely to whatever is currently driving Stage 1
    // (WaypointNavigator's own Navigation-tier output).
    //
    // Once Stage 1 DOES report arrival, this state also absorbs a real,
    // GUI/integration-test-traced gap between the two stages: Stage 1's own
    // HomeNavigator-based steering hard-stops (zero wheel speeds) the
    // moment it is within HomeNavigator::kHomeArrivalRadius (0.40) of the
    // staging point, in WHATEVER direction it happened to be approaching
    // from - nothing about "arrived" implies the robot is anywhere near the
    // dock's own centerline, which kReverseLateralErrorToleranceWorldUnits
    // (0.10) requires. Asking Stage 1 to "just replan" here is a no-op: the
    // goal point and the robot's (now-stationary) pose are both unchanged,
    // so a fresh GridPathPlanner run deterministically reproduces the exact
    // same route and the exact same immediate "Arrived, zero speed" result
    // forever (a real infinite loop this exact design closes - see this
    // state's own .cpp docs). So instead, once arrived-but-off-centerline,
    // THIS controller drives a short straight precision creep of its own -
    // rotate to face the exact staging point, then drive forward - until
    // the centerline tolerance is met. Still Navigation-tier, still no new
    // DriveAuthority tier, and still not the reverse-parking maneuver
    // itself (that only ever starts once AlignForReverse begins) - just
    // Stage 2 finishing the job Stage 1's coarser arrival radius could not.
    NavigateToStagingPoint,

    // At (or very near) the staging point - rotating in place until the
    // robot's FORWARD heading points toward the desk interior (i.e. AWAY
    // from the dock - see computeDockReverseHeadingDegrees()'s own docs),
    // so its REAR faces the dock. Never begins reversing before this
    // heading is within tolerance - see class docs, "no diagonal/sideways
    // entry." PURELY rotational - it cannot correct lateral (off-
    // centerline) error by itself; by the time this state is reached,
    // NavigateToStagingPoint's own precision creep (see that state's own
    // docs) has already guaranteed the robot is within
    // kReverseLateralErrorToleranceWorldUnits of the centerline, so pure
    // rotation is always sufficient here.
    AlignForReverse,

    // Aligned within tolerance - driving BACKWARD (equal negative wheel
    // speeds) at the slower kReverseDockSpeed along the dock's own
    // centerline. Falls back to AlignForReverse if heading drifts past the
    // release tolerance, OR if lateral error from the centerline grows too
    // large - by the time ReverseApproach begins, NavigateToStagingPoint's
    // own precision creep (see that state's own docs) has already ruled
    // out a large systemic offset, so lateral drift here can only be
    // gradual accumulation from a small residual heading error; re-zeroing
    // heading via AlignForReverse stops further accumulation - see class
    // docs, "no complicated curved reverse parking," a deliberate V1
    // simplification that never attempts a curved in-lane correction of
    // its own.
    ReverseApproach,

    // Both rear charging receivers are simultaneously within
    // kContactAlignmentToleranceWorldUnits of their matching dock pin
    // (chargingContactsAligned() - see DockChargingContacts.hpp), AND
    // heading is within kReverseDockHeadingToleranceDegrees of the target
    // reverse heading - never position/contact-distance alone (Phase 13Y
    // brief: "do not consider docking successful from center-distance
    // alone"). NEVER a one-way "done forever" latch: contacts/heading are
    // re-checked every update() call even while Docked, so a robot
    // displaced away again (e.g. ordinary avoidance in the brief window
    // before HomeReached is consumed, or a Safety recovery) resumes
    // AlignForReverse rather than freezing permanently with zero wheels -
    // the same real, traced defect class this re-check pattern already
    // closed once for Phase 13X's own Arrived state (see .cpp docs).
    Docked,

    // The derived staging point/dock geometry failed its own geometric
    // self-check (see isDockStagingGeometryValid() below) - a defensive
    // contract for a future geometry change, never expected to trigger
    // for this project's current fixed dock/table/robot dimensions (see
    // DockApproachControllerTests.cpp).
    Failed
};

constexpr std::string_view toString(DockApproachState state) noexcept
{
    switch (state)
    {
        case DockApproachState::Inactive: return "Inactive";
        case DockApproachState::NavigateToStagingPoint: return "NavigateToStagingPoint";
        case DockApproachState::AlignForReverse: return "AlignForReverse";
        case DockApproachState::ReverseApproach: return "ReverseApproach";
        case DockApproachState::Docked: return "Docked";
        case DockApproachState::Failed: return "Failed";
    }
    return "Unknown";
}

// Everything one update() call reports.
struct DockApproachOutput
{
    WheelSpeeds wheelSpeeds;
    DockApproachState state = DockApproachState::Inactive;
    bool docked = false;

    // True while THIS controller (not Stage 1's WaypointNavigator) owns
    // producing the Navigation-tier wheel speeds - i.e. AlignForReverse or
    // ReverseApproach. The caller (main3d.cpp) uses this both to pick
    // whose wheelSpeeds to apply and to withhold the ordinary obstacle-
    // triggered avoidance trigger for this one validated lane - see class
    // docs, "FINAL APPROACH HAZARD CONTRACT" (Phase 13X, unchanged in
    // spirit for Phase 13Y).
    bool driving = false;

    // True on the exact frame this controller hands back to Stage 1
    // because the robot ended up materially far from the staging point
    // again (e.g. a Safety table-edge recovery displaced it mid-approach -
    // see kReentryDistanceMultiplier's own docs) - genuinely needs a fresh
    // global route, unlike the ordinary excessive-lateral-error case (which
    // NavigateToStagingPoint's own precision creep now corrects locally,
    // in-controller, without ever involving Stage 1 - see that state's own
    // docs for why a Stage-1 replan is a no-op there). The caller
    // (main3d.cpp) ORs this into WaypointNavigator's own `forceReplan`
    // parameter (the same existing mechanism already used for avoidance/
    // Safety displacement - see main3d.cpp's own docs). Read with the same
    // one-frame sticky-capture convention this file's caller already uses
    // elsewhere.
    bool needsStage1Replan = false;

    // Phase 13Y dock-capture LATCH fix (real-GUI-traced): true from the
    // first update() call `arrivedAtStagingPoint` was ever true (while
    // `enabled`), for the rest of THIS session (reset only when `enabled`
    // goes false) - a real, GUI-traced defect this field closes: the
    // caller's own entry predicate (DockCaptureRegion::isDockCaptureEligible()
    // in practice) is a per-frame geometric snapshot that can legitimately
    // flicker false again mid-maneuver (this project's real desk layout
    // has a genuine ambiguous region where a desk object briefly becomes
    // numerically nearer than the dock housing - see
    // DockCaptureRegion.hpp's own docs), even while the robot is
    // physically CLOSER to the exact staging point than when entry was
    // first granted. Re-checking the raw entry predicate every frame as an
    // ownership kill switch made the controller drop all drive authority
    // mid-approach the instant that flicker occurred (RobotState stuck
    // ReturningHome, `Kontrol: Normal`/DriveAuthority::Fsm, zero wheels,
    // indefinitely - a real, traced production stall). ENTRY (may this
    // session begin) and CONTINUATION (does it keep going) are therefore
    // now deliberately different questions: `arrivedAtStagingPoint` only
    // ever gates ENTRY (see .cpp), while every internal transition
    // thereafter is gated on THIS latch plus this controller's own
    // existing safety checks (lateral/heading tolerance, kReentryDistanceMultiplier,
    // contact alignment) - never again on the caller's raw per-frame
    // signal. The caller may use this (sticky, one frame lagged - same
    // convention as `needsStage1Replan`) to ALSO latch its own dock-hazard
    // suppression window for the whole session, not just AlignForReverse/
    // ReverseApproach/Docked - see DockCaptureRegion.hpp's own docs.
    bool captured = false;
};

// Small named margin, on top of the geometric terms below, so the
// staging point sits comfortably clear of the platform's own footprint
// edge rather than exactly on it - same "named margin on top of a derived
// value" precedent as NavigationClearance::kPlanningExtraMargin.
inline constexpr float kDockStagingMarginWorldUnits = 0.15F;

// Deterministic docking staging point (Phase 13Y) - derived ONLY from
// BasePlatform/TableSurface geometry plus this project's existing robot-
// size/clearance constants, never a hardcoded/visually guessed
// coordinate. Sits on the dock's own longitudinal centerline, on the side
// facing the desk interior (dockInwardDirection() - see
// DockChargingContacts.hpp), offset out from `base.position` by:
//   (platform half-depth along the dock axis) + RobotCollision::
//   kRobotCollisionRadius (in-place rotation clearance) +
//   kDockStagingMarginWorldUnits
// far enough that the robot's own footprint, centered at the staging
// point, clears the platform's footprint entirely with room to rotate in
// place (AlignForReverse) before ever starting the reverse approach - see
// DockApproachControllerTests.cpp's own StagingPointHasPlanningClearance/
// StagingPointIsInsideTable for the proof against this project's real
// dock geometry. Named "staging point" (Phase 13Y), not "approach point"
// (Phase 13X's old name) - the robot no longer drives FORWARD from here
// into the dock; it rotates in place, then reverses.
Vec3 computeDockStagingPoint(const BasePlatform& base, const TableSurface& tableSurface) noexcept;

// The heading (degrees, this project's 0 = +Z convention) the robot's
// FORWARD direction must hold while reversing into the dock - i.e.
// pointing toward the desk interior, AWAY from the dock
// (dockInwardDirection() - see DockChargingContacts.hpp) - so that
// NEGATIVE (reverse) wheel motion drives the robot toward the dock, rear-
// first. This is the deliberate 180-degree opposite of Phase 13X's old
// "drive forward into the dock" entrance heading; derived from the exact
// same dock-axis fact (dockInwardDirection()) computeDockStagingPoint()
// and DockChargingContacts.hpp's own contact-lateral-axis both use, so
// none of them can ever disagree about which way the dock faces.
float computeDockReverseHeadingDegrees(const BasePlatform& base, const TableSurface& tableSurface) noexcept;

// True if the derived staging point sits comfortably inside `tableSurface`
// and is not degenerate (finite, non-zero offset from `base.position`) -
// the "prove the reverse-docking lane is physically valid" self-check
// this class's own docs promise, evaluated once whenever this controller
// (re)activates. Deliberately does NOT read VirtualWorld::obstacles()
// (the dock housing itself) - by construction, the staging point is
// offset TOWARD the desk-interior side, strictly farther from the nearest
// table edge (and therefore from the housing, which sits on the
// OPPOSITE, near-edge side of `base.position` - see VirtualWorld.cpp's
// own kDockHousingZ derivation) than `base.position` itself.
bool isDockStagingGeometryValid(const BasePlatform& base, const TableSurface& tableSurface) noexcept;

// Deterministic, raylib-free Stage-2 "precision reverse docking"
// controller (Phase 13Y) - the counterpart to GridPathPlanner/
// WaypointNavigator's Stage-1 global routing, which now only ever routes
// to computeDockStagingPoint(). Has no knowledge of RobotStateMachine,
// Event, IRobotHardware, RobotController, or ExplorationMap - exactly
// like HomeNavigator's own boundary. The caller (main3d.cpp) is
// responsible for:
//   - routing Stage 1 (WaypointNavigator) to computeDockStagingPoint(),
//     never the literal base.position - see main3d.cpp's own docs;
//   - telling this controller when Stage 1 has arrived
//     (`arrivedAtStagingPoint`);
//   - applying this controller's own wheelSpeeds via the SAME Navigation-
//     tier override API WaypointNavigator already uses whenever
//     DockApproachOutput::driving is true (never a new DriveAuthority
//     tier - Safety > Manual > AutonomousAvoidance > Navigation > Fsm is
//     completely unchanged);
//   - withholding the ordinary obstacle-triggered avoidance TRIGGER
//     condition (never avoidance itself, never Safety) while `driving`
//     (or Docked) is true, since this one lane has already been proven
//     physically valid by isDockStagingGeometryValid() - see
//     main3d.cpp's own docs at the call site for the full "why."
//
// STATE MACHINE: NavigateToStagingPoint (waiting for Stage 1) ->
// AlignForReverse (rotate in place toward computeDockReverseHeadingDegrees(),
// shortest-path, no repeated 360 - identical turn-direction convention to
// HomeNavigator/TableEdgeSafetyController) -> ReverseApproach (equal
// NEGATIVE wheel speeds at kReverseDockSpeed, monitoring both heading
// drift and lateral error off the dock centerline - either one exceeding
// tolerance falls back to AlignForReverse, never a curved correction) ->
// Docked (both charging contact pairs aligned AND heading aligned - see
// DockApproachState::Docked's own docs). If the robot ends up displaced
// materially far from the staging point again (e.g. a Safety recovery
// mid-approach), this controller falls back to NavigateToStagingPoint on
// its own next update() call - Stage 1 naturally re-routes it back, and
// this controller simply recomputes fresh from wherever the robot ends up
// next time Stage 1 reports arrival again, exactly like HomeNavigator's
// own continuous per-frame recomputation (never a cached/stale target).
class DockApproachController
{
public:
    // Heading hysteresis (mirrors HomeNavigator's own two-threshold
    // shape) - tight, since precision reverse docking wants a genuinely
    // straight rear-first entry, never a diagonal or sideways one.
    static constexpr float kReverseDockHeadingToleranceDegrees = 5.0F;
    static constexpr float kReverseDockHeadingReleaseToleranceDegrees = 10.0F;

    // In-place turn speed while AlignForReverse - reuses HomeNavigator's
    // own magnitude so every "autonomous-authority turning" maneuver in
    // this codebase's HUD/telemetry still reads consistently (same
    // precedent HomeNavigator's own kNavigationTurnSpeed docs cite).
    static constexpr float kAligningTurnSpeed = HomeNavigator::kNavigationTurnSpeed;

    // Reverse speed while ReverseApproach - deliberately, namedly slower
    // than Phase 13X's old forward final-approach speed (0.4, itself half
    // of HomeNavigator::kNavigationForwardSpeed's 0.8): precision reverse
    // parking toward two small physical contact points is the most
    // delicate maneuver in this codebase, so it gets the slowest named
    // speed. Derived FROM HomeNavigator::kNavigationForwardSpeed (never a
    // second, independently-chosen magic number) at 0.375x - lands at
    // 0.3, inside the Phase 13Y brief's own suggested 0.25-0.35 range.
    static constexpr float kReverseDockSpeed = HomeNavigator::kNavigationForwardSpeed * 0.375F;

    // Maximum perpendicular distance from the dock's own centerline
    // tolerated while ReverseApproach is actively backing in (and the
    // stopping criterion for NavigateToStagingPoint's own precision creep -
    // see that state's own docs). MUST stay comfortably TIGHTER than
    // kContactAlignmentToleranceWorldUnits, never looser - a real,
    // integration-test-traced defect this exact derivation closes: a pure
    // straight reverse (equal wheel speeds along the current heading) never
    // changes the robot's lateral offset from the centerline at all, so
    // whatever lateral error existed the moment ReverseApproach begins is
    // exactly what persists for the rest of the maneuver. If this tolerance
    // were looser than the contact tolerance, the controller could
    // legitimately keep reversing (never triggering "excessive lateral
    // error, correct") with an offset that can PHYSICALLY never satisfy
    // chargingContactsAligned() - both contacts share the same lateral
    // offset as the robot's own center (see DockChargingContacts.hpp's own
    // symmetric-spacing docs) - so it would just keep reversing straight
    // through the correct depth and into the dock housing's own physical
    // collision boundary, wedging there forever (exactly what a real,
    // traced multi-thousand-frame integration test run showed happening
    // before this fix). Derived AT 2/3 of kContactAlignmentToleranceWorldUnits
    // (never a second, independently-chosen number) - leaves roughly a
    // third of the contact budget for heading-drift-induced contact skew
    // and residual depth error once the exact dock depth is reached.
    static constexpr float kReverseLateralErrorToleranceWorldUnits = kContactAlignmentToleranceWorldUnits * (2.0F / 3.0F);

    // NavigateToStagingPoint's own precision-creep heading tolerance -
    // looser than kReverseDockHeadingToleranceDegrees since this is only
    // "point roughly at the staging point before creeping forward," not the
    // final rear-facing alignment itself.
    static constexpr float kStagingApproachHeadingToleranceDegrees = 10.0F;

    // Forward speed for NavigateToStagingPoint's own precision creep (see
    // that state's own docs) - the same slow, deliberate magnitude as
    // kReverseDockSpeed (derived from HomeNavigator::kNavigationForwardSpeed,
    // never a second independently-chosen number), since this creep is
    // covering a similarly small, precision-sensitive final distance.
    static constexpr float kStagingApproachSpeed = kReverseDockSpeed;

    // If the robot ends up farther than this multiple of the staging
    // offset away from the staging point while AlignForReverse/
    // ReverseApproach/Docked (e.g. a Safety table-edge recovery displaced
    // it), this controller falls back to NavigateToStagingPoint rather
    // than continuing to chase a fixed-heading lane from a position that
    // may no longer be on it - Stage 1 (WaypointNavigator, whose own goal
    // is always the staging point) naturally re-routes the robot back.
    static constexpr float kReentryDistanceMultiplier = 2.0F;

    // Phase 13Y dock-capture LATCH fix (real-GUI-traced): the maximum
    // number of CONSECUTIVE ReverseApproach->AlignForReverse fallbacks (see
    // that transition's own .cpp docs) tolerated before this controller
    // concludes the cycle is not resolving on its own and releases the
    // session latch instead - this project's own equivalent of
    // NavigationProgressTracker's "stuck" detection (see that class's own
    // docs), for the same underlying reason: a real, traced integration run
    // showed this exact fallback can repeat forever, deterministically,
    // once the robot has settled right at the dock housing's own physical
    // collision boundary without ever quite satisfying contact alignment.
    // Small - this is a genuine stuck-cycle escape hatch, not a retry
    // budget; a healthy approach should essentially never come close to it.
    static constexpr int kMaxReverseFallbackStreak = 5;

    // Phase 13Y final docking visual-precision polish (real-GUI-validated):
    // once the robot is genuinely close to physical contact, the final
    // parked pose needs to look mechanically believable - centered,
    // parallel, not visibly diagonal - not merely "within the generic
    // reverse-entry tolerances" (which are deliberately loose enough to
    // let a long straight reverse begin from a variety of approach
    // headings without prematurely aborting). This "precision zone" is the
    // last short stretch where ReverseApproach switches to tighter
    // acceptance criteria and a slower, more deliberate speed - never a
    // new DockApproachState (kept inside ReverseApproach itself, per this
    // fix's own brief: "prefer minimal architecture change").

    // Distance (the worse of the two DockChargingContactErrors, see that
    // struct's own docs) below which the precision zone engages. Derived
    // FROM kContactAlignmentToleranceWorldUnits (never an independently-
    // chosen number) - 3x it, comfortably larger than the contact
    // tolerance itself so the precision zone's own tighter checks/slower
    // speed have real room to operate before Docked can ever be reached,
    // yet still small relative to the staging/reverse-approach distances
    // (never touching the general navigation/staging tolerances - see this
    // fix's own "do not change" list).
    static constexpr float kPrecisionZoneEntryDistanceWorldUnits = kContactAlignmentToleranceWorldUnits * 3.0F;

    // Reverse speed while inside the precision zone - substantially slower
    // than the general kReverseDockSpeed, for the final, most delicate
    // stretch. Derived AT HALF of kReverseDockSpeed (never a second,
    // independently-chosen magic number) - lands at 0.15, inside this
    // fix's own suggested 0.12-0.15 range.
    static constexpr float kPrecisionReverseDockSpeed = kReverseDockSpeed * 0.5F;

    // In-place turn speed for AlignForReverse's own micro-corrections once
    // inside the precision zone - a real, derived necessity, not merely a
    // "go slower for realism" choice: kAligningTurnSpeed's own fixed (bang-
    // bang, non-proportional) rotation produces a per-frame heading step
    // large enough to reliably converge into the GENERAL 5-degree
    // kReverseDockHeadingToleranceDegrees window, but would just as
    // reliably overshoot PAST the much narrower final-contact window
    // (kFinalContactHeadingToleranceDegrees, 2 degrees) every single
    // correction attempt, burning through kMaxPrecisionDockRetries via
    // perpetual overshoot rather than ever actually landing inside it.
    // Derived AT 20% of kAligningTurnSpeed (never a second, independently-
    // chosen number) - small enough that its own per-frame heading step
    // stays comfortably under kFinalContactHeadingToleranceDegrees, so a
    // precision correction can actually converge instead of oscillating.
    static constexpr float kPrecisionAligningTurnSpeed = kAligningTurnSpeed * 0.2F;

    // Final-contact heading tolerance - tighter than
    // kReverseDockHeadingToleranceDegrees (the general reverse-entry
    // tolerance), only enforced once inside the precision zone. Derived AT
    // 40% of kReverseDockHeadingToleranceDegrees (never an independently-
    // chosen number) - lands at 2.0 degrees, inside this fix's own
    // suggested 1.0-2.0 degree range.
    static constexpr float kFinalContactHeadingToleranceDegrees = kReverseDockHeadingToleranceDegrees * 0.4F;

    // Final-contact lateral tolerance - tighter than
    // kReverseLateralErrorToleranceWorldUnits (the general reverse-approach
    // tolerance), only enforced once inside the precision zone. Derived AT
    // HALF of kContactAlignmentToleranceWorldUnits (never an independently-
    // chosen number, and never looser than it - the same "must stay
    // tighter than the contact tolerance" rule
    // kReverseLateralErrorToleranceWorldUnits's own docs already establish,
    // applied again here at an even tighter final-approach margin) - lands
    // at 0.0225, exactly this fix's own suggested "<=50% of contact
    // tolerance."
    static constexpr float kFinalLateralToleranceWorldUnits = kContactAlignmentToleranceWorldUnits * 0.5F;

    // Maximum number of CONSECUTIVE precision-zone corrections (heading or
    // lateral drifting back outside the tighter final-contact tolerances
    // while already inside the precision zone) tolerated before this
    // controller gives up on precision docking entirely and reports
    // Failed, per this fix's own "bounded retries, deterministic, no
    // random behavior" brief - a stricter, harsher escape than
    // kMaxReverseFallbackStreak's own (which merely hands back to Stage 1
    // for a fresh approach): repeatedly failing to hold the TIGHT
    // precision-zone tolerances after this many tries signals a genuinely
    // unrecoverable geometry, not a transient drift.
    static constexpr int kMaxPrecisionDockRetries = 8;

    // Named margin on top of HomeNavigator::kHomeArrivalRadius (never an
    // arbitrary second radius) - see kDockStagingCaptureRadius's own docs.
    static constexpr float kDockCaptureMarginWorldUnits = 0.20F;

    // Phase 13Y dock-capture handoff fix (real-GUI-traced): the radius,
    // around the exact staging point, within which the caller (see
    // DockCaptureRegion.hpp's own docs) may consider this controller
    // eligible to take over from Stage 1 WITHOUT waiting for
    // WaypointNavigatorState::Arrived specifically. A real GUI trace proved
    // relying on Stage 1's own (tighter) arrival flag alone is not robust:
    // the dock housing sits close enough to the staging point that
    // ReactiveObstacleAvoidance can legitimately trigger on final approach,
    // repeatedly dragging the robot's pose around under
    // DriveAuthority::AutonomousAvoidance (a strictly higher tier than
    // Navigation) for many frames, which can prevent WaypointNavigator from
    // ever cleanly reporting Arrived (or leave it Failed) even though the
    // robot is, physically, right next to a perfectly valid docking
    // position. Deliberately LARGER than HomeNavigator::kHomeArrivalRadius
    // (0.40) - by kDockCaptureMarginWorldUnits - so a robot Stage 1 was
    // ever going to consider "close enough" is unconditionally inside this
    // radius too; see DockCaptureRegion.hpp's own docs for the additional
    // physical-safety/hazard-attribution conditions the caller layers on
    // top before actually treating the robot as captured.
    static constexpr float kDockStagingCaptureRadius = HomeNavigator::kHomeArrivalRadius + kDockCaptureMarginWorldUnits;

    // Advances the controller by one frame/step. `enabled` mirrors
    // HomeNavigator/WaypointNavigator's own contract - false immediately
    // resets to Inactive (and clears the capture latch - see
    // DockApproachOutput::captured's own docs). `arrivedAtStagingPoint` is
    // the caller's Stage-1-complete-OR-dock-capture-eligible signal - true
    // either when WaypointNavigatorState::Arrived (Stage 1's own report,
    // its own goal being computeDockStagingPoint()) OR when the caller's
    // own DockCaptureRegion::isDockCaptureEligible() check passes (see that
    // function's own docs for the full physical-safety/hazard-attribution
    // contract).
    //
    // Phase 13Y dock-capture LATCH fix (real-GUI-traced): this parameter
    // now ONLY ever gates ENTRY into a captured session - the very first
    // update() call it is true while `enabled` latches
    // DockApproachOutput::captured for the rest of the session (see that
    // field's own docs for the full real-traced defect this closes). Once
    // captured, every SUBSEQUENT frame's `arrivedAtStagingPoint` value is
    // read (still latches the flag if newly true, harmlessly a no-op once
    // already latched) but no longer required to REMAIN true for the
    // controller to keep progressing - continuation safety is enforced
    // entirely by this controller's own existing checks (lateral/heading
    // tolerance, kReentryDistanceMultiplier's own big-displacement
    // fallback, contact alignment), never by re-deriving the caller's
    // per-frame entry snapshot. This controller never recomputes the ENTRY
    // fact itself, so it can never disagree with the caller about
    // whether/why a session began - it only ever asks "has the caller ever
    // told me it was my turn" once per session.
    DockApproachOutput update(const RobotPose& pose, const BasePlatform& base, const TableSurface& tableSurface,
                               bool enabled, bool arrivedAtStagingPoint) noexcept;

    // Explicit reset to Inactive - mirrors HomeNavigator::reset(). Also
    // clears the capture latch (see DockApproachOutput::captured's own
    // docs) - an explicit reset always ends the current session, exactly
    // like `enabled=false` does.
    void reset() noexcept;

    DockApproachState state() const noexcept;

private:
    DockApproachState state_ = DockApproachState::Inactive;

    // Phase 13Y dock-capture LATCH fix - see DockApproachOutput::captured's
    // own docs for the full "why." Set true the first frame
    // `arrivedAtStagingPoint` is true; cleared back to false on
    // reset()/enabled=false, on the big-displacement reentry fallback (see
    // kReentryDistanceMultiplier's own docs), and on
    // kMaxReverseFallbackStreak's own stuck-cycle escape - every one of
    // those is itself an explicit "this session needs fresh re-validation"
    // event, never a momentary per-frame flicker.
    bool everArrivedAtStaging_ = false;

    // Phase 13Y dock-capture LATCH fix - see kMaxReverseFallbackStreak's
    // own docs. Counts CONSECUTIVE ReverseApproach->AlignForReverse
    // fallbacks; reset to 0 on reaching Docked or on either latch-release
    // path above (a fresh session attempt deserves a fresh count).
    int reverseFallbackStreak_ = 0;

    // Phase 13Y final docking visual-precision polish - see
    // kMaxPrecisionDockRetries's own docs. Counts CONSECUTIVE precision-
    // zone corrections (distinct from reverseFallbackStreak_ above - a
    // stricter, later-stage counter that only increments once inside the
    // tighter final-contact precision zone); reset to 0 on reaching Docked
    // or on reset()/enabled=false.
    int precisionCorrectionCount_ = 0;

    // Phase 13Y final docking visual-precision polish - a LATCH (see
    // update()'s own docs for why a raw per-frame re-derivation is not
    // robust here), mirroring everArrivedAtStaging_'s own shape: set true
    // the first frame the robot is genuinely close to physical contact,
    // cleared on the same session-ending/re-validation events
    // everArrivedAtStaging_ itself clears on (reset()/enabled=false, the
    // big-displacement reentry fallback, kMaxReverseFallbackStreak's own
    // stuck-cycle escape) - deliberately NOT cleared merely on reaching
    // Docked, since a docked-then-briefly-displaced robot is still
    // genuinely close and should stay in precision mode, never suddenly
    // fall back to the general (coarser) tolerances for that.
    bool inPrecisionZoneLatched_ = false;
};

} // namespace robot::visual
