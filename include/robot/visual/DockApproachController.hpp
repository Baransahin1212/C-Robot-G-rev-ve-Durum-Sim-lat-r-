#pragma once

#include <string_view>

#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/HomeNavigator.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Phase 13X final-approach fix: global A* route planning (GridPathPlanner/
// WaypointNavigator) is deliberately NOT responsible for precision docking -
// it only ever needs to bring the robot to a point comfortably in front of
// the dock, through known Free space, respecting the same clearance/
// avoidance rules every other route does. The last short stretch into the
// actual charging dock is a different kind of maneuver entirely: a straight,
// aligned approach along a single known-safe lane, deliberately passing
// close to the dock's own rear housing (a real, permanent obstacle the
// robot must enter NEAR, not avoid) - see this header's own
// computeDockApproachPoint()/computeDockEntranceHeadingDegrees() docs below
// for how that lane is derived, and DockApproachController's own class docs
// for how the two stages hand off to each other.
enum class DockApproachState
{
    Inactive,

    // Enabled, but the caller has not yet reported the global route has
    // reached the approach point - this controller produces zero wheel
    // speeds and defers entirely to whatever is currently driving Stage 1
    // (WaypointNavigator's own Navigation-tier output).
    NavigatingToApproach,

    // At (or very near) the approach point - rotating in place toward the
    // fixed dock entrance heading. Never drives forward from here; see
    // class docs on "no diagonal entry."
    Aligning,

    // Aligned within tolerance - driving straight forward at the slower
    // kFinalApproachSpeed toward the literal home position.
    FinalApproach,

    // Physically within HomeNavigator::kHomeArrivalRadius of the literal
    // home position - the existing HomeReached/HomeArrivalEventSource-style
    // semantics this controller's own arrival event source builds on.
    // NEVER a one-way "done forever" latch: distance is re-checked every
    // update() call even while Arrived, so a robot displaced away again
    // (e.g. by ordinary avoidance in the brief window before HomeReached
    // is actually consumed, or a Safety recovery) resumes Aligning rather
    // than freezing permanently with zero wheels - a real, traced defect
    // this exact re-check closes (see .cpp docs).
    Arrived,

    // The derived approach point/lane failed its own geometric self-check
    // (see isDockApproachGeometryValid() below) - a defensive contract for
    // a future geometry change, never expected to trigger for this
    // project's current fixed dock/table dimensions (see
    // DockApproachControllerTests.cpp).
    Failed
};

constexpr std::string_view toString(DockApproachState state) noexcept
{
    switch (state)
    {
        case DockApproachState::Inactive: return "Inactive";
        case DockApproachState::NavigatingToApproach: return "NavigatingToApproach";
        case DockApproachState::Aligning: return "Aligning";
        case DockApproachState::FinalApproach: return "FinalApproach";
        case DockApproachState::Arrived: return "Arrived";
        case DockApproachState::Failed: return "Failed";
    }
    return "Unknown";
}

// Everything one update() call reports.
struct DockApproachOutput
{
    WheelSpeeds wheelSpeeds;
    DockApproachState state = DockApproachState::Inactive;
    bool arrived = false;

    // True while THIS controller (not Stage 1's WaypointNavigator) owns
    // producing the Navigation-tier wheel speeds - i.e. Aligning or
    // FinalApproach. The caller (main3d.cpp) uses this both to pick whose
    // wheelSpeeds to apply and to withhold the ordinary obstacle-triggered
    // avoidance trigger for this one validated lane - see class docs,
    // "FINAL APPROACH HAZARD CONTRACT."
    bool driving = false;
};

// Small named margin, on top of the geometric terms below, so the
// approach point sits comfortably clear of the platform's own footprint
// edge rather than exactly on it - same "named margin on top of a derived
// value" precedent as NavigationClearance::kPlanningExtraMargin.
inline constexpr float kDockApproachMarginWorldUnits = 0.15F;

// Deterministic dock approach point (Phase 13X final-approach fix) -
// derived ONLY from BasePlatform/TableSurface geometry plus this project's
// existing robot-size/clearance constants, never a hardcoded/visually
// guessed coordinate. Sits on the platform's own longitudinal centerline
// (matching `base.position` on whichever axis the platform is NOT offset
// toward a table edge on), on the side FACING AWAY from the nearest table
// edge (the same side the platform's own open entrance faces - see
// VirtualWorld.cpp's own kRobotStartZ docs for why that is always the
// "desk interior" side for this project's dock placement), offset out from
// `base.position` by:
//   (platform half-depth along the entrance axis) + RobotCollision::
//   kRobotCollisionRadius (rotation clearance) + kDockApproachMarginWorldUnits
// far enough that the robot's own footprint, centered at the approach
// point, clears the platform's footprint entirely with room to rotate in
// place (Aligning) before ever starting the final straight approach - see
// DockApproachControllerTests.cpp's own DockApproachPointHasRobotClearance/
// DockApproachPointIsOutsideDockHousing for the proof against this
// project's real dock geometry.
Vec3 computeDockApproachPoint(const BasePlatform& base, const TableSurface& tableSurface) noexcept;

// The fixed heading (degrees, this project's 0 = +Z convention) the robot
// must hold to drive straight from computeDockApproachPoint() into
// `base.position` - i.e. the direction FROM the approach point TOWARD the
// platform, derived from the exact same nearest-table-edge axis
// computeDockApproachPoint() uses, so the two can never disagree about
// which way the dock actually faces.
float computeDockEntranceHeadingDegrees(const BasePlatform& base, const TableSurface& tableSurface) noexcept;

// True if the derived approach point sits comfortably inside `tableSurface`
// and is not degenerate (finite, non-zero offset from `base.position`) -
// the "prove the final centerline approach corridor is physically valid"
// self-check this class's own docs promise, evaluated once whenever this
// controller (re)activates. Deliberately does NOT read
// VirtualWorld::obstacles() (the dock housing itself) - by construction,
// the approach point is offset TOWARD the entrance side, strictly farther
// from the nearest table edge (and therefore from the housing, which sits
// on the OPPOSITE, near-edge side of `base.position` - see
// VirtualWorld.cpp's own kDockHousingZ derivation) than `base.position`
// itself, so a geometrically valid approach point can never be closer to
// the housing than the already-tested literal home point is - see
// DockApproachControllerTests.cpp's own DockApproachPointIsOutsideDockHousing
// for the direct proof against this project's real housing geometry.
bool isDockApproachGeometryValid(const BasePlatform& base, const TableSurface& tableSurface) noexcept;

// Deterministic, raylib-free Stage-2 "precision docking" controller (Phase
// 13X final-approach fix) - the counterpart to GridPathPlanner/
// WaypointNavigator's Stage-1 global routing. Has no knowledge of
// RobotStateMachine, Event, IRobotHardware, RobotController, or
// ExplorationMap - exactly like HomeNavigator's own boundary. The caller
// (main3d.cpp) is responsible for:
//   - routing Stage 1 (WaypointNavigator) to computeDockApproachPoint(),
//     never the literal base.position - see main3d.cpp's own docs;
//   - telling this controller when Stage 1 has arrived
//     (`arrivedAtApproachPoint`);
//   - applying this controller's own wheelSpeeds via the SAME Navigation-
//     tier override API WaypointNavigator already uses whenever
//     DockApproachOutput::driving is true (never a new DriveAuthority
//     tier - Safety > Manual > AutonomousAvoidance > Navigation > Fsm is
//     completely unchanged);
//   - withholding the ordinary obstacle-triggered avoidance TRIGGER
//     condition (never avoidance itself, never Safety) while `driving` is
//     true, since this one lane has already been proven physically valid
//     by isDockApproachGeometryValid() - see main3d.cpp's own docs at the
//     call site for the full "why," and this header's own top-level docs.
//
// STATE MACHINE (mirrors HomeNavigator's own Aligning/Driving hysteresis
// shape, one level up): NavigatingToApproach (waiting for Stage 1) ->
// Aligning (rotate in place toward computeDockEntranceHeadingDegrees(),
// shortest-path, no repeated 360 - identical turn-direction convention to
// HomeNavigator/TableEdgeSafetyController) -> FinalApproach (equal wheel
// speeds at kFinalApproachSpeed, slower than HomeNavigator's own
// kNavigationForwardSpeed) -> Arrived (within
// HomeNavigator::kHomeArrivalRadius of the LITERAL base.position - the
// existing, unchanged arrival semantics). If the robot ends up displaced
// materially far from the approach point again (e.g. Safety recovery
// mid-approach), this controller falls back to NavigatingToApproach on its
// own next update() call - Stage 1 naturally re-routes it back, and this
// controller simply recomputes fresh from wherever the robot ends up next
// time Stage 1 reports arrival again, exactly like HomeNavigator's own
// continuous per-frame recomputation (never a cached/stale target).
class DockApproachController
{
public:
    // Heading hysteresis (mirrors HomeNavigator's own two-threshold shape,
    // tighter here since precision docking wants a genuinely straight
    // entry, never a diagonal one - see class docs, "no diagonal entry").
    static constexpr float kDockingHeadingToleranceDegrees = 5.0F;
    static constexpr float kDockingStopToleranceDegrees = 10.0F;

    // In-place turn speed while Aligning - reuses HomeNavigator's own
    // magnitude so every "autonomous-authority turning" maneuver in this
    // codebase's HUD/telemetry still reads consistently (same precedent
    // HomeNavigator's own kNavigationTurnSpeed docs cite).
    static constexpr float kAligningTurnSpeed = HomeNavigator::kNavigationTurnSpeed;

    // Forward speed while FinalApproach - deliberately, namedly slower
    // than ordinary navigation (HomeNavigator::kNavigationForwardSpeed,
    // 0.8F): half speed, precision docking over speed for the last short
    // stretch into the dock. Derived FROM that existing constant (never a
    // second, independently-chosen magic number) so the two stay in a
    // fixed, documented ratio if either is ever retuned.
    static constexpr float kFinalApproachSpeed = HomeNavigator::kNavigationForwardSpeed / 2.0F;

    // If the robot ends up farther than this multiple of the approach
    // offset away from the approach point while Aligning/FinalApproach
    // (e.g. a Safety table-edge recovery displaced it), this controller
    // falls back to NavigatingToApproach rather than continuing to chase a
    // fixed-heading lane from a position that may no longer be on it -
    // Stage 1 (WaypointNavigator, whose own goal is always the approach
    // point) naturally re-routes the robot back.
    static constexpr float kReentryDistanceMultiplier = 2.0F;

    // Advances the controller by one frame/step. `enabled` mirrors
    // HomeNavigator/WaypointNavigator's own contract - false immediately
    // resets to Inactive. `arrivedAtApproachPoint` is the caller's
    // Stage-1-complete signal (WaypointNavigatorState::Arrived while its
    // own goal is computeDockApproachPoint()) - this controller never
    // recomputes that fact itself, so it can never disagree with Stage 1
    // about whether the global route finished.
    DockApproachOutput update(const RobotPose& pose, const BasePlatform& base, const TableSurface& tableSurface,
                               bool enabled, bool arrivedAtApproachPoint) noexcept;

    // Explicit reset to Inactive - mirrors HomeNavigator::reset().
    void reset() noexcept;

    DockApproachState state() const noexcept;

private:
    DockApproachState state_ = DockApproachState::Inactive;
};

} // namespace robot::visual
