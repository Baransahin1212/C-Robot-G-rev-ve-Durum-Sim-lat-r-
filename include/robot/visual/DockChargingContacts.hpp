#pragma once

#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

// Phase 13Y (precision reverse docking): pure geometry only - no raylib,
// no FSM/controller knowledge. Deliberately lives in robot_visual_world
// (never robot_visual_simulation) so BOTH robot_visual's Renderer3D
// (drawing the two dock pins and two robot rear receivers at their real
// positions - never a separately-eyeballed visual guess, matching this
// project's own established "draw the real computed position" precedent
// for the dock housing/platform) and robot_visual_simulation's
// DockApproachController (deciding whether they are physically ALIGNED)
// can share the exact same derivation - the two can never silently
// disagree about where a contact actually is.
//
// A single small metallic point-contact - position plus a display radius
// (also used as the collision-adjacent visual size, never a physics
// radius: contact ALIGNMENT is decided purely by center-to-center
// distance, see chargingContactsAligned() below).
struct DockChargingContact
{
    Vec3 position;
    float radius = 0.0F;
};

// One left/right pair - named fields (not a generic array) so a caller
// can never accidentally transpose which is which mid-expression.
struct DockChargingContactPair
{
    DockChargingContact left;
    DockChargingContact right;
};

// Contact spacing (Phase 13Y brief: "45-60% of robot body width") - 50%,
// the midpoint of that range, derived FROM RobotDimensions::kBodyWidth
// (never an independently-chosen literal) so it stays correct if the
// robot's own width is ever retuned. At this project's current 0.40F
// body width, this is exactly 0.20F - comfortably inside the suggested
// 0.18-0.22 example range, and comfortably inside the body width itself
// (a full kBodyWidth/4 = 0.10F margin on each side - see
// ChargingContactSpacingFitsRobotWidth/RobotContactsMatchDockSpacing).
inline constexpr float kContactPairSpacingWorldUnits = RobotDimensions::kBodyWidth * 0.5F;

// Absolute world Y (never a per-object local offset) both the dock pins
// and the robot's rear receivers are placed at - the ONE reason this can
// be a single shared constant rather than two independently-tuned
// heights: with a single shared value, "same vertical height" (the Phase
// 13Y brief's own contact-geometry requirement) is true by construction,
// never merely by coincidence, and contact-alignment distance checks
// never need special-casing for Y. Chosen near the table surface (Y=0),
// where a real robot-vacuum-style charging pad/pin realistically sits -
// well below RobotDimensions::kBodyHeight's own midpoint, since these are
// bottom-mounted contacts, not a body-center feature.
inline constexpr float kContactHeightWorldUnits = 0.05F;

// Small, clearly visible but not oversized - see class docs above and
// this project's own existing dock-contact-pad precedent (Renderer3D.cpp,
// pre-Phase-13Y, drew 0.07-wide pad cubes at a similar scale).
inline constexpr float kContactRadiusWorldUnits = 0.025F;

// How close two contact centers must be to count as physically mated -
// small relative to kContactPairSpacingWorldUnits (Phase 13Y brief: "small
// relative to robot width... roughly 0.03-0.06"), the midpoint of that
// suggested range - tight enough that a visibly misaligned approach never
// reports Docked, loose enough to tolerate ordinary simulation-step
// discretization.
inline constexpr float kContactAlignmentToleranceWorldUnits = 0.045F;

// The unit direction (x,z; y always 0, ground-plane only) FROM the dock's
// own position toward the desk-interior/staging side - i.e. away from
// whichever table edge the dock sits nearest to. The one shared "which
// way does this dock face" fact every dock-geometry function in this
// header, and DockApproachController's own staging-point/reverse-heading
// derivation, is built from - never independently re-derived, so none of
// them can ever disagree about the dock's orientation. Matches the same
// placement fact VirtualWorld.cpp's own kRobotStartZ/kDockHousingZ
// constants already assume (see that file's own docs) - derived here
// rather than hardcoded, so this stays correct if the dock's placement
// ever changes. The dock's own ENTRY direction (desk interior INTO the
// dock) is simply the negation of this.
Vec3 dockInwardDirection(const BasePlatform& base, const TableSurface& tableSurface) noexcept;

// The two dock-side charging pins, in world space - derived ONLY from
// `base`/`tableSurface` (the exact same nearest-table-edge dock-axis
// derivation DockApproachController's own staging-point/heading functions
// use - see that header's own docs - so the pins, the staging point, and
// the reverse heading can never disagree about which way the dock faces).
// Centered on the dock's own longitudinal axis, offset laterally by
// kContactPairSpacingWorldUnits/2 along the dock's own entry-facing
// "right" direction (i.e. the lateral axis a robot APPROACHING the dock
// head-on would perceive as right) - see chargingContactsAligned()'s own
// docs for why the ROBOT's rear pair ends up cross-matched against this
// one, never index-for-index, once the robot has actually backed in.
DockChargingContactPair computeDockChargingContacts(const BasePlatform& base,
                                                      const TableSurface& tableSurface) noexcept;

// The two rear-mounted receiver contacts on the robot's OWN body, in
// world space, for `pose` - always on the REAR face (opposite
// forwardDirection(pose)), Left/Right defined relative to the robot's OWN
// current rightDirection(pose) (a real, body-fixed robotics convention -
// these labels rotate WITH the physical robot, exactly like a real
// vehicle's left/right tail-lights), at the exact same
// kContactPairSpacingWorldUnits/kContactHeightWorldUnits the dock pins
// use.
DockChargingContactPair computeRobotRearChargingContacts(const RobotPose& pose) noexcept;

// True only when BOTH dock pins are simultaneously close (within
// `toleranceWorldUnits`) to a distinct one of the two robot contacts -
// i.e. either same-index (dock.left<->robot.left, dock.right<->robot.right)
// OR cross-matched (dock.left<->robot.right, dock.right<->robot.left) is
// fully satisfied; a single matching contact alone (the other pair
// farther than tolerance under BOTH orderings) is never enough - see
// Phase 13Y's own "NO WRONG-SIDE CONTACT" brief. Cross-matching exists
// because dock pins are labeled relative to the dock's fixed
// entry-facing frame while robot contacts are labeled relative to the
// robot's own body frame: when the robot has correctly backed in
// (forward heading pointing AWAY from the dock, per
// DockApproachController's own AlignForReverse target), its body-frame
// "right" ends up on the dock's "left" side and vice versa (a
// deterministic, always-true 180-degree flip - rightDirection(theta+180)
// == -rightDirection(theta) for any theta) - checking both orderings
// makes this function correct regardless of that labeling choice, rather
// than requiring every caller to reason about the flip itself.
// Deliberately position-only - callers that also require heading
// alignment (DockApproachController does, per the brief's own "do not
// consider docking successful from center-distance alone") must check
// that separately; this function has no RobotPose heading of its own to
// judge.
bool chargingContactsAligned(const DockChargingContactPair& dock, const DockChargingContactPair& robot,
                              float toleranceWorldUnits) noexcept;

// Phase 13Y final docking visual-precision polish: the per-side distances
// chargingContactsAligned() only ever collapses into a single pass/fail -
// exposed separately so a caller (DockApproachController's own final
// precision-docking check) can reason about `left`/`right` individually
// (e.g. HUD/diagnostics, or a tighter final-approach acceptance rule) and
// `maxError` (the worse of the two - the natural "how close is the WORST
// contact" summary a single Docked decision needs). Resolves the exact
// same same-order-vs-cross-matched ambiguity chargingContactsAligned()'s
// own docs describe: computes both pairings and returns whichever one is
// actually closer overall (lower max of its own two distances) - so
// `left`/`right` always describe the genuinely-closer physical pairing,
// never an arbitrary fixed choice that could silently report a
// wrong-side-crossed distance as if it were the real one.
struct DockChargingContactErrors
{
    float leftError = 0.0F;
    float rightError = 0.0F;
    float maxError = 0.0F;
};

DockChargingContactErrors computeChargingContactErrors(const DockChargingContactPair& dock,
                                                         const DockChargingContactPair& robot) noexcept;

} // namespace robot::visual
