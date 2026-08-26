#pragma once

#include "robot/visual/NavigationClearance.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Raylib-free forward BODY-clearance check (Phase 13R). Fixes the Phase
// 13Q limitation documented in docs/technical-decisions.md: "the forward
// sensor ray is clear" is a single-point-ray perception fact, not proof
// that the robot's physical (circular) collision footprint has room to
// move forward - a heading can clear VirtualDistanceSensor's narrow ray
// while still running the robot's collision circle straight into the same
// obstacle's corner a few frames later (RobotCollision.hpp).
// ForwardClearanceProbe answers a different, complementary question:
// "does the swept corridor the robot's BODY would occupy over the next
// kLookaheadDistance world units, starting from its current pose, stay
// entirely outside every enabled obstacle's (safety-margin-expanded)
// footprint?" It has no opinion on perception (VirtualDistanceSensor's
// job) or on the final physical penetration guard (RobotCollision's job,
// still applied unconditionally by VirtualRobotHardware::update() to
// every proposed pose regardless of what this class reports) - see
// docs/technical-decisions.md (Phase 13R) for the three-way split.
//
// Geometry: a Minkowski-sum-style corridor test. The robot's collision
// footprint is a circle (RobotCollision.hpp's kRobotCollisionRadius) plus
// a small safety margin (kSafetyMargin below); rather than sweeping that
// circle along the forward direction and testing it against each
// obstacle's exact X/Z AABB, this expands each obstacle's AABB outward by
// the same clearance radius on every side and tests the finite forward
// CENTER-LINE segment (robot center -> robot center +
// forwardDirection(pose) * kLookaheadDistance) against the expanded box -
// an equivalent, simpler formulation of the same swept-circle-vs-box
// question. Raylib-free and read-only over VirtualWorld's plain pose/
// obstacle data, exactly like VirtualDistanceSensor/RobotCollision - no
// raylib type is ever named here.
class ForwardClearanceProbe
{
public:
    // How far ahead, in world units, the swept-body corridor is checked -
    // long enough that once avoidance releases (isForwardCorridorClear()
    // becomes true), the robot can make visible forward progress before
    // RobotCollision's tighter last-resort guard would need to reject a
    // proposed pose again; not so long that the robot is forced to keep
    // turning well past any obstacle it has already cleared. 1.4F sits
    // inside the brief's suggested 1.25F-1.50F range and, for this
    // project's demo obstacle sizes (~0.6-1.2 world units) and turn
    // radius, was confirmed empirically (see the Phase 13R closed-loop
    // integration test in VirtualRobotHardwareTests.cpp) to let the robot
    // clear the corner it used to get stuck on within a bounded number of
    // avoidance-turn frames.
    static constexpr float kLookaheadDistance = 1.4F;

    // Added on top of RobotCollision.hpp's kRobotCollisionRadius (the
    // same source of truth RobotCollision's own physical guard uses - see
    // that header's docs for its derivation from RobotDimensions) so the
    // corridor this class reports clear is a little more conservative
    // than the exact boundary RobotCollision would reject at - avoidance
    // releases slightly before, not exactly at, the collision guard's own
    // threshold, so a released MoveForward is not immediately right back
    // at the rejection boundary. 0.08F sits inside the brief's suggested
    // 0.05F-0.10F range.
    //
    // Phase 13X final blocker fix: now DEFINED from
    // NavigationClearance::kLocalHazardMargin (NavigationClearance.hpp)
    // instead of its own separately-hand-picked literal - same 0.08F
    // value as before (no behavior change here), but now the ONE shared
    // source of truth GridPathPlanner's own planning clearance is also
    // derived from, so the two layers can never again silently drift
    // apart the way they did before this phase (see
    // NavigationClearance.hpp's own top-level docs for the full
    // reasoning and the traced mismatch this closes).
    static constexpr float kSafetyMargin = NavigationClearance::kLocalHazardMargin;

    // world must outlive this object - the same non-owning-reference
    // pattern VirtualDistanceSensor/RobotCollision use throughout this
    // codebase.
    explicit ForwardClearanceProbe(const VirtualWorld& world);

    // True when no ENABLED obstacle's clearance-radius-expanded X/Z AABB
    // intersects the finite forward center-line segment described above -
    // i.e. the robot's body has a physically safe corridor to move
    // forward along its CURRENT heading for kLookaheadDistance. False the
    // instant any enabled obstacle's expanded footprint intersects that
    // segment, including a tangent/boundary touch (see the .cpp for the
    // exact deterministic tolerance). Disabled obstacles are always
    // ignored, matching VirtualDistanceSensor/RobotCollision's own
    // enabled-only convention. Equivalent to
    // isForwardCorridorClearWithinDistance(kLookaheadDistance).
    bool isForwardCorridorClear() const;

    // Same corridor test as isForwardCorridorClear(), but with an
    // explicit lookahead distance instead of the fixed
    // kLookaheadDistance - added for the manual-validation bugfix so
    // VirtualRobotHardware can reuse this class's body-aware corridor
    // geometry as a secondary, width-aware obstacle-detection hazard
    // signal with its OWN, shorter, independently-derived lookahead,
    // without changing kLookaheadDistance itself (still reserved for
    // ReactiveObstacleAvoidance's release condition - see
    // docs/technical-decisions.md, manual-validation bugfix, for why
    // reusing the same 1.4F lookahead for both purposes would have
    // changed existing centered-obstacle detection distances and risked
    // event oscillation). `lookaheadDistance` must be >= 0.
    bool isForwardCorridorClearWithinDistance(float lookaheadDistance) const;

private:
    const VirtualWorld& world_;
};

} // namespace robot::visual
