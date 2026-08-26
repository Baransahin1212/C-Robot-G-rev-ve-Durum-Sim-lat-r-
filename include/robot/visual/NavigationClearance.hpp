#pragma once

#include "robot/visual/RobotCollision.hpp"

namespace robot::visual
{

// Phase 13X final blocker fix - the one shared, raylib-free, VirtualWorld-
// free navigation-clearance contract every layer that reasons about "how
// far from an obstacle is safe" now derives its own radius from, instead of
// each layer separately hand-picking its own magic-number margin on top of
// RobotCollision.hpp's kRobotCollisionRadius (the actual bug this phase's
// integration test caught: GridPathPlanner's old kPlanningSafetyMargin
// (0.05F) and ForwardClearanceProbe/ReactiveObstacleAvoidance's local
// reactive-hazard margin (0.08F) were two independently-chosen numbers with
// no shared derivation, so a route GridPathPlanner planned as "clear" could
// sit inside the local reactive layer's own hazard envelope - see
// docs/technical-decisions.md, Phase 13X final blocker fix, for the full
// traced reproduction).
//
// Deliberately keeps three concepts distinct (never conflated - see this
// phase's own brief):
//   - PHYSICAL COLLISION CLEARANCE (kPhysicalRadius) - RobotCollision's own
//     bare "never overlaps a real obstacle" guard, unchanged.
//   - LOCAL/REACTIVE HAZARD CLEARANCE (kLocalHazardRadius) - what
//     ForwardClearanceProbe/the reactive avoidance corridor treat as "too
//     close, turn away." ForwardClearanceProbe.hpp's own kSafetyMargin is
//     now DEFINED from kLocalHazardMargin below (same 0.08F value it
//     always used - this is a source-of-truth unification, not a behavior
//     change to the reactive layer itself, which this phase's brief
//     explicitly says not to redesign without proof it is the actual
//     defect - the trace proved the mismatch was on the PLANNING side).
//   - PLANNING CLEARANCE (kPlanningRadius) - what GridPathPlanner requires
//     between a candidate route and every occupied/table-edge boundary.
//     These do not need to be numerically identical, but a globally
//     planned nominal path must never sit permanently inside the local
//     hazard envelope - so kPlanningRadius is derived as AT LEAST
//     kLocalHazardRadius, never smaller than it, plus one small additional
//     named planning-only margin.
namespace NavigationClearance
{

// Same source of truth RobotCollision.hpp's own physical guard uses -
// never independently hand-duplicated.
inline const float kPhysicalRadius = kRobotCollisionRadius;

// The local/reactive hazard margin every reactive-layer geometry check
// (ForwardClearanceProbe's own corridor test) adds on top of
// kPhysicalRadius - moved here as the ONE named source of truth
// ForwardClearanceProbe.hpp itself now defines its own kSafetyMargin from,
// rather than a second, independently-chosen literal (see this namespace's
// own top-level docs). Value unchanged from ForwardClearanceProbe's
// pre-existing 0.08F (inside the brief's own suggested 0.05F-0.10F range).
inline constexpr float kLocalHazardMargin = 0.08F;

inline const float kLocalHazardRadius = kPhysicalRadius + kLocalHazardMargin;

// Small additional PLANNING-only margin, on top of kLocalHazardRadius
// itself (not on top of kPhysicalRadius alone, unlike the old, too-small
// kPlanningSafetyMargin this replaces) - the "+ smallPlanningMargin" this
// phase's own brief asks for, so a route A* considers clear is not planned
// exactly AT the local hazard layer's own boundary. Deliberately smaller
// than kLocalHazardMargin itself - this is a second, additional buffer on
// TOP of already matching the reactive layer, not a second independent
// large inflation.
inline constexpr float kPlanningExtraMargin = 0.02F;

// planningClearance = max(physicalCollisionClearance,
// nominalLocalHazardClearance) + smallPlanningMargin (this phase's own
// brief, verbatim) - kLocalHazardRadius is always >= kPhysicalRadius by
// construction (it is kPhysicalRadius plus a strictly non-negative
// margin), so the max() collapses algebraically to kLocalHazardRadius,
// but is still written explicitly below (rather than assumed) so this
// stays correct even if a future change ever made the local hazard
// margin negative for some reason.
inline const float kPlanningRadius =
    (kLocalHazardRadius > kPhysicalRadius ? kLocalHazardRadius : kPhysicalRadius) + kPlanningExtraMargin;

} // namespace NavigationClearance

} // namespace robot::visual
