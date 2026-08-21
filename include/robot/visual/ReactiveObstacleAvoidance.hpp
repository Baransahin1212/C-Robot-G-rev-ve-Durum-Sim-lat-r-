#pragma once

#include "robot/visual/DifferentialDrive.hpp"

namespace robot::visual
{

// Deterministic, raylib-free reactive obstacle-avoidance policy. As of
// Phase 13R this is a small stateful LATCH, not the stateless Phase 13Q
// "turn while WaitingForObstacleClear + sensor detected" snapshot check -
// see docs/technical-decisions.md (Phase 13R) for why: a single forward
// sensor ray clearing does not mean the robot's physical BODY has a safe
// forward corridor (ForwardClearanceProbe.hpp), so the FSM returning to
// `Moving` on the real `ObstacleCleared` edge must NOT, by itself, hand
// the wheels back to the FSM if the body corridor is still blocked - the
// robot needs to keep turning past that point. This class owns exactly
// that "keep turning until it is actually safe" lifecycle; it still knows
// nothing about RobotStateMachine, Event, IRobotHardware,
// VirtualDistanceSensor, ForwardClearanceProbe, or VirtualWorld - the
// caller (main3d.cpp) computes the three plain booleans update() takes
// and decides *when* to apply avoidanceWheelSpeeds() via
// VirtualRobotHardware::setAutonomousWheelSpeeds()/
// clearAutonomousWheelOverride(), exactly like Phase 13Q. This is still
// REACTIVE avoidance only - not pathfinding, not A*, not waypoint
// planning, not SLAM/mapping, not full navigation: "turn until the
// forward BODY corridor clears, then continue" is the entire policy, and
// it may permanently change the robot's heading with no attempt to
// return to its original trajectory.
class ReactiveObstacleAvoidance
{
public:
    // Fixed in-place turn wheel-speed magnitude, in world units/second -
    // deliberately smaller than VirtualRobotHardware::kForwardWheelSpeed
    // (1.0F) so an avoidance turn reads as a distinct maneuver, not a
    // full-speed spin. left = -kTurnWheelSpeed, right = +kTurnWheelSpeed:
    // by DifferentialDrive's omega = (vRight - vLeft) / wheelTrack
    // convention (verified against TurningDirectionMatchesConvention in
    // DifferentialDriveTests.cpp), this makes omega > 0, which increases
    // headingDegrees - rotating the front marker from +Z toward +X, this
    // project's one heading convention (VisualMath.hpp). Always turns
    // this one deterministic direction - no obstacle-side clearance
    // probing, no randomness, unchanged from Phase 13Q.
    static constexpr float kTurnWheelSpeed = 0.6F;

    // Advances the latch by one frame/step. Semantics (Phase 13R):
    //   - `enabled` false (the `A` toggle off) forces the latch inactive
    //     immediately, regardless of the other two arguments - this is
    //     the only way to force-exit early, matching the brief's
    //     requirement that turning `A` off while active must clear the
    //     override on the spot.
    //   - `triggerAvoidance` true activates the latch (idempotent if
    //     already active). The caller's normal trigger condition (Phase
    //     13Q, unchanged): avoidance enabled AND the FSM is actually
    //     WaitingForObstacleClear AND the forward sensor still reports
    //     the obstacle.
    //   - Once active, the latch stays active across calls - including
    //     calls where `triggerAvoidance` has already gone false, e.g. the
    //     instant the real `ObstacleCleared` edge returns the FSM to
    //     `Moving` - until `forwardCorridorClear` is observed true, at
    //     which point it deactivates. This is the fix for the Phase 13Q
    //     limitation: the FSM reaching `Moving` no longer by itself hands
    //     wheel authority back to the FSM.
    // Deliberately has no knowledge of *why* forwardCorridorClear is
    // true/false - the caller is expected to pass
    // ForwardClearanceProbe::isForwardCorridorClear() each frame.
    void update(bool enabled, bool triggerAvoidance, bool forwardCorridorClear) noexcept;

    // True while the latch is currently engaged - the caller's cue to
    // hold VirtualRobotHardware's autonomous-avoidance wheel override
    // active (see main3d.cpp). False initially (the latch starts
    // inactive) and after any update() call that deactivates it.
    bool active() const noexcept;

    // Always returns {-kTurnWheelSpeed, +kTurnWheelSpeed}, independent of
    // active() - deterministic and stateless in its own right, so calling
    // it is always safe; the caller is responsible for only applying it
    // while active() is true. (left + right) / 2 == 0, so
    // DifferentialDrive integrates this as pure in-place rotation: zero
    // linear velocity, non-zero angular velocity - the robot's center
    // stays fixed while its heading changes.
    WheelSpeeds avoidanceWheelSpeeds() const noexcept;

private:
    bool active_ = false;
};

} // namespace robot::visual
