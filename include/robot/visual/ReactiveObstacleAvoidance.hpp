#pragma once

#include "robot/visual/DifferentialDrive.hpp"

namespace robot::visual
{

// Deterministic, raylib-free reactive obstacle-avoidance policy (Phase
// 13Q): while WaitingForObstacleClear persists because the forward sensor
// still reports an obstacle, this is the fixed pair of wheel speeds that
// rotates the robot in place until the sensor clears - see
// docs/technical-decisions.md (Phase 13Q) for the full policy and its
// explicit limitations. This is REACTIVE avoidance only - not pathfinding,
// not A*, not waypoint planning, not SLAM/mapping, not full navigation:
// "turn until the forward path clears, then continue" is the entire
// policy, and it may permanently change the robot's heading with no
// attempt to return to its original trajectory.
//
// Has no knowledge of RobotStateMachine, Event, IRobotHardware,
// VirtualDistanceSensor, or VirtualWorld - the decision of *when* to apply
// these wheel speeds (avoidance enabled, current FSM state, sensor
// reading) lives entirely in main3d.cpp, exactly like ManualDriveInput's
// decision function stays separate from main3d.cpp's raylib input
// polling. This class only ever answers "what wheel speeds does an
// avoidance turn use" - it never mutates VirtualRobotHardware,
// VirtualWorld, or anything else itself.
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
    // project's one heading convention (VisualMath.hpp). V1 always turns
    // this one deterministic direction - no obstacle-side clearance
    // probing, no randomness, per the Phase 13Q brief's explicit
    // preference.
    static constexpr float kTurnWheelSpeed = 0.6F;

    // Always returns {-kTurnWheelSpeed, +kTurnWheelSpeed} - deterministic
    // and stateless, so calling it repeatedly is always safe.
    // (left + right) / 2 == 0, so DifferentialDrive integrates this as
    // pure in-place rotation: zero linear velocity, non-zero angular
    // velocity - the robot's center stays fixed while its heading
    // changes.
    WheelSpeeds avoidanceWheelSpeeds() const noexcept;
};

} // namespace robot::visual
