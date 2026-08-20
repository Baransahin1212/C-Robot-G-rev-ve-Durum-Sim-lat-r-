#pragma once

#include "robot/visual/VirtualWorld.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

// Left/right wheel linear speeds, in world units/second (positive = wheel
// spinning forward). Plain data - no behavior of its own.
struct WheelSpeeds
{
    float left = 0.0F;
    float right = 0.0F;
};

// Deterministic, raylib-free differential-drive kinematic model (Phase
// 13P): turns a pair of commanded wheel speeds into changes to a
// RobotPose's position and heading. Has no knowledge of RobotStateMachine,
// Event, IRobotHardware, RobotController, or VirtualWorld beyond the plain
// RobotPose/Vec3 data types it operates on - it never reads or writes
// VirtualWorld itself (that remains VirtualRobotHardware's job, exactly
// like Phase 13N/13O's movement math did) and never bounds-clamps
// position - world bounds are a VirtualRobotHardware/VirtualWorld concern,
// not a kinematics concern. Only includes VisualRobot.hpp for
// RobotDimensions::kBodyWidth (the wheel-track default) - that header has
// no raylib dependency of its own, matching the precedent
// VirtualDistanceSensor.cpp already set by including it for
// RobotDimensions::kBodyLength.
//
// Uses the project's one heading convention throughout (see
// VisualMath.hpp's forwardDirection()): headingDegrees 0 faces +Z,
// increasing headingDegrees rotates the front marker from +Z toward +X.
// Equations (see docs/technical-decisions.md, Phase 13P):
//
//   v     = (vRight + vLeft) / 2                  (linear velocity)
//   omega = (vRight - vLeft) / wheelTrack          (angular velocity, rad/s)
//
// Straight motion (|omega| below a small epsilon) integrates as
// dx = sin(theta)*v*dt, dz = cos(theta)*v*dt, matching Phase 13N/13O's
// existing forward-movement math exactly when both wheel speeds are equal.
// Turning motion integrates the exact constant-wheel-speed arc (not a
// discrete Euler step), so heading and position stay consistent regardless
// of delta-time size. Wheel speeds change immediately when commanded - no
// acceleration/inertia/friction model exists.
class DifferentialDrive
{
public:
    // Default wheel track (distance between the left/right wheel centers,
    // in world units), derived from RobotDimensions::kBodyWidth
    // (VisualRobot.hpp) - the same source of truth Renderer3D/VisualRobot
    // use to place the wheels at +-kBodyWidth/2, so this never duplicates
    // an independently-guessed geometry constant. ~0.60F, close to the
    // ~0.55F this phase's brief suggests as a reasonable default.
    static constexpr float kDefaultWheelTrack = RobotDimensions::kBodyWidth;

    // wheelTrack must be > 0. Defaults to kDefaultWheelTrack; an explicit
    // value exists so tests can compare angular rate across different
    // tracks (a real per-robot geometry parameter, not a global constant)
    // without needing a second class.
    explicit DifferentialDrive(float wheelTrack = kDefaultWheelTrack) noexcept;

    // Commands both wheel speeds immediately (no ramping).
    void setWheelSpeeds(float left, float right) noexcept;

    // Current commanded wheel speeds.
    WheelSpeeds wheelSpeeds() const noexcept;

    // Equivalent to setWheelSpeeds(0.0F, 0.0F).
    void stop() noexcept;

    // Wheel track this instance uses, in world units.
    float wheelTrack() const noexcept;

    // Advances `pose` by `deltaSeconds` of motion at the current wheel
    // speeds: mutates pose.position.x/z and pose.headingDegrees in place
    // (pose.position.y is never touched - movement stays on the ground
    // plane). headingDegrees is normalized into [0, 360) afterward so it
    // never grows or shrinks without bound across many calls.
    void update(RobotPose& pose, float deltaSeconds) const noexcept;

private:
    float wheelTrack_;
    WheelSpeeds speeds_;
};

} // namespace robot::visual
