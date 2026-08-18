#pragma once

#include <vector>

namespace robot::visual
{

// Minimal project-owned 3D vector - deliberately not raylib's Vector3.
// VirtualWorld (pure world/scene data) has zero raylib dependency; only
// Renderer3D/VisualRobot convert to raylib's Vector3, at the point where
// drawing actually happens. World convention: X = horizontal, Y = up
// (ground is Y = 0), Z = depth - the robot moves conceptually on the X/Z
// plane. See docs/technical-decisions.md (Phase 13M).
struct Vec3
{
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

// Pose of the visual robot: world-space position plus a heading in
// degrees, measured as a rotation around the world Y axis where 0 means
// facing +Z. Phase 13M never mutates headingDegrees after construction -
// the robot is stationary; a future phase will drive this from
// RobotStateMachine, not this struct.
struct RobotPose
{
    Vec3 position;
    float headingDegrees = 0.0F;
};

// An axis-aligned box obstacle: position is its center, size is its full
// width/height/depth along X/Y/Z respectively.
struct BoxObstacle
{
    Vec3 position;
    Vec3 size;
};

// The docking/base platform - a flat box marker for Phase 13M, with no
// docking/navigation logic behind it yet.
struct BasePlatform
{
    Vec3 position;
    Vec3 size;
};

// Deterministic, hard-coded demo scene (Phase 13M): a stationary robot, a
// handful of box obstacles, and one base platform, all on the Y = 0
// ground plane. VirtualWorld is pure data/state - it owns no raylib type
// and performs no drawing; see Renderer3D for that. No physics, FSM, or
// sensor integration exists yet - see docs/technical-decisions.md.
class VirtualWorld
{
public:
    // Constructs the fixed Phase 13M demo scene: identical every time,
    // deliberately - no randomness, no configuration file.
    VirtualWorld();

    const RobotPose& robotPose() const noexcept;
    const BasePlatform& basePlatform() const noexcept;
    const std::vector<BoxObstacle>& obstacles() const noexcept;

private:
    RobotPose robotPose_;
    BasePlatform basePlatform_;
    std::vector<BoxObstacle> obstacles_;
};

} // namespace robot::visual
