#pragma once

#include <cstddef>
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
// facing +Z. As of Phase 13N, position moves (via
// VirtualRobotHardware::update()) but headingDegrees is still never
// mutated by any production code path - the demo scene has no
// turning/differential-drive logic yet, only straight-line forward
// movement along whatever heading the robot already has.
struct RobotPose
{
    Vec3 position;
    float headingDegrees = 0.0F;
};

// An axis-aligned box obstacle: position is its center, size is its full
// width/height/depth along X/Y/Z respectively. `enabled` (Phase 13O) governs
// both rendering (Renderer3D skips a disabled obstacle) and sensing
// (VirtualDistanceSensor ignores a disabled obstacle) - defaults to true so
// every existing call site that does not mention it behaves exactly as
// before.
struct BoxObstacle
{
    Vec3 position;
    Vec3 size;
    bool enabled = true;
};

// The docking/base platform - a flat box marker for Phase 13M, with no
// docking/navigation logic behind it yet.
struct BasePlatform
{
    Vec3 position;
    Vec3 size;
};

// The rectangular safe tabletop surface the physical robot is confined to
// (Phase 13S) - a distinct concept from any generic simulation-coordinate
// bounds VirtualRobotHardware may also enforce (a much larger, purely
// defensive numeric safety net, never the primary UX - see
// VirtualRobotHardware.cpp). Anything outside this rectangle is NOT a
// solid wall: it represents a drop off the edge of the table, so it is
// deliberately never modeled via RobotCollision's obstacle-AABB
// machinery. See VirtualCliffSensor.hpp (edge detection) and
// TableEdgeSafetyController.hpp (recovery policy) for how this is
// actually used, and docs/technical-decisions.md (Phase 13S) for the
// full rationale.
struct TableSurface
{
    float minX = 0.0F;
    float maxX = 0.0F;
    float minZ = 0.0F;
    float maxZ = 0.0F;
};

// Deterministic, hard-coded demo scene (Phase 13M): a stationary robot, a
// handful of box obstacles, and one base platform, all on the Y = 0
// ground plane. VirtualWorld is pure data/state - it owns no raylib type
// and performs no drawing; see Renderer3D for that. No physics, FSM, or
// sensor integration exists yet - see docs/technical-decisions.md.
class VirtualWorld
{
public:
    // Index of the one demo obstacle deliberately positioned directly in the
    // robot's initial forward path (see VirtualWorld.cpp) - RobotSimulator3D's
    // "O" key toggles this specific obstacle enabled/disabled to demonstrate
    // ObstacleDetected/ObstacleCleared deterministically (Phase 13O).
    static constexpr std::size_t kBlockingObstacleIndex = 3;

    // Constructs the fixed Phase 13M demo scene: identical every time,
    // deliberately - no randomness, no configuration file.
    VirtualWorld();

    const RobotPose& robotPose() const noexcept;
    const BasePlatform& basePlatform() const noexcept;
    const std::vector<BoxObstacle>& obstacles() const noexcept;

    // The fixed Phase 13S demo table surface - see TableSurface above.
    // No production mutator exists (unlike setRobotPosition()/
    // setObstaclePosition()/setObstacleEnabled()): the table's shape is
    // deliberately fixed for the lifetime of one VirtualWorld, matching
    // basePlatform()'s own no-mutator precedent.
    const TableSurface& tableSurface() const noexcept;

    // Moves the robot to `position`, leaving headingDegrees unchanged - no
    // turning/differential-drive logic exists yet (Phase 13N). This is
    // VirtualWorld's only mutation entry point; rendering continues to
    // consume VirtualWorld through a const reference exclusively (see
    // Renderer3D). Intended caller: VirtualRobotHardware::update(), once
    // per rendered frame - not rendering code.
    void setRobotPosition(const Vec3& position);

    // Sets the robot's heading in degrees, leaving position unchanged. No
    // production caller exists yet as of Phase 13N (the demo scene's
    // heading stays fixed, and no turning/differential-drive logic
    // exists) - this exists so VirtualRobotHardware's heading-to-movement
    // direction convention can be tested directly (see
    // VirtualRobotHardwareTests.cpp), and so a future turning phase has a
    // ready mutation point.
    void setRobotHeading(float headingDegrees);

    // Moves obstacle `index` to `position`, leaving its size and enabled
    // flag unchanged. Returns false (no-op) for an out-of-range index.
    // Exists, alongside setObstacleEnabled() below, purely so
    // VirtualDistanceSensor's ray/AABB geometry can be exercised against
    // deterministic, controlled obstacle placements (see
    // VirtualDistanceSensorTests.cpp) without inventing a second,
    // disconnected obstacle representation just for tests - the same
    // rationale as setRobotPosition()/setRobotHeading() (Phase 13N). No
    // production caller moves an obstacle after construction as of Phase
    // 13O; RobotSimulator3D's "O" key only ever calls setObstacleEnabled().
    bool setObstaclePosition(std::size_t index, const Vec3& position);

    // Enables or disables obstacle `index` - see BoxObstacle::enabled.
    // Returns false (no-op) for an out-of-range index.
    bool setObstacleEnabled(std::size_t index, bool enabled);

    // True when obstacle `index` is currently enabled. Returns false for an
    // out-of-range index.
    bool obstacleEnabled(std::size_t index) const;

private:
    RobotPose robotPose_;
    BasePlatform basePlatform_;
    std::vector<BoxObstacle> obstacles_;
    TableSurface tableSurface_;
};

} // namespace robot::visual
