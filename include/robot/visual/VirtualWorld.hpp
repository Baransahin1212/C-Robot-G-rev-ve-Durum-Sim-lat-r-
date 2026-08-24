#pragma once

#include <cstddef>
#include <string_view>
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
// docking/navigation logic behind it yet. Phase 13W: this remains the one
// semantic "home" representation `HomeNavigator`/`HomeArrivalEventSource`/
// the exploration map's home marker all target - Renderer3D now draws it
// as a richer charging-dock visual (see Renderer3D.cpp's
// drawChargingDock()), but that is presentation only; this struct itself,
// and every navigation/arrival semantic built on it, is unchanged. See
// docs/technical-decisions.md (Phase 13W) for why a separate ChargingDock
// world-model type was deliberately NOT introduced.
struct BasePlatform
{
    Vec3 position;
    Vec3 size;
};

// Phase 13W: the small, fixed set of recognizable desktop objects the demo
// workspace is dressed with - Renderer3D dispatches on this to draw a
// monitor/keyboard/mouse/mug/notebook/lamp-base instead of a generic box.
// Purely a presentation-dispatch tag: nothing outside Renderer3D (no
// sensor, collision, or mapping code) ever reads DeskObjectType - see
// DeskObject's own docs below for why.
//
// Phase 13W final workspace redesign: the PRODUCTION desk (see
// VirtualWorld.cpp's constructor) now instantiates only Monitor/Keyboard/
// Mouse - an explicit product requirement for a clean, minimal desk
// (mug/notebook/lamp-base read as clutter). Mug/Notebook/LampBase remain
// defined here (and Renderer3D still keeps their own drawX() helpers)
// purely so a hypothetical future scene/test can still dress a desk with
// them without reintroducing the type - never unnecessary churn for its
// own sake - but nothing in the default constructor pushes one anymore.
enum class DeskObjectType
{
    Monitor,
    Keyboard,
    Mouse,
    Mug,
    Notebook,
    LampBase
};

// Visual-only, raylib-free - mirrors every other visual-simulation enum's
// own toString() shape in this codebase.
constexpr std::string_view toString(DeskObjectType type) noexcept
{
    switch (type)
    {
        case DeskObjectType::Monitor: return "Monitor";
        case DeskObjectType::Keyboard: return "Keyboard";
        case DeskObjectType::Mouse: return "Mouse";
        case DeskObjectType::Mug: return "Mug";
        case DeskObjectType::Notebook: return "Notebook";
        case DeskObjectType::LampBase: return "LampBase";
    }
    return "Unknown";
}

// Phase 13W: a semantic desktop object - separate from its physical
// collision geometry (a BoxObstacle, registered alongside it - see
// VirtualWorld.cpp's constructor) and separate from rendering (Renderer3D
// interprets `type` to choose a drawMonitor()/drawKeyboard()/.../
// drawLampBase() helper - see Renderer3D.hpp/.cpp). `position`/`size` are
// this object's FOOTPRINT - identical to the BoxObstacle registered for it
// in VirtualWorld::obstacles(), so collision/sensing and this semantic
// record never disagree about where the object physically is; each
// drawX() helper derives its own taller/richer visual proportions from
// this footprint plus small fixed internal constants (e.g. a monitor's
// screen height is not `size.y` - only the STAND footprint is registered
// as collision geometry, deliberately smaller than the full visual
// screen+neck+stand volume, so the robot is never blocked by a monitor's
// screen floating above head height - see docs/technical-decisions.md,
// Phase 13W, "why simple AABB proxies are used").
//
// No sensor, collision, or mapping code anywhere in this codebase ever
// reads DeskObject or DeskObjectType - VirtualDistanceSensor/
// VirtualObstacleSensorArray/ForwardClearanceProbe/RobotCollision/
// ExplorationMapper all continue to see ONLY the plain BoxObstacle
// registered alongside each DeskObject, exactly as before this phase (see
// ExplorationMapperTests.cpp's MapperDoesNotRequireVirtualWorldReference
// and this phase's own new DeskObjectTypeNeverReachesExplorationMapper-
// style regression test) - this is what keeps progressive exploration
// mapping honest: the map can only ever discover occupied GEOMETRY, never
// a semantic label.
struct DeskObject
{
    DeskObjectType type = DeskObjectType::Monitor;
    Vec3 position;
    Vec3 size;
    bool enabled = true;
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
    // Phase 13W human-visual-redesign v2: the old fixed-generic-cube demo
    // scene (Phase 13M) is gone entirely - the default workspace's ONLY
    // obstacles are the desk objects plus the charging dock's rear
    // housing (see deskObjects()/kDockHousingIndex/deskObjectObstacleIndex()
    // below). There is no more standalone "legacy blocking cube" concept,
    // so `kBlockingObstacleIndex` is retired along with it -
    // RobotSimulator3D's `O` key now toggles a named desk object instead
    // (see deskObjectObstacleIndex()).

    // Phase 13W final workspace redesign: index, in obstacles(), of the
    // charging dock's rear-housing collision box - always the LAST entry
    // pushed in the constructor, i.e. deskObjects().size() (currently 3:
    // Monitor/Keyboard/Mouse - the earlier v2 pass's Mug/Notebook/LampBase
    // are gone). Deliberately NOT a hardcoded literal (a fixed "6" here
    // silently went out of bounds the moment the desk-object count
    // dropped to 3) - this stays correct if that count ever changes
    // again. Renderer3D uses this to draw the housing at its exact
    // collision position - see drawChargingDock().
    static constexpr std::size_t kDockHousingIndex = 3;

    // Phase 13W human-visual-redesign v2: returns the index into
    // obstacles() of the BoxObstacle registered for the (first) DeskObject
    // of `type` - the "named indices / semantic lookup" the brief asks
    // production code and tests to use instead of a raw magic index like
    // the old `kBlockingObstacleIndex`. Returns obstacles().size() (an
    // always-out-of-range sentinel, consistent with
    // setObstaclePosition()/setObstacleEnabled()'s own out-of-range
    // convention) if no enabled-at-construction DeskObject of that type
    // exists - never happens for the six types the default constructor
    // creates, but this stays honest for a hypothetical future world that
    // omits one.
    std::size_t deskObjectObstacleIndex(DeskObjectType type) const noexcept;

    // Constructs the fixed Phase 13M demo scene: identical every time,
    // deliberately - no randomness, no configuration file.
    VirtualWorld();

    const RobotPose& robotPose() const noexcept;
    const BasePlatform& basePlatform() const noexcept;
    const std::vector<BoxObstacle>& obstacles() const noexcept;

    // Phase 13W: the demo desktop workspace's semantic dressing - purely
    // additive over obstacles() above (every DeskObject also has a
    // matching BoxObstacle entry in obstacles(), registered together in
    // the constructor - see DeskObject's own docs for why). Renderer3D is
    // the only consumer; sensors/collision/mapping never call this.
    const std::vector<DeskObject>& deskObjects() const noexcept;

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

    // Phase 13W human-visual-redesign v2: resizes obstacle `index`,
    // leaving its position/enabled flag unchanged. Returns false (no-op)
    // for an out-of-range index - same convention as
    // setObstaclePosition()/setObstacleEnabled(). Added because, since the
    // default workspace's obstacle set is now six differently-sized desk
    // objects plus the dock housing rather than uniform generic boxes, a
    // test that wants "a controlled obstacle of a specific, known size at
    // index 0" (VirtualDistanceSensorTests.cpp's own established
    // disable-everything-then-reposition-one pattern) needs to set that
    // size explicitly rather than relying on whichever desk object
    // happens to occupy that slot.
    bool setObstacleSize(std::size_t index, const Vec3& size);

    // Enables or disables obstacle `index` - see BoxObstacle::enabled.
    // Returns false (no-op) for an out-of-range index.
    bool setObstacleEnabled(std::size_t index, bool enabled);

    // True when obstacle `index` is currently enabled. Returns false for an
    // out-of-range index.
    bool obstacleEnabled(std::size_t index) const;

private:
    // Phase 13W: appends `object` to deskObjects_ AND a matching
    // BoxObstacle (same position/footprint, always enabled at
    // construction) to obstacles_ - the one place a DeskObject and its
    // collision geometry are ever created, so the two can never drift out
    // of sync. Used only by the constructor.
    void addDeskObject(DeskObjectType type, const Vec3& position, const Vec3& size);

    RobotPose robotPose_;
    BasePlatform basePlatform_;
    std::vector<BoxObstacle> obstacles_;
    std::vector<DeskObject> deskObjects_;
    TableSurface tableSurface_;
};

} // namespace robot::visual
