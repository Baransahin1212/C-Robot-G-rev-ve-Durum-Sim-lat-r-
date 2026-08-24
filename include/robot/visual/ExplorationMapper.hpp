#pragma once

#include <vector>

#include "robot/visual/ExplorationMap.hpp"
#include "robot/visual/RangeObservation.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Phase 13V: turns this frame's real sensor RangeObservations plus the
// robot's current RobotPose into ExplorationMap updates - the ONLY
// component in this codebase allowed to call ExplorationMap::markFree()/
// markOccupied(). Deliberately knows nothing about VirtualWorld,
// VirtualObstacleSensorArray, or any other sensor/geometry type beyond
// the plain RobotPose/RangeObservation structs update() takes - it never
// takes a `const VirtualWorld&` anywhere in its public API (see
// ExplorationMapperTests.cpp's own MapperDoesNotRequireVirtualWorldReference
// test), so it is architecturally incapable of reading
// VirtualWorld::obstacles() and "cheating" a complete ready-made map into
// existence. Every RangeObservation it consumes must come from a real ray
// cast performed elsewhere (VirtualObstacleSensorArray::observations() in
// production - see main3d.cpp) - this class only ever traces the already-
// measured result, never re-derives or duplicates the ray/AABB
// intersection math itself.
//
// Ray tracing uses an integer Bresenham line algorithm over grid (col,
// row) coordinates (not continuous-space micro-stepping) - deterministic,
// with no frame-rate or step-size dependence: the exact same
// RobotPose/RangeObservation pair always marks the exact same set of
// cells, regardless of how many times or how often update() is called
// (see RepeatedObservationIsIdempotent in ExplorationMapperTests.cpp).
// For a ray that did not hit anything, the traced segment is
// [origin, origin + direction * maxRange), all marked Free (see
// RangeObservation.hpp's own `distance` convention - already `maxRange`
// when `hit` is false, so this class never branches on `hit` to pick
// which field to read). For a ray that did hit, cells strictly before the
// endpoint are marked Free and the endpoint cell itself is marked
// Occupied - never the reverse, and never both on the same cell (the
// endpoint is excluded from the Free pass specifically so it is not
// marked Free immediately before being marked Occupied on the same
// update() call, which would be harmless under ExplorationMap's own
// Occupied-wins precedence, but is avoided anyway for clarity).
class ExplorationMapper
{
public:
    // `map` must outlive this object - the same non-owning-reference
    // pattern every other visual-simulation component in this codebase
    // uses (VirtualDistanceSensor, ForwardClearanceProbe, ...).
    explicit ExplorationMapper(ExplorationMap& map) noexcept;

    // Traces every observation in `observations` (see the class docs
    // above), then marks the small footprint of cells physically under
    // the robot (per RobotDimensions, VisualRobot.hpp) as Free - the
    // robot's own body is, by construction, standing on ground it is
    // physically occupying right now, so this needs no sensor
    // observation of its own to justify (see markRobotFootprint() in the
    // .cpp for the exact rotated-rectangle test used, deliberately NOT a
    // circle - see this phase's own brief, "do not paint a huge circle").
    void update(const RobotPose& pose, const std::vector<RangeObservation>& observations);

private:
    void traceObservation(const RangeObservation& observation);
    void markRobotFootprint(const RobotPose& pose);

    ExplorationMap& map_;
};

} // namespace robot::visual
