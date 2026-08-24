#pragma once

#include <array>
#include <optional>

#include "robot/visual/RangeObservation.hpp"
#include "robot/visual/VirtualDistanceSensor.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// The three parallel forward obstacle-perception rays this class models
// (manual-validation bugfix, following Phase 13S): FrontCenter is
// geometrically identical to VirtualDistanceSensor's own single ray (same
// origin/direction/thresholds - kept as a genuinely separate, independent
// computation here rather than reused by reference, matching this
// codebase's established precedent of each geometry component owning its
// own self-contained ray-vs-obstacle math, e.g. ForwardClearanceProbe.cpp
// already duplicates VirtualDistanceSensor.cpp's own slab-test technique
// rather than sharing it). FrontLeft/FrontRight are new: offset laterally
// from center so a solid obstacle that a single narrow center ray can
// pass beside - while still intersecting the robot's actual body-width
// forward path - is no longer invisible to perception. See
// docs/technical-decisions.md (manual-validation bugfix) for the full
// before/after and the exact blind-spot geometry this fixes.
enum class ObstacleRayPosition
{
    FrontLeft,
    FrontCenter,
    FrontRight
};

// Per-ray distance/detection readings plus the aggregate. `detected`
// mirrors VirtualDistanceSensor::obstacleDetected()'s own semantics
// exactly (true when a ray's nearest hit is within
// VirtualDistanceSensor::kDetectionDistance) - reusing that class's
// existing public constants as the "existing range/detection threshold
// semantics" every ray shares, rather than inventing new ones.
struct ObstacleSensorArrayReadings
{
    std::optional<float> frontLeftDistance;
    std::optional<float> frontCenterDistance;
    std::optional<float> frontRightDistance;
    bool frontLeftDetected = false;
    bool frontCenterDetected = false;
    bool frontRightDetected = false;

    // The aggregate IRobotHardware::obstacleDetected() is ultimately
    // built from (VirtualRobotHardware.cpp) - true the instant ANY of
    // the three rays detects, closing the single-center-ray blind spot.
    bool anyDetected() const noexcept
    {
        return frontLeftDetected || frontCenterDetected || frontRightDetected;
    }
};

// Raylib-free, read-only, three-ray forward obstacle-perception array
// (manual-validation bugfix): FrontLeft/FrontCenter/FrontRight rays cast
// in parallel along the robot's forward direction, each independently
// tested against enabled obstacle geometry using the same deterministic
// ray-vs-AABB slab technique VirtualDistanceSensor.cpp uses for its own
// single ray. This widens PERCEPTION coverage across the robot's body
// width; it is still a perception/range-sensor concept, distinct from
// ForwardClearanceProbe's swept-body corridor (continuous, not three
// discrete points) and from RobotCollision's final penetration guard -
// see docs/technical-decisions.md for the full three-way (now four-way,
// counting this class) responsibility split. Reads VirtualWorld's live
// robot pose/obstacle data through a const reference, exactly like
// VirtualDistanceSensor/ForwardClearanceProbe - never mutates
// VirtualWorld, has no FSM/Event/IRobotHardware knowledge of its own.
class VirtualObstacleSensorArray
{
public:
    // How far inside the robot's physical half-body-width
    // (RobotDimensions::kBodyWidth / 2) the left/right ray origins sit,
    // in world units - so rays originate slightly inside the physical
    // body envelope rather than exactly on/outside it (a ray starting
    // exactly on the body edge could, at certain headings, graze past an
    // obstacle actually touching that edge due to floating-point
    // boundary handling; starting slightly inside removes that edge
    // case). Named rather than folded silently into the offset
    // computation, so the intent is visible at the call site.
    static constexpr float kLateralInset = 0.05F;

    // world must outlive this object - the same non-owning-reference
    // pattern used throughout this codebase.
    explicit VirtualObstacleSensorArray(const VirtualWorld& world);

    // All three readings for the current robot pose/obstacle state.
    ObstacleSensorArrayReadings readings() const;

    // World-space origin of one ray - FrontCenter matches
    // VirtualDistanceSensor::sensorOrigin() exactly (robot center +
    // forwardDirection(pose) * half body length); FrontLeft/FrontRight
    // are additionally offset by
    // rightDirection(pose) * ((RobotDimensions::kBodyWidth / 2) -
    // kLateralInset), in the respective direction. Exposed so a caller
    // (Renderer3D's ray visualization) can draw sensor-accurate rays
    // without duplicating this geometry.
    Vec3 rayOrigin(ObstacleRayPosition position) const;

    // World-space forward direction shared by all three rays - they are
    // parallel, not fanned out; only their origins differ.
    Vec3 rayDirection() const;

    // Phase 13V: the same three rays as readings()/rayOrigin() above,
    // repackaged as RangeObservation values (origin/direction/distance/
    // maxRange/hit) in FrontLeft, FrontCenter, FrontRight order - the one
    // sensor-observation surface ExplorationMapper is allowed to consume
    // (see RangeObservation.hpp and ExplorationMapper.hpp's own docs).
    // Reuses this class's own existing ray/AABB intersection computation
    // internally (the same private helper readings() already calls) -
    // never a second, duplicated implementation of the intersection math,
    // and never a new one added inside ExplorationMapper.
    std::array<RangeObservation, 3> observations() const;

private:
    const VirtualWorld& world_;
};

} // namespace robot::visual
