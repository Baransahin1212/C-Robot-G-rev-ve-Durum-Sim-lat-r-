#pragma once

#include <optional>

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Deterministic, geometry-based, read-only forward distance sensor over
// VirtualWorld's enabled obstacles. Casts a single ray in the robot's
// forward direction (see forwardDirection() in VisualMath.hpp - the exact
// same heading convention VirtualRobotHardware::update() uses for movement)
// from the robot's FRONT position, not its center, and reports the distance
// to the nearest enabled BoxObstacle's X/Z footprint the ray intersects, if
// any within kMaximumRange.
//
// Raylib-free and independent of FSM/Event/IRobotHardware types - it only
// ever reads VirtualWorld's plain pose/obstacle data through a const
// reference, exactly like Renderer3D does, so it stays genuinely headless
// and unit-testable (see VirtualDistanceSensorTests.cpp). It never mutates
// VirtualWorld. VirtualRobotHardware is the one production caller that
// turns its output into IRobotHardware::obstacleDetected() - this class has
// no opinion on Events; that boundary belongs to HardwareEventSource. See
// docs/technical-decisions.md (Phase 13O).
class VirtualDistanceSensor
{
public:
    // Maximum range the sensor can report a hit within, in world units.
    static constexpr float kMaximumRange = 2.5F;

    // Distance at or below which obstacleDetected() below is true. Kept
    // here (not in VirtualRobotHardware) so the sensor and its own
    // detection predicate share one source of truth.
    static constexpr float kDetectionDistance = 1.0F;

    // world must outlive this object - the same non-owning-reference
    // pattern used throughout this codebase.
    explicit VirtualDistanceSensor(const VirtualWorld& world);

    // World-space origin the forward ray is cast from: the robot's center
    // plus forwardDirection(pose) * half the robot's body length (from
    // VisualRobot.hpp's RobotDimensions::kBodyLength, the same source of
    // truth the renderer uses) - the FRONT of the robot, not its center.
    // Exposed so a caller (e.g. Renderer3D's sensor-ray visualization) can
    // draw a sensor-accurate ray without duplicating this geometry.
    Vec3 sensorOrigin() const;

    // World-space forward direction the ray is cast along.
    Vec3 sensorDirection() const;

    // Distance from sensorOrigin() to the nearest enabled obstacle's X/Z
    // footprint the forward ray intersects, ignoring intersections behind
    // the sensor, beyond kMaximumRange, or with a disabled obstacle.
    // std::nullopt when no such obstacle exists - never infinity, never a
    // negative sentinel.
    std::optional<float> distanceToNearestObstacle() const;

    // True exactly when distanceToNearestObstacle() has a value and that
    // value is <= kDetectionDistance.
    bool obstacleDetected() const;

private:
    const VirtualWorld& world_;
};

} // namespace robot::visual
