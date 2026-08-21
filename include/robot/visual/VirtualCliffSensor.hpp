#pragma once

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// The four corners of the robot's rectangular footprint a virtual cliff
// sensor is modeled at (Phase 13S).
enum class CliffSensorPosition
{
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight
};

// Presence/absence of supporting tabletop beneath each of the robot's
// four footprint corners. IMPORTANT SEMANTICS, documented explicitly
// since an inverted convention here would be a dangerous, silent bug in
// a safety-critical component: **true = CLIFF DETECTED** (that corner is
// NOT over the table - unsafe), **false = SAFE** (that corner is over
// the table). This mirrors VirtualDistanceSensor's own true-means-
// hazard `obstacleDetected()` convention.
struct CliffSensorReadings
{
    bool frontLeft = false;
    bool frontRight = false;
    bool rearLeft = false;
    bool rearRight = false;

    // True when either front corner reports a cliff - the condition
    // TableEdgeSafetyController uses to trigger/continue backing away.
    bool anyFrontCliff() const noexcept
    {
        return frontLeft || frontRight;
    }

    // True when either rear corner reports a cliff - the condition
    // TableEdgeSafetyController uses to trigger/continue moving forward
    // away from a rear edge.
    bool anyRearCliff() const noexcept
    {
        return rearLeft || rearRight;
    }

    // True when ANY of the four corners reports a cliff - the condition
    // TableEdgeSafetyController uses to decide whether it must remain
    // engaged (its Turning state does not release until this is false).
    bool anyCliff() const noexcept
    {
        return frontLeft || frontRight || rearLeft || rearRight;
    }

    // True only when ALL FOUR corners report a cliff - i.e. the robot's
    // entire footprint has left the table, not merely one corner
    // overhanging it. This is deliberately a much stricter condition
    // than anyCliff(): a partial overhang is the NORMAL, expected,
    // transient state while TableEdgeSafetyController is actively
    // backing away/moving forward to recover, so a guard that rejected
    // motion on anyCliff() would make recovery impossible. allCliff() is
    // instead the trigger for VirtualRobotHardware's table-support
    // fail-safe guard, which only needs to catch a genuine full-footprint
    // fall/tunneling event caused by an unusually large delta time - see
    // docs/technical-decisions.md (Phase 13S, "table-support fail-safe").
    bool allCliff() const noexcept
    {
        return frontLeft && frontRight && rearLeft && rearRight;
    }
};

// World-space position of one cliff sensor, for an arbitrary `pose` -
// exactly at that corner of the robot's rectangular footprint
// (RobotDimensions::kBodyWidth/kBodyLength, VisualRobot.hpp - the same
// source of truth Renderer3D/VirtualDistanceSensor use, never a
// duplicated body dimension), rotated by pose.headingDegrees using this
// project's one heading convention (forwardDirection()/rightDirection(),
// VisualMath.hpp). Pure function - no VirtualWorld dependency of its own
// beyond the plain RobotPose passed in - so it is reusable both for
// VirtualWorld's live pose (VirtualCliffSensor below) and for a
// not-yet-committed proposed pose (VirtualRobotHardware's table-support
// guard).
Vec3 cliffSensorWorldPosition(const RobotPose& pose, CliffSensorPosition sensorPosition) noexcept;

// True when `point`'s X/Z lies within `table`, INCLUSIVE of the exact
// boundary - a point exactly on the table's edge is still counted as
// supported. A simple, deterministic choice for the boundary case (see
// VirtualCliffSensorTests.cpp's SensorExactlyOnBoundaryHandledDeterministically).
bool isPointOnTable(const Vec3& point, const TableSurface& table) noexcept;

// Computes all four sensor readings for an arbitrary `pose` against an
// arbitrary `table` - the pure geometry both VirtualCliffSensor (live
// VirtualWorld pose) and VirtualRobotHardware's table-support guard (a
// proposed, not-yet-committed pose) share, so the two never independently
// define "on the table."
CliffSensorReadings computeCliffSensorReadings(const RobotPose& pose, const TableSurface& table) noexcept;

// RECOVERY-ONLY geometry (table-edge recovery bugfix #2): true when
// `point` lies inside `table` shrunk inward by `margin` on every side -
// strictly stricter than isPointOnTable() above (whose true=on-table
// semantics represent the ACTUAL physical edge and must never change).
// This exists solely so TableEdgeSafetyController can require a small
// safety buffer before releasing - "barely on the table by a hair at the
// exact inclusive boundary" is not robust recovery. `margin` must be
// smaller than half of both table.maxX - table.minX and
// table.maxZ - table.minZ, or every point would be reported unsafe.
bool isPointSafelyInsideTable(const Vec3& point, const TableSurface& table, float margin) noexcept;

// True when ALL FOUR of the robot's footprint corners (for `pose`) are
// safely inside `table` by `margin` - see isPointSafelyInsideTable()
// above. This is a DIFFERENT question from CliffSensorReadings::anyCliff()
// (the actual physical edge, no margin) and from heading alignment (is
// the robot pointed the right way) - TableEdgeSafetyController combines
// this with its own heading check, never collapsing the two into one
// boolean. See docs/technical-decisions.md (table-edge recovery bugfix
// #2) for the full three-concept split.
bool areAllCornersSafelyInsideTable(const RobotPose& pose, const TableSurface& table, float margin) noexcept;

// Raylib-free, read-only virtual cliff/table-edge sensor (Phase 13S):
// four downward-looking corner sensors modeling the presence/absence of
// supporting tabletop beneath the robot's footprint. This is NOT a
// downward distance/depth sensor (no physics of a real IR/ToF cliff
// sensor is modeled - only presence/absence of support), and it is NOT a
// solid-obstacle sensor (VirtualDistanceSensor's job - see
// docs/technical-decisions.md, Phase 13S, for the full responsibility
// split). Reads VirtualWorld's live robot pose and table surface through
// a const reference, exactly like VirtualDistanceSensor/
// ForwardClearanceProbe - never mutates VirtualWorld, has no FSM/Event/
// IRobotHardware knowledge of its own.
class VirtualCliffSensor
{
public:
    // world must outlive this object - the same non-owning-reference
    // pattern used throughout this codebase.
    explicit VirtualCliffSensor(const VirtualWorld& world);

    CliffSensorReadings readings() const;

    // Exposed so a caller (e.g. an optional Renderer3D sensor
    // visualization) can draw sensor-accurate points without duplicating
    // this geometry.
    Vec3 sensorWorldPosition(CliffSensorPosition sensorPosition) const;

private:
    const VirtualWorld& world_;
};

} // namespace robot::visual
