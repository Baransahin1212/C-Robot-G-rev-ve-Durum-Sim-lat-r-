#pragma once

#include <optional>
#include <string_view>

#include "robot/IRobotHardware.hpp"
#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VirtualDistanceSensor.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Current actuator command VirtualRobotHardware is holding - the last
// command RobotController issued through IRobotHardware, translated into
// this simulator's own small drive-command vocabulary. Deliberately not
// SimulatedRobotHardware's RobotCommand: that enum is robot_hardware's own
// observability detail, not a shared vocabulary type, and reusing it here
// would create an undesirable dependency for no benefit - see
// docs/technical-decisions.md (Phase 13N).
enum class VirtualDriveCommand
{
    Stopped,
    MoveForward,
    ReturnToBase
};

// Visual-only, not part of any FSM/RobotState convention - mirrors
// RobotState.hpp's own toString() shape for consistency.
constexpr std::string_view toString(VirtualDriveCommand command) noexcept
{
    switch (command)
    {
        case VirtualDriveCommand::Stopped: return "Stopped";
        case VirtualDriveCommand::MoveForward: return "MoveForward";
        case VirtualDriveCommand::ReturnToBase: return "ReturnToBase";
    }
    return "Unknown";
}

// IRobotHardware implementation that is the 3D visual simulator's
// actuator/sensor boundary: RobotController's moveForward()/stop()/
// returnToBase() calls only ever record the current VirtualDriveCommand
// here - actual VirtualWorld robot-pose movement happens later, once per
// rendered frame, inside update(), the only place this class ever mutates
// VirtualWorld. As of Phase 13O, obstacleDetected() is backed by a real
// VirtualDistanceSensor reading VirtualWorld's obstacle geometry - battery
// and emergency-stop reads remain fixed/safe (full battery, no emergency
// stop); no battery drain or emergency-stop simulation exists yet.
// VirtualRobotHardware only ever exposes sensor readings; it never decides
// FSM transitions or produces Event values itself - that boundary belongs to
// HardwareEventSource, unmodified. See docs/technical-decisions.md (Phase
// 13N/13O).
//
// VirtualRobotHardware has no raylib dependency of its own - it depends
// only on IRobotHardware and VirtualWorld's plain data, exactly like
// SimulatedRobotHardware/RealRobotHardware depend on nothing beyond their
// own respective boundaries.
//
// As of Phase 13P, VirtualRobotHardware owns a DifferentialDrive and
// translates each IRobotHardware actuator call into wheel speeds on it
// (moveForward() -> equal positive wheel speeds, stop()/returnToBase() ->
// zero) instead of computing straight-line movement itself - see
// DifferentialDrive.hpp for the kinematics. It also exposes a small
// visual-simulator-only manual wheel override (setManualWheelSpeeds()/
// clearManualWheelOverride()) so RobotSimulator3D can prove turning
// interactively without IRobotHardware, RobotController, RobotRuntime, or
// RobotStateMachine ever gaining any notion of wheels/turning - see
// docs/technical-decisions.md (Phase 13P).
class VirtualRobotHardware : public IRobotHardware
{
public:
    // world must outlive this object - the same non-owning-reference
    // pattern used throughout this codebase. No heap allocation.
    explicit VirtualRobotHardware(VirtualWorld& world);

    int batteryLevelPercent() const override;
    bool obstacleDetected() const override;
    bool emergencyStopPressed() const override;

    // moveForward()/stop()/returnToBase() always update currentCommand()
    // (below) regardless of manual override state, so the FSM's intent is
    // never lost - only physical wheel speeds are affected by an active
    // manual override (see setManualWheelSpeeds()).
    void moveForward() override;
    void stop() override;
    void returnToBase() override;

    // Current actuator command, for HUD/observability - mirrors
    // SimulatedRobotHardware::currentCommand()'s role; not part of
    // IRobotHardware.
    VirtualDriveCommand currentCommand() const noexcept;

    // Current forward-sensor reading, in world units - nullopt when no
    // enabled obstacle is within VirtualDistanceSensor::kMaximumRange.
    // Visual-simulator-only telemetry getter (HUD/rendering), not part of
    // IRobotHardware - mirrors currentCommand()'s role (Phase 13O).
    std::optional<float> obstacleDistance() const;

    // Current left/right wheel speeds actually driving movement (the
    // manual override's values when active, otherwise currentCommand()'s
    // mapped values) - visual-simulator-only telemetry getter (HUD), not
    // part of IRobotHardware (Phase 13P).
    WheelSpeeds wheelSpeeds() const noexcept;

    // True while a manual wheel override (below) is active. Visual-
    // simulator-only telemetry getter (HUD "drive mode"), not part of
    // IRobotHardware (Phase 13P).
    bool manualOverrideActive() const noexcept;

    // Visual-simulator-only debug/test control (Phase 13P): temporarily
    // overrides the physical wheel speeds DifferentialDrive uses, without
    // changing currentCommand() or touching IRobotHardware/RobotController/
    // RobotRuntime/RobotStateMachine in any way. Intended caller:
    // RobotSimulator3D's manual-drive-mode keyboard controls only - not
    // part of the normal FSM-driven actuator path.
    void setManualWheelSpeeds(float left, float right) noexcept;

    // Ends the manual override and immediately restores the wheel speeds
    // corresponding to currentCommand() (equal positive for MoveForward,
    // zero for Stopped/ReturnToBase) - see moveForward()/stop()/
    // returnToBase() above.
    void clearManualWheelOverride() noexcept;

    // Advances VirtualWorld's robot pose by one simulated tick of
    // `deltaSeconds` via DifferentialDrive, using whichever wheel speeds
    // wheelSpeeds() currently reports (manual override or FSM command).
    // The proposed position is clamped to the demo world's ~10-unit bounds
    // (unchanged from Phase 13N), then validated against enabled obstacle
    // geometry (Phase 13P - see RobotCollision.hpp): if the clamped
    // proposed position would collide, only the position is rejected
    // (kept at its previous value) - the proposed heading is still
    // committed, since the robot's circular collision footprint is
    // rotation-independent, so in-place/combined turning is never blocked
    // by a translation rejection. This is a last-resort physical guard,
    // not a replacement for VirtualDistanceSensor/HardwareEventSource's
    // existing Phase 13O stop-before-contact behavior, which is
    // unmodified and still what normally halts the FSM-driven robot well
    // before this guard would ever trigger. Return-to-base navigation is
    // not implemented yet, so it is deliberately treated the same as
    // Stopped (zero wheel speeds) for this phase (see
    // docs/technical-decisions.md, Phase 13N/13P).
    void update(float deltaSeconds);

    // True when the most recent update() call rejected the proposed
    // position due to obstacle collision. Visual-simulator-only telemetry
    // getter (HUD), not part of IRobotHardware (Phase 13P).
    bool collidedLastUpdate() const noexcept;

private:
    WheelSpeeds wheelSpeedsForCommand(VirtualDriveCommand command) const noexcept;
    void applyCurrentCommandToDrive();

    VirtualWorld& world_;
    VirtualDistanceSensor sensor_;
    DifferentialDrive drive_;
    VirtualDriveCommand command_ = VirtualDriveCommand::Stopped;
    bool manualOverrideActive_ = false;
    bool collidedLastUpdate_ = false;
};

} // namespace robot::visual
