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

// Who currently owns the physical wheel speeds DifferentialDrive executes
// (Phase 13Q) - deliberately distinct from currentCommand(), which is only
// ever what RobotController/the FSM *wants*. Priority is fixed and always
// Manual > AutonomousAvoidance > Fsm; see
// VirtualRobotHardware::driveAuthority() and
// docs/technical-decisions.md (Phase 13Q) for the full authority model.
enum class DriveAuthority
{
    Fsm,
    AutonomousAvoidance,
    Manual
};

// Visual-only, not part of any FSM/RobotState convention - mirrors
// VirtualDriveCommand's own toString() shape. These exact strings ("FSM"/
// "AUTONOMOUS"/"MANUAL") are what the HUD displays - deliberately never
// "FSM" for an autonomous-avoidance turn merely because the FSM's
// WaitingForObstacleClear state is what triggered it (see section 16 of
// the Phase 13Q brief).
constexpr std::string_view toString(DriveAuthority authority) noexcept
{
    switch (authority)
    {
        case DriveAuthority::Fsm: return "FSM";
        case DriveAuthority::AutonomousAvoidance: return "AUTONOMOUS";
        case DriveAuthority::Manual: return "MANUAL";
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
//
// As of Phase 13Q, a second override - the autonomous-avoidance override
// (setAutonomousWheelSpeeds()/clearAutonomousWheelOverride()) - sits
// between the manual override and the FSM command in a fixed, explicit
// priority: Manual > AutonomousAvoidance > Fsm (see driveAuthority()).
// Both overrides are tracked independently and never destroyed by an
// unrelated actor: RobotController's moveForward()/stop()/returnToBase()
// calls always update currentCommand(), but only ever change the
// *physical* wheel speeds when neither override is active; clearing the
// manual override falls through to the autonomous override if one is
// still active, and clearing the autonomous override falls through to
// the FSM command if manual is not active - see applyEffectiveWheelSpeeds()
// in the .cpp and docs/technical-decisions.md (Phase 13Q).
class VirtualRobotHardware : public IRobotHardware
{
public:
    // Forward wheel speed FSM MoveForward commands, in world units/second
    // - "approximately 1.0" per the Phase 13P brief. Exposed publicly
    // (Phase 13Q) so other code - e.g. ReactiveObstacleAvoidanceTests -
    // can compare a turn speed's magnitude against it without duplicating
    // this value.
    static constexpr float kForwardWheelSpeed = 1.0F;

    // world must outlive this object - the same non-owning-reference
    // pattern used throughout this codebase. No heap allocation.
    explicit VirtualRobotHardware(VirtualWorld& world);

    int batteryLevelPercent() const override;
    bool obstacleDetected() const override;
    bool emergencyStopPressed() const override;

    // moveForward()/stop()/returnToBase() always update currentCommand()
    // (below) regardless of override state, so the FSM's intent is never
    // lost - only physical wheel speeds are affected by an active manual
    // or autonomous-avoidance override (see driveAuthority()).
    void moveForward() override;
    void stop() override;
    void returnToBase() override;

    // Current actuator command, for HUD/observability - mirrors
    // SimulatedRobotHardware::currentCommand()'s role; not part of
    // IRobotHardware. This is what RobotController/the FSM *wants* -
    // deliberately distinct from driveAuthority() (who currently owns the
    // physical wheels) and wheelSpeeds() (what is actually executing).
    VirtualDriveCommand currentCommand() const noexcept;

    // Current forward-sensor reading, in world units - nullopt when no
    // enabled obstacle is within VirtualDistanceSensor::kMaximumRange.
    // Visual-simulator-only telemetry getter (HUD/rendering), not part of
    // IRobotHardware - mirrors currentCommand()'s role (Phase 13O).
    std::optional<float> obstacleDistance() const;

    // Current left/right wheel speeds actually driving movement -
    // whichever of manual override, autonomous-avoidance override, or
    // currentCommand()'s mapped speeds currently has drive authority (see
    // driveAuthority()). Visual-simulator-only telemetry getter (HUD), not
    // part of IRobotHardware (Phase 13P/13Q).
    WheelSpeeds wheelSpeeds() const noexcept;

    // Who currently owns wheelSpeeds() - Manual > AutonomousAvoidance >
    // Fsm, fixed priority, never ambiguous. Visual-simulator-only
    // telemetry getter (HUD "drive authority"), not part of
    // IRobotHardware (Phase 13Q).
    DriveAuthority driveAuthority() const noexcept;

    // True while a manual wheel override (below) is active - equivalent
    // to driveAuthority() == DriveAuthority::Manual. Visual-simulator-only
    // telemetry getter, not part of IRobotHardware (Phase 13P).
    bool manualOverrideActive() const noexcept;

    // True while an autonomous-avoidance wheel override (below) is
    // active - note this can be true even while driveAuthority() reports
    // Manual (an active manual override does not clear a pending
    // avoidance request; see clearManualWheelOverride() below). Visual-
    // simulator-only telemetry getter, not part of IRobotHardware (Phase
    // 13Q).
    bool autonomousOverrideActive() const noexcept;

    // Visual-simulator-only debug/test control (Phase 13P): temporarily
    // overrides the physical wheel speeds DifferentialDrive uses, without
    // changing currentCommand() or touching IRobotHardware/RobotController/
    // RobotRuntime/RobotStateMachine in any way. Intended caller:
    // RobotSimulator3D's manual-drive-mode keyboard controls only - not
    // part of the normal FSM-driven actuator path. Takes drive authority
    // away from an active autonomous-avoidance override without clearing
    // it (see clearManualWheelOverride()).
    void setManualWheelSpeeds(float left, float right) noexcept;

    // Ends the manual override. If an autonomous-avoidance override is
    // still active, drive authority immediately falls through to it (its
    // last-commanded wheel speeds, unchanged); otherwise it falls through
    // to the wheel speeds corresponding to currentCommand() (equal
    // positive for MoveForward, zero for Stopped/ReturnToBase) - see
    // moveForward()/stop()/returnToBase() above.
    void clearManualWheelOverride() noexcept;

    // Visual-simulator-only autonomous-avoidance control (Phase 13Q):
    // temporarily overrides the physical wheel speeds DifferentialDrive
    // uses, without changing currentCommand() or touching IRobotHardware/
    // RobotController/RobotRuntime/RobotStateMachine in any way - the same
    // shape as setManualWheelSpeeds(), one priority level below it.
    // Intended caller: RobotSimulator3D's reactive-obstacle-avoidance
    // policy (main3d.cpp), driven by ReactiveObstacleAvoidance's wheel
    // speeds - never called from inside VirtualRobotHardware itself. Has
    // no effect on which wheel speeds are physically executed while a
    // manual override is active (Manual still wins), but is still
    // recorded so it takes effect the moment the manual override clears.
    void setAutonomousWheelSpeeds(float left, float right) noexcept;

    // Ends the autonomous-avoidance override. If a manual override is
    // still active, nothing about physical wheel speeds changes (Manual
    // was already winning); otherwise drive authority falls through to
    // the wheel speeds corresponding to currentCommand().
    void clearAutonomousWheelOverride() noexcept;

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
    void applyEffectiveWheelSpeeds() noexcept;

    VirtualWorld& world_;
    VirtualDistanceSensor sensor_;
    DifferentialDrive drive_;
    VirtualDriveCommand command_ = VirtualDriveCommand::Stopped;
    bool manualOverrideActive_ = false;
    WheelSpeeds manualSpeeds_{};
    bool autonomousOverrideActive_ = false;
    WheelSpeeds autonomousSpeeds_{};
    bool collidedLastUpdate_ = false;
};

} // namespace robot::visual
