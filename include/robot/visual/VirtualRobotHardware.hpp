#pragma once

#include <optional>
#include <string_view>

#include "robot/IRobotHardware.hpp"
#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VirtualDistanceSensor.hpp"
#include "robot/visual/VirtualObstacleSensorArray.hpp"
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
// (Phase 13Q; extended Phase 13S; extended Phase 13T) - deliberately
// distinct from currentCommand(), which is only ever what
// RobotController/the FSM *wants*. Priority is fixed and always
// Safety > Manual > AutonomousAvoidance > Navigation > Fsm; see
// VirtualRobotHardware::driveAuthority() and
// docs/technical-decisions.md (Phase 13Q/13S/13T) for the full authority
// model. Safety (Phase 13S, TableEdgeSafetyController) sits above even
// Manual - a physical robot must never be driveable off a table edge,
// including under direct human control. Navigation (Phase 13T,
// HomeNavigator) sits just above the plain Fsm command - a Return Home
// mission still yields to Manual/AutonomousAvoidance/Safety exactly like
// every other command source, but otherwise steers the robot itself
// rather than merely driving the fixed straight-line MoveForward speed
// currentCommand() == ReturnToBase would otherwise map to (see
// wheelSpeedsForCommand() in the .cpp).
enum class DriveAuthority
{
    Fsm,
    Navigation,
    AutonomousAvoidance,
    Manual,
    Safety
};

// Visual-only, not part of any FSM/RobotState convention - mirrors
// VirtualDriveCommand's own toString() shape. These exact strings ("FSM"/
// "NAVIGATION"/"AUTONOMOUS"/"MANUAL") are what the HUD displays -
// deliberately never "FSM" for an autonomous-avoidance turn merely
// because the FSM's WaitingForObstacleClear state is what triggered it
// (see section 16 of the Phase 13Q brief), and likewise never "FSM" for a
// HomeNavigator-steered turn/drive merely because ReturningHome is the
// FSM state that made Navigation active.
constexpr std::string_view toString(DriveAuthority authority) noexcept
{
    switch (authority)
    {
        case DriveAuthority::Fsm: return "FSM";
        case DriveAuthority::Navigation: return "NAVIGATION";
        case DriveAuthority::AutonomousAvoidance: return "AUTONOMOUS";
        case DriveAuthority::Manual: return "MANUAL";
        case DriveAuthority::Safety: return "SAFETY";
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
// As of the Phase 13S manual-validation bugfix, obstacleDetected() is no
// longer backed by VirtualDistanceSensor's single center ray alone - a
// solid obstacle offset from the robot's exact centerline, but still
// intersecting its actual body-width forward path, could pass beside that
// one ray undetected, so the robot would keep receiving MoveForward until
// RobotCollision (a last-resort guard, never meant to be the primary
// obstacle signal) finally rejected a pose. obstacleDetected() is now the
// OR of a three-ray VirtualObstacleSensorArray (perception coverage
// across the body width) and a width-aware ForwardClearanceProbe hazard
// check with its own short, independently-derived lookahead (closes the
// gaps between the three discrete rays) - see effectiveObstacleHazard()
// in the .cpp and docs/technical-decisions.md (manual-validation bugfix)
// for the full derivation, and why it deliberately does not reuse
// ForwardClearanceProbe::kLookaheadDistance (that value remains reserved
// for ReactiveObstacleAvoidance's release condition). obstacleDistance()
// below is unchanged - still exactly VirtualDistanceSensor's own single
// center-ray reading.
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
//
// As of Phase 13S, a third override - the safety override
// (setSafetyWheelSpeeds()/clearSafetyWheelOverride()) - sits ABOVE the
// manual override, the single highest-priority wheel-speed source:
// Safety > Manual > AutonomousAvoidance > Fsm. It is driven by
// TableEdgeSafetyController's cliff-sensor-triggered emergency recovery
// (main3d.cpp), and follows the exact same never-destroyed-by-an-
// unrelated-actor / falls-through-on-clear pattern as the manual and
// autonomous overrides: clearing it falls through to manual if still
// active, otherwise to autonomous if still active, otherwise to the FSM
// command - see applyEffectiveWheelSpeeds() and
// docs/technical-decisions.md (Phase 13S).
//
// As of Phase 13S, update() also validates a proposed pose against a
// second, independent guard beyond the existing obstacle-collision one:
// a table-support fail-safe (see update()'s own docs below) - entirely
// separate from RobotCollision, since a table edge is not a solid
// obstacle.
//
// As of Phase 13T, a fourth override - the navigation override
// (setNavigationWheelSpeeds()/clearNavigationWheelOverride()) - sits
// between the autonomous-avoidance override and the FSM command:
// Safety > Manual > AutonomousAvoidance > Navigation > Fsm. It is driven
// by HomeNavigator's Aligning/Driving wheel speeds (main3d.cpp) while a
// Return Home mission is in progress, and follows the exact same never-
// destroyed-by-an-unrelated-actor / falls-through-on-clear pattern as the
// other three overrides: clearing it falls through to autonomous if
// still active, otherwise manual, otherwise safety, otherwise the FSM
// command - see applyEffectiveWheelSpeeds() and
// docs/technical-decisions.md (Phase 13T). HomeNavigator itself never
// calls this override - main3d.cpp does, and only while it also
// determines Navigation is currently the FSM/controller's intent (see
// HomeNavigator.hpp).
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

    // Manual-validation bugfix: this frame's three-ray perception
    // readings (FrontLeft/FrontCenter/FrontRight) - FrontCenter is
    // geometrically identical to obstacleDistance()/the value
    // obstacleDetected() used to be based on alone. Visual-simulator-
    // only telemetry getter (HUD), not part of IRobotHardware.
    ObstacleSensorArrayReadings obstacleSensorReadings() const;

    // Manual-validation bugfix: true when the width-aware forward-body
    // corridor hazard check (a ForwardClearanceProbe query using this
    // class's own short, detection-purpose lookahead - see
    // effectiveObstacleHazard() in the .cpp) currently reports blocked,
    // independent of whether any individual ray in
    // obstacleSensorReadings() detected anything. This is one of the two
    // inputs obstacleDetected() ORs together; exposed separately so the
    // HUD can show which signal is actually driving detection. Visual-
    // simulator-only telemetry getter, not part of IRobotHardware.
    bool bodyCorridorObstacleHazard() const;

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

    // True while a safety wheel override (below) is active - the highest-
    // priority override, can be true even while a manual and/or
    // autonomous request is also recorded underneath (neither is cleared
    // by an active safety override; see clearSafetyWheelOverride()
    // below). Visual-simulator-only telemetry getter, not part of
    // IRobotHardware (Phase 13S).
    bool safetyOverrideActive() const noexcept;

    // True while a navigation wheel override (below) is active - can be
    // true even while driveAuthority() reports something higher
    // (Safety/Manual/AutonomousAvoidance), exactly like
    // autonomousOverrideActive() can be true while Manual wins: none of
    // these overrides destroy each other, only applyEffectiveWheelSpeeds()
    // decides which one is currently PHYSICALLY applied. Visual-
    // simulator-only telemetry getter, not part of IRobotHardware (Phase
    // 13T).
    bool navigationOverrideActive() const noexcept;

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

    // Visual-simulator-only safety control (Phase 13S): temporarily
    // overrides the physical wheel speeds DifferentialDrive uses, without
    // changing currentCommand() or touching IRobotHardware/RobotController/
    // RobotRuntime/RobotStateMachine in any way - the same shape as
    // setManualWheelSpeeds()/setAutonomousWheelSpeeds(), but the single
    // HIGHEST priority level, above even the manual override. Intended
    // caller: RobotSimulator3D's table-edge safety policy (main3d.cpp),
    // driven by TableEdgeSafetyController's recovery wheel speeds - never
    // called from inside VirtualRobotHardware itself. Wins over an active
    // manual override without clearing it, so manual driving resumes
    // automatically the instant safety releases if the user is still
    // holding a manual command.
    void setSafetyWheelSpeeds(float left, float right) noexcept;

    // Ends the safety override. Falls through to whichever of manual,
    // autonomous-avoidance, or the FSM command is next in priority and
    // still active - the exact same falls-through pattern
    // clearManualWheelOverride()/clearAutonomousWheelOverride() already
    // use.
    void clearSafetyWheelOverride() noexcept;

    // Visual-simulator-only navigation control (Phase 13T): temporarily
    // overrides the physical wheel speeds DifferentialDrive uses, without
    // changing currentCommand() or touching IRobotHardware/RobotController/
    // RobotRuntime/RobotStateMachine in any way - the same shape as
    // setAutonomousWheelSpeeds(), one priority level below it (and below
    // Manual/Safety), but above the plain Fsm command. Intended caller:
    // RobotSimulator3D's HomeNavigator-driven Return Home policy
    // (main3d.cpp) - never called from inside VirtualRobotHardware itself,
    // and never decided by HomeNavigator on its own (see HomeNavigator.hpp).
    // Has no effect on which wheel speeds are physically executed while a
    // higher-priority override is active, but is still recorded so it
    // takes effect the moment that override clears.
    void setNavigationWheelSpeeds(float left, float right) noexcept;

    // Ends the navigation override. If Safety/Manual/AutonomousAvoidance is
    // still active, nothing about physical wheel speeds changes (it was
    // already winning); otherwise drive authority falls through to the
    // wheel speeds corresponding to currentCommand().
    void clearNavigationWheelOverride() noexcept;

    // Advances VirtualWorld's robot pose by one simulated tick of
    // `deltaSeconds` via DifferentialDrive, using whichever wheel speeds
    // wheelSpeeds() currently reports (safety/manual override, autonomous
    // override, or FSM command). The proposed position is clamped to a
    // generic ~10-unit simulation-coordinate bound (unchanged from Phase
    // 13N - a purely defensive numeric safety net, not the tabletop
    // safety boundary; see docs/technical-decisions.md, Phase 13S), then
    // validated against TWO independent guards before being committed:
    //   1. enabled obstacle geometry (Phase 13P - RobotCollision.hpp)
    //   2. the table-support fail-safe (Phase 13S - see below)
    // If EITHER guard rejects it, only the position is rejected (kept at
    // its previous value) - the proposed heading is still committed,
    // since the robot's circular collision footprint is rotation-
    // independent, so in-place/combined turning is never blocked by a
    // translation rejection.
    //
    // Table-support fail-safe (Phase 13S): rejects the proposed position
    // only when computeCliffSensorReadings() reports ALL FOUR footprint
    // corners off the table (VirtualCliffSensor.hpp's `allCliff()`) - a
    // genuine full-footprint fall/tunneling event, not merely one corner
    // overhanging the edge (which is the normal, expected, transient
    // state while TableEdgeSafetyController is actively recovering, and
    // must never be blocked by this guard - see
    // docs/technical-decisions.md, Phase 13S). This is a last-resort
    // backstop against an unusually large `deltaSeconds` skipping past
    // the edge in one step, not the primary table-edge-avoidance
    // behavior - that is TableEdgeSafetyController's job, engaged well
    // before this guard would ever trigger, exactly like
    // VirtualDistanceSensor/HardwareEventSource's existing Phase 13O
    // stop-before-contact behavior versus RobotCollision's own guard.
    // Return-to-base navigation is not implemented yet, so it is
    // deliberately treated the same as Stopped (zero wheel speeds) for
    // this phase (see docs/technical-decisions.md, Phase 13N/13P).
    void update(float deltaSeconds);

    // True when the most recent update() call rejected the proposed
    // position due to obstacle collision (RobotCollision.hpp) - never
    // true for a table-support rejection (see tableEdgeRejectedLastUpdate()
    // below); the two are deliberately distinct telemetry, matching the
    // distinct guards that produce them. Visual-simulator-only telemetry
    // getter (HUD), not part of IRobotHardware (Phase 13P).
    bool collidedLastUpdate() const noexcept;

    // True when the most recent update() call rejected the proposed
    // position because the table-support fail-safe guard fired (Phase
    // 13S) - never true for an obstacle-collision rejection. Visual-
    // simulator-only telemetry getter (HUD), not part of IRobotHardware.
    bool tableEdgeRejectedLastUpdate() const noexcept;

private:
    WheelSpeeds wheelSpeedsForCommand(VirtualDriveCommand command) const noexcept;
    void applyEffectiveWheelSpeeds() noexcept;
    bool effectiveObstacleHazard() const;

    VirtualWorld& world_;
    VirtualDistanceSensor sensor_;
    DifferentialDrive drive_;
    VirtualDriveCommand command_ = VirtualDriveCommand::Stopped;
    bool manualOverrideActive_ = false;
    WheelSpeeds manualSpeeds_{};
    bool autonomousOverrideActive_ = false;
    WheelSpeeds autonomousSpeeds_{};
    bool safetyOverrideActive_ = false;
    WheelSpeeds safetySpeeds_{};
    bool navigationOverrideActive_ = false;
    WheelSpeeds navigationSpeeds_{};
    bool collidedLastUpdate_ = false;
    bool tableEdgeRejectedLastUpdate_ = false;
};

} // namespace robot::visual
