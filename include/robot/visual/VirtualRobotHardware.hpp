#pragma once

#include <string_view>

#include "robot/IRobotHardware.hpp"
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
// VirtualWorld. Sensor reads are fixed/safe for this phase (full battery,
// no obstacle, no emergency stop) - no obstacle sensing, collision, or
// battery drain exists yet; see docs/technical-decisions.md (Phase 13N).
//
// VirtualRobotHardware has no raylib dependency of its own - it depends
// only on IRobotHardware and VirtualWorld's plain data, exactly like
// SimulatedRobotHardware/RealRobotHardware depend on nothing beyond their
// own respective boundaries.
class VirtualRobotHardware : public IRobotHardware
{
public:
    // world must outlive this object - the same non-owning-reference
    // pattern used throughout this codebase. No heap allocation.
    explicit VirtualRobotHardware(VirtualWorld& world);

    int batteryLevelPercent() const override;
    bool obstacleDetected() const override;
    bool emergencyStopPressed() const override;

    void moveForward() override;
    void stop() override;
    void returnToBase() override;

    // Current actuator command, for HUD/observability - mirrors
    // SimulatedRobotHardware::currentCommand()'s role; not part of
    // IRobotHardware.
    VirtualDriveCommand currentCommand() const noexcept;

    // Advances VirtualWorld's robot pose by one simulated tick of
    // `deltaSeconds`: moves the robot forward along its current heading
    // at a fixed speed when the current command is MoveForward, and does
    // nothing when Stopped or ReturnToBase - return-to-base navigation is
    // not implemented yet, so it is deliberately treated the same as
    // Stopped for this phase (see docs/technical-decisions.md, Phase
    // 13N). Robot position is clamped to the demo world's bounds so it
    // never moves indefinitely far away; this is a simple bound, not
    // collision detection.
    void update(float deltaSeconds);

private:
    VirtualWorld& world_;
    VirtualDriveCommand command_ = VirtualDriveCommand::Stopped;
};

} // namespace robot::visual
