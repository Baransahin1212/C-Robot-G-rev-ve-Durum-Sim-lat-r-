#include "robot/visual/VirtualRobotHardware.hpp"

#include <algorithm>

namespace robot::visual
{

namespace
{

// Forward wheel speed, world units per second - "approximately 1.0" per
// the task brief, as a named constant. Both wheels receive this value on
// moveForward(), matching Phase 13N/13O's prior straight-line speed
// exactly when DifferentialDrive integrates equal wheel speeds.
constexpr float kForwardWheelSpeed = 1.0F;

// Half-extent of the demo world's ground plane (matches Renderer3D's own
// ~10x10 ground/grid - see docs/technical-decisions.md, Phase 13M/13N).
// Robot position is clamped to stay within this square so it can never
// drift indefinitely far away; this is a simple bound, independent of
// (and applied before) the obstacle-collision guard below (Phase 13P).
constexpr float kWorldHalfExtent = 10.0F;

float clampToWorldBounds(float value)
{
    return std::max(-kWorldHalfExtent, std::min(kWorldHalfExtent, value));
}

} // namespace

VirtualRobotHardware::VirtualRobotHardware(VirtualWorld& world)
    : world_(world)
    , sensor_(world)
{
}

int VirtualRobotHardware::batteryLevelPercent() const
{
    return 100;
}

bool VirtualRobotHardware::obstacleDetected() const
{
    return sensor_.obstacleDetected();
}

bool VirtualRobotHardware::emergencyStopPressed() const
{
    return false;
}

void VirtualRobotHardware::moveForward()
{
    command_ = VirtualDriveCommand::MoveForward;
    applyCurrentCommandToDrive();
}

void VirtualRobotHardware::stop()
{
    command_ = VirtualDriveCommand::Stopped;
    applyCurrentCommandToDrive();
}

void VirtualRobotHardware::returnToBase()
{
    command_ = VirtualDriveCommand::ReturnToBase;
    applyCurrentCommandToDrive();
}

VirtualDriveCommand VirtualRobotHardware::currentCommand() const noexcept
{
    return command_;
}

std::optional<float> VirtualRobotHardware::obstacleDistance() const
{
    return sensor_.distanceToNearestObstacle();
}

WheelSpeeds VirtualRobotHardware::wheelSpeeds() const noexcept
{
    return drive_.wheelSpeeds();
}

bool VirtualRobotHardware::manualOverrideActive() const noexcept
{
    return manualOverrideActive_;
}

void VirtualRobotHardware::setManualWheelSpeeds(float left, float right) noexcept
{
    manualOverrideActive_ = true;
    drive_.setWheelSpeeds(left, right);
}

void VirtualRobotHardware::clearManualWheelOverride() noexcept
{
    manualOverrideActive_ = false;
    applyCurrentCommandToDrive();
}

WheelSpeeds VirtualRobotHardware::wheelSpeedsForCommand(VirtualDriveCommand command) const noexcept
{
    switch (command)
    {
        case VirtualDriveCommand::MoveForward:
            return WheelSpeeds{kForwardWheelSpeed, kForwardWheelSpeed};
        case VirtualDriveCommand::Stopped:
        case VirtualDriveCommand::ReturnToBase:
            return WheelSpeeds{0.0F, 0.0F};
    }
    return WheelSpeeds{0.0F, 0.0F};
}

void VirtualRobotHardware::applyCurrentCommandToDrive()
{
    // A manual override, while active, owns physical wheel speeds - the
    // FSM command above is still recorded (command_/currentCommand()), so
    // clearManualWheelOverride() can restore it later, but it must not
    // fight the override's wheel speeds every time RobotController calls
    // moveForward()/stop()/returnToBase() while the override is active.
    if (manualOverrideActive_)
    {
        return;
    }

    const WheelSpeeds speeds = wheelSpeedsForCommand(command_);
    drive_.setWheelSpeeds(speeds.left, speeds.right);
}

void VirtualRobotHardware::update(float deltaSeconds)
{
    RobotPose pose = world_.robotPose();
    drive_.update(pose, deltaSeconds);

    pose.position.x = clampToWorldBounds(pose.position.x);
    pose.position.z = clampToWorldBounds(pose.position.z);

    if (robotPositionCollidesWithObstacles(pose.position, world_.obstacles()))
    {
        // Reject the translation only - the proposed heading is still
        // committed below, since the collision footprint is rotation-
        // independent (see RobotCollision.hpp): pure in-place rotation
        // proposes the same position it started from, so it is never
        // affected by this rejection.
        collidedLastUpdate_ = true;
        world_.setRobotHeading(pose.headingDegrees);
        return;
    }

    collidedLastUpdate_ = false;
    world_.setRobotPosition(pose.position);
    world_.setRobotHeading(pose.headingDegrees);
}

bool VirtualRobotHardware::collidedLastUpdate() const noexcept
{
    return collidedLastUpdate_;
}

} // namespace robot::visual
