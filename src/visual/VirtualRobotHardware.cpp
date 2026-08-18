#include "robot/visual/VirtualRobotHardware.hpp"

#include <algorithm>
#include <cmath>

namespace robot::visual
{

namespace
{

// Forward speed, world units per second - "approximately 1.0" per the
// task brief, as a named constant.
constexpr float kMoveSpeed = 1.0F;

// Half-extent of the demo world's ground plane (matches Renderer3D's own
// ~10x10 ground/grid - see docs/technical-decisions.md, Phase 13M/13N).
// Robot position is clamped to stay within this square so it can never
// drift indefinitely far away; this is a simple bound, not collision
// detection - obstacle boxes are not treated as collisions yet.
constexpr float kWorldHalfExtent = 10.0F;

constexpr float kPi = 3.14159265358979323846F;

float clampToWorldBounds(float value)
{
    return std::max(-kWorldHalfExtent, std::min(kWorldHalfExtent, value));
}

} // namespace

VirtualRobotHardware::VirtualRobotHardware(VirtualWorld& world)
    : world_(world)
{
}

int VirtualRobotHardware::batteryLevelPercent() const
{
    return 100;
}

bool VirtualRobotHardware::obstacleDetected() const
{
    return false;
}

bool VirtualRobotHardware::emergencyStopPressed() const
{
    return false;
}

void VirtualRobotHardware::moveForward()
{
    command_ = VirtualDriveCommand::MoveForward;
}

void VirtualRobotHardware::stop()
{
    command_ = VirtualDriveCommand::Stopped;
}

void VirtualRobotHardware::returnToBase()
{
    command_ = VirtualDriveCommand::ReturnToBase;
}

VirtualDriveCommand VirtualRobotHardware::currentCommand() const noexcept
{
    return command_;
}

void VirtualRobotHardware::update(float deltaSeconds)
{
    if (command_ != VirtualDriveCommand::MoveForward)
    {
        return;
    }

    const RobotPose& pose = world_.robotPose();
    const float distance = kMoveSpeed * deltaSeconds;
    const float headingRadians = pose.headingDegrees * (kPi / 180.0F);

    // Matches VisualRobot.cpp's actual rlRotatef(headingDegrees, 0, 1, 0)
    // convention exactly (right-hand rotation around +Y): heading 0 faces
    // +Z, and increasing heading rotates the front marker from +Z toward
    // +X. Movement must follow the same (sin, cos) mapping the renderer
    // already uses, not an assumed one - see
    // HeadingZeroMovesInFrontMarkerDirection/
    // HeadingNinetyMovesInCorrectDirection in
    // VirtualRobotHardwareTests.cpp, which prove this against that same
    // convention.
    Vec3 newPosition = pose.position;
    newPosition.x = clampToWorldBounds(newPosition.x + (std::sin(headingRadians) * distance));
    newPosition.z = clampToWorldBounds(newPosition.z + (std::cos(headingRadians) * distance));

    world_.setRobotPosition(newPosition);
}

} // namespace robot::visual
