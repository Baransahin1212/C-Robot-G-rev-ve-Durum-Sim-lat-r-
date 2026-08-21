#include "robot/visual/VirtualRobotHardware.hpp"

#include <algorithm>

#include "robot/visual/ForwardClearanceProbe.hpp"
#include "robot/visual/VirtualCliffSensor.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

namespace
{

// The lookahead ForwardClearanceProbe's body-aware corridor test uses
// when reused here as a secondary, width-aware obstacle-DETECTION hazard
// (manual-validation bugfix) - deliberately NOT
// ForwardClearanceProbe::kLookaheadDistance (1.4F), which remains
// reserved exclusively for ReactiveObstacleAvoidance's release condition
// and must not change (docs/technical-decisions.md, Phase 13R).
//
// Derived so a PERFECTLY CENTERED obstacle (directly on the robot's
// heading, zero lateral offset) triggers this hazard at EXACTLY the same
// distance VirtualDistanceSensor's own single center ray already does -
// this widening changes detection COVERAGE (catching an obstacle offset
// enough to miss all three rays but still within the body-width-expanded
// corridor) without changing the existing detection DISTANCE for a
// centered obstacle, so every pre-existing centered-obstacle regression
// test keeps its original timing.
//
// Derivation: the ray's total reach from the robot's CENTER is
// (RobotDimensions::kBodyLength / 2) [sensor origin sits at the robot's
// front] + VirtualDistanceSensor::kDetectionDistance [how close within
// that origin counts as detected]. The corridor's effective reach from
// center to an obstacle's un-expanded near face is
// (kRobotCollisionRadius + ForwardClearanceProbe::kSafetyMargin)
// [clearanceRadius, which the corridor expands the obstacle's AABB by]
// + this lookahead. Setting the two reaches equal and solving for this
// lookahead gives the expression below. (Using the SAME 1.4F lookahead
// as avoidance release here would make this hazard trigger considerably
// earlier than the existing ray for a centered obstacle, breaking
// several pre-existing regression tests that pin specific approach
// distances - see docs/technical-decisions.md, manual-validation
// bugfix, for the worked example that first caught this.)
const float kBodyCorridorHazardLookahead = (RobotDimensions::kBodyLength / 2.0F) +
                                            VirtualDistanceSensor::kDetectionDistance -
                                            (kRobotCollisionRadius + ForwardClearanceProbe::kSafetyMargin);

// Half-extent of a generic SIMULATION-COORDINATE bound (matches
// Renderer3D's own ~10x10 ground/grid) - deliberately larger than, and a
// distinct concept from, VirtualWorld::tableSurface()'s physical tabletop
// safe surface (Phase 13S; see docs/technical-decisions.md, Phase 13S,
// "simulation bounds vs tabletop safety"). This clamp is a purely
// defensive numeric safety net against the robot drifting indefinitely
// far away in raw coordinates - under normal operation the table-edge
// safety system (VirtualCliffSensor/TableEdgeSafetyController,
// engaged well before this bound, plus the table-support fail-safe
// below) is what actually keeps the robot confined; this clamp is not
// expected to ever be the thing that stops the robot in practice.
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
    return effectiveObstacleHazard();
}

bool VirtualRobotHardware::effectiveObstacleHazard() const
{
    // rangeSensorObstacleDetected: perception coverage across the body
    // width (manual-validation bugfix) - true the instant any of the
    // three FrontLeft/FrontCenter/FrontRight rays detects.
    const bool rangeSensorObstacleDetected = VirtualObstacleSensorArray(world_).readings().anyDetected();

    // bodyCorridorBlocked: closes the theoretical gaps a discrete
    // three-ray array still leaves between rays - the SAME body-aware
    // swept-corridor concept ForwardClearanceProbe already uses for
    // avoidance release, reused here with its own short, detection-
    // purpose lookahead (kBodyCorridorHazardLookahead, above) rather than
    // ForwardClearanceProbe::kLookaheadDistance. Only ever considers the
    // forward direction (the corridor segment starts at the robot's
    // current position and extends along its CURRENT heading), so an
    // obstacle behind the robot can never contribute here, exactly like
    // the range rays.
    const bool bodyCorridorBlocked =
        !ForwardClearanceProbe(world_).isForwardCorridorClearWithinDistance(kBodyCorridorHazardLookahead);

    return rangeSensorObstacleDetected || bodyCorridorBlocked;
}

bool VirtualRobotHardware::emergencyStopPressed() const
{
    return false;
}

void VirtualRobotHardware::moveForward()
{
    command_ = VirtualDriveCommand::MoveForward;
    applyEffectiveWheelSpeeds();
}

void VirtualRobotHardware::stop()
{
    command_ = VirtualDriveCommand::Stopped;
    applyEffectiveWheelSpeeds();
}

void VirtualRobotHardware::returnToBase()
{
    command_ = VirtualDriveCommand::ReturnToBase;
    applyEffectiveWheelSpeeds();
}

VirtualDriveCommand VirtualRobotHardware::currentCommand() const noexcept
{
    return command_;
}

std::optional<float> VirtualRobotHardware::obstacleDistance() const
{
    return sensor_.distanceToNearestObstacle();
}

ObstacleSensorArrayReadings VirtualRobotHardware::obstacleSensorReadings() const
{
    return VirtualObstacleSensorArray(world_).readings();
}

bool VirtualRobotHardware::bodyCorridorObstacleHazard() const
{
    return !ForwardClearanceProbe(world_).isForwardCorridorClearWithinDistance(kBodyCorridorHazardLookahead);
}

WheelSpeeds VirtualRobotHardware::wheelSpeeds() const noexcept
{
    return drive_.wheelSpeeds();
}

DriveAuthority VirtualRobotHardware::driveAuthority() const noexcept
{
    if (safetyOverrideActive_)
    {
        return DriveAuthority::Safety;
    }
    if (manualOverrideActive_)
    {
        return DriveAuthority::Manual;
    }
    if (autonomousOverrideActive_)
    {
        return DriveAuthority::AutonomousAvoidance;
    }
    if (navigationOverrideActive_)
    {
        return DriveAuthority::Navigation;
    }
    return DriveAuthority::Fsm;
}

bool VirtualRobotHardware::manualOverrideActive() const noexcept
{
    return manualOverrideActive_;
}

bool VirtualRobotHardware::autonomousOverrideActive() const noexcept
{
    return autonomousOverrideActive_;
}

bool VirtualRobotHardware::safetyOverrideActive() const noexcept
{
    return safetyOverrideActive_;
}

bool VirtualRobotHardware::navigationOverrideActive() const noexcept
{
    return navigationOverrideActive_;
}

void VirtualRobotHardware::setManualWheelSpeeds(float left, float right) noexcept
{
    manualOverrideActive_ = true;
    manualSpeeds_ = WheelSpeeds{left, right};
    applyEffectiveWheelSpeeds();
}

void VirtualRobotHardware::clearManualWheelOverride() noexcept
{
    manualOverrideActive_ = false;
    applyEffectiveWheelSpeeds();
}

void VirtualRobotHardware::setAutonomousWheelSpeeds(float left, float right) noexcept
{
    autonomousOverrideActive_ = true;
    autonomousSpeeds_ = WheelSpeeds{left, right};
    applyEffectiveWheelSpeeds();
}

void VirtualRobotHardware::clearAutonomousWheelOverride() noexcept
{
    autonomousOverrideActive_ = false;
    applyEffectiveWheelSpeeds();
}

void VirtualRobotHardware::setSafetyWheelSpeeds(float left, float right) noexcept
{
    safetyOverrideActive_ = true;
    safetySpeeds_ = WheelSpeeds{left, right};
    applyEffectiveWheelSpeeds();
}

void VirtualRobotHardware::clearSafetyWheelOverride() noexcept
{
    safetyOverrideActive_ = false;
    applyEffectiveWheelSpeeds();
}

void VirtualRobotHardware::setNavigationWheelSpeeds(float left, float right) noexcept
{
    navigationOverrideActive_ = true;
    navigationSpeeds_ = WheelSpeeds{left, right};
    applyEffectiveWheelSpeeds();
}

void VirtualRobotHardware::clearNavigationWheelOverride() noexcept
{
    navigationOverrideActive_ = false;
    applyEffectiveWheelSpeeds();
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

void VirtualRobotHardware::applyEffectiveWheelSpeeds() noexcept
{
    // Fixed priority (Phase 13Q; extended Phase 13S; extended Phase 13T):
    // Safety > Manual > AutonomousAvoidance > Navigation > Fsm.
    // RobotController's moveForward()/stop()/returnToBase() calls (via
    // command_ above) and setManualWheelSpeeds()/setAutonomousWheelSpeeds()/
    // setSafetyWheelSpeeds()/setNavigationWheelSpeeds() all funnel through
    // this one function, so drive authority is never decided by scattered
    // ad-hoc checks elsewhere - this is the single source of truth
    // main3d.cpp relies on instead of duplicating arbitration itself
    // (docs/technical-decisions.md, Phase 13S/13T).
    WheelSpeeds speeds;
    if (safetyOverrideActive_)
    {
        speeds = safetySpeeds_;
    }
    else if (manualOverrideActive_)
    {
        speeds = manualSpeeds_;
    }
    else if (autonomousOverrideActive_)
    {
        speeds = autonomousSpeeds_;
    }
    else if (navigationOverrideActive_)
    {
        speeds = navigationSpeeds_;
    }
    else
    {
        speeds = wheelSpeedsForCommand(command_);
    }

    drive_.setWheelSpeeds(speeds.left, speeds.right);
}

void VirtualRobotHardware::update(float deltaSeconds)
{
    RobotPose pose = world_.robotPose();
    drive_.update(pose, deltaSeconds);

    pose.position.x = clampToWorldBounds(pose.position.x);
    pose.position.z = clampToWorldBounds(pose.position.z);

    const bool collides = robotPositionCollidesWithObstacles(pose.position, world_.obstacles());

    // Table-support fail-safe (Phase 13S): ALL FOUR footprint corners off
    // the table, not merely one - see this function's own docs in the
    // header for why allCliff() (not anyCliff()) is the correct condition
    // here. Deliberately independent of the obstacle-collision check
    // above - a table edge is never modeled as a solid-obstacle AABB.
    const bool completelyOffTable = computeCliffSensorReadings(pose, world_.tableSurface()).allCliff();

    if (collides || completelyOffTable)
    {
        // Reject the translation only - the proposed heading is still
        // committed below, since both guards are evaluated against
        // position alone (the collision footprint is rotation-
        // independent, see RobotCollision.hpp; the table-support corners
        // move with heading, but a rejected translation leaves position -
        // and therefore all four corners - exactly where they already
        // were, so a heading-only change is never affected by this
        // rejection either).
        collidedLastUpdate_ = collides;
        tableEdgeRejectedLastUpdate_ = completelyOffTable;
        world_.setRobotHeading(pose.headingDegrees);
        return;
    }

    collidedLastUpdate_ = false;
    tableEdgeRejectedLastUpdate_ = false;
    world_.setRobotPosition(pose.position);
    world_.setRobotHeading(pose.headingDegrees);
}

bool VirtualRobotHardware::collidedLastUpdate() const noexcept
{
    return collidedLastUpdate_;
}

bool VirtualRobotHardware::tableEdgeRejectedLastUpdate() const noexcept
{
    return tableEdgeRejectedLastUpdate_;
}

} // namespace robot::visual
