#pragma once

#include <optional>
#include <string_view>

#include "raylib.h"

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Plain, raylib-free display/telemetry values passed into renderFrame() -
// main3d converts the real RobotState/VirtualDriveCommand/
// VirtualDistanceSensor readings into this struct so Renderer3D never needs
// to depend on RobotStateMachine, RobotController, IRobotHardware,
// HardwareEventSource, or VirtualDistanceSensor directly (Phase 13N/13O) -
// it stays a sibling target to robot_visual_simulation, not layered on it.
// sensorOrigin/sensorDirection/obstacleDistance/sensorMaximumRange come
// straight from a VirtualDistanceSensor instance main3d owns, so the
// rendered sensor ray's origin, heading, and length always match the actual
// sensor calculation exactly - Renderer3D never re-derives this geometry
// itself.
struct VisualTelemetry
{
    std::string_view stateText;
    std::string_view commandText;
    Vec3 sensorOrigin;
    Vec3 sensorDirection;
    std::optional<float> obstacleDistance;
    bool obstacleDetected = false;
    float sensorMaximumRange = 0.0F;

    // Phase 13P: the wheel speeds actually driving movement this frame
    // (whichever of manual override, autonomous-avoidance override, or
    // FSM command currently has drive authority - see
    // VirtualRobotHardware::wheelSpeeds()), plus a display label for who
    // currently owns them ("FSM"/"AUTONOMOUS"/"MANUAL" - see
    // VirtualRobotHardware::driveAuthority(), Phase 13Q). Renderer3D only
    // ever displays these; it has no wheel-speed/authority logic of its
    // own.
    float leftWheelSpeed = 0.0F;
    float rightWheelSpeed = 0.0F;
    std::string_view driveAuthorityText;

    // Phase 13P: true when VirtualRobotHardware::update() most recently
    // rejected a proposed position due to obstacle collision (see
    // RobotCollision.hpp) - debug telemetry only, so the collision guard's
    // effect is visible in the HUD without needing console spam.
    bool collidedLastUpdate = false;

    // Phase 13Q: whether RobotSimulator3D's reactive-obstacle-avoidance
    // policy is currently enabled (the `A` toggle in main3d.cpp) -
    // independent of whether it is actively turning the robot right now
    // (see avoidanceActive below for that).
    bool avoidanceEnabled = false;

    // Phase 13R: whether ReactiveObstacleAvoidance's latch is currently
    // engaged this frame (main3d.cpp's avoidance.active()) - distinct
    // from driveAuthorityText, which can briefly still read "AUTONOMOUS"
    // even one frame after this turns false (the override is cleared the
    // same frame active() goes false, so in practice they change
    // together, but they are conceptually different questions: this is
    // "does the avoidance policy want the wheels," driveAuthorityText is
    // "who currently has them"). Makes the key Phase 13R transitional
    // state - the latch remaining active after the FSM has already
    // returned to Moving - directly visible in the HUD.
    bool avoidanceActive = false;

    // Phase 13R: this frame's ForwardClearanceProbe::isForwardCorridorClear()
    // reading - true when the robot's physical body has a safe forward
    // corridor along its current heading, independent of (and generally
    // lagging behind) obstacleDetected() above, which only reflects a
    // single forward sensor ray. This is the value that actually gates
    // avoidanceActive's release.
    bool forwardClearanceClear = false;

    // Phase 13R: ForwardClearanceProbe::kLookaheadDistance, surfaced so
    // the HUD can display it - Renderer3D never computes clearance
    // geometry itself, it only ever displays already-computed telemetry.
    float clearanceLookahead = 0.0F;
};

// Owns the Camera3D and draws one complete frame - ground, grid,
// obstacles, base platform, robot, and a 2D HUD overlay - for a given
// VirtualWorld. Does not create or destroy the raylib window itself; that
// is main3d's responsibility (see docs/technical-decisions.md, Phase
// 13M), so a Renderer3D must only be constructed/used while a window is
// already open. Has no knowledge of RobotStateMachine, RobotRuntime,
// RobotController, or IRobotHardware - it only ever reads VirtualWorld's
// plain pose/obstacle/base data, plus the two already-formatted display
// strings passed into renderFrame() (Phase 13N), never a FSM/hardware
// type directly. This keeps robot_visual free of any dependency on
// robot_visual_simulation - the two are sibling targets under
// RobotSimulator3D, not layered on each other.
class Renderer3D
{
public:
    Renderer3D();

    // Draws one complete frame for `world`. When `updateCamera` is true,
    // the camera is first advanced per raylib's built-in CAMERA_FREE
    // controls (mouse look, WASD, scroll zoom); when false, the camera is
    // left exactly as it was on the previous frame - main3d decides
    // `updateCamera` based on whether mouse capture (DisableCursor()) is
    // currently active, so the camera never drifts while the cursor has
    // been released for normal desktop use. `telemetry` carries the
    // already-formatted state/command text plus sensor readings - the
    // caller (main3d) is responsible for converting the real
    // RobotState/VirtualDriveCommand/VirtualDistanceSensor values, so this
    // class never needs to know any of those types. Call exactly once per
    // iteration of the main render loop, between InitWindow() and
    // CloseWindow().
    void renderFrame(const VirtualWorld& world, bool updateCamera, const VisualTelemetry& telemetry);

private:
    void drawScene(const VirtualWorld& world, const VisualTelemetry& telemetry) const;
    void drawHud(const VirtualWorld& world, const VisualTelemetry& telemetry) const;

    Camera3D camera_;
};

} // namespace robot::visual
