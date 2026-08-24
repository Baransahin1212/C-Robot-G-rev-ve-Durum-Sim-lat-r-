#pragma once

#include <optional>
#include <string_view>

#include "raylib.h"

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Presentation-only HUD detail level (UX polish) - toggled by `H` in
// main3d.cpp, has ZERO effect on RobotStateMachine, RobotController,
// RobotRuntime, VirtualRobotHardware, obstacle detection, avoidance,
// table-edge safety, wheel commands, physics, or collision: it only
// changes which already-computed VisualTelemetry fields Renderer3D
// chooses to draw. Deliberately lives here (the presentation layer),
// never in a core robot/domain header.
//
// Final UI/HUD polish: user-facing terminology is now Sade ("simple" -
// a small ~6-8-line operational summary, the new default) and Ayrıntılı
// ("detailed" - the full engineering telemetry panel, unchanged in
// content from before this pass, only relabeled). The enum identifiers
// themselves (Full/Compact) are kept exactly as-is - only drawHud()'s
// Turkish presentation output changed, per this phase's own instruction
// to translate presentation, not rename internal identifiers -
// Full now BACKS the Ayrıntılı (detailed) panel and Compact now BACKS the
// Sade (simple) panel; main3d.cpp's initial hudMode is Compact so Sade is
// what a user sees by default.
enum class HudMode
{
    Full,
    Compact
};

// Visual-only, raylib-free - mirrors VirtualDriveCommand/DriveAuthority's
// own toString() shape. Not used by drawHud() itself (which switches on
// the enum directly), but kept as a small, genuinely headless unit in
// case a caller/test wants a display string without depending on
// raylib - see docs/technical-decisions.md (UX polish) for why a two-
// state UI toggle does not otherwise warrant dedicated unit tests.
constexpr std::string_view toString(HudMode mode) noexcept
{
    switch (mode)
    {
        case HudMode::Full: return "Full";
        case HudMode::Compact: return "Compact";
    }
    return "Unknown";
}

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

    // Manual-validation bugfix: RobotStateMachine::returnHomeReason()'s
    // already-formatted text ("None"/"MissionAbort"/"UserRequest") -
    // optional Full-HUD-only diagnostic, never consulted by any decision
    // in this class; helps a human observer confirm WHY the robot is
    // currently returning home without needing console access.
    std::string_view returnHomeReasonText;
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

    // Phase 13S: this frame's CliffSensorReadings, already computed by
    // main3d.cpp's VirtualCliffSensor - true = cliff detected (that
    // corner is off the table), matching CliffSensorReadings' own
    // documented convention exactly. Renderer3D never computes cliff
    // geometry itself.
    bool cliffFrontLeft = false;
    bool cliffFrontRight = false;
    bool cliffRearLeft = false;
    bool cliffRearRight = false;

    // Phase 13S: whether TableEdgeSafetyController's recovery latch is
    // currently engaged this frame (main3d.cpp's tableEdgeSafety.active()) -
    // the same active()-vs-driveAuthorityText relationship
    // avoidanceActive already has with ReactiveObstacleAvoidance.
    bool edgeSafetyActive = false;

    // Phase 13S: already-formatted TableEdgeSafetyController::state()
    // text (TableEdgeSafetyController.hpp's toString()) - "Inactive"/
    // "BackingAway"/"MovingForwardFromRearEdge"/"Turning". Renderer3D
    // never has any notion of the recovery state machine itself.
    std::string_view edgeRecoveryStateText;

    // Phase 13S bugfix: TableEdgeSafetyController's current recovery
    // target heading (degrees, this project's 0 = +Z / 90 = +X
    // convention) and the live signed error from the robot's current
    // heading to it - already computed by
    // TableEdgeSafetyController::targetRecoveryHeadingDegrees()/
    // currentHeadingErrorDegrees(). Only meaningful while
    // edgeSafetyActive is true; Renderer3D displays them unconditionally
    // regardless (matching every other HUD line's always-shown style),
    // never computing the heading math itself.
    float edgeTargetHeadingDegrees = 0.0F;
    float edgeHeadingErrorDegrees = 0.0F;

    // Manual-validation bugfix: left/right perception-ray telemetry -
    // the CENTER ray reuses sensorOrigin/sensorDirection/obstacleDistance/
    // obstacleDetected above (geometrically identical to
    // VirtualObstacleSensorArray's FrontCenter ray - never recomputed
    // twice). All three rays share sensorDirection (they are parallel,
    // only their origins differ). Renderer3D never computes this
    // geometry itself.
    Vec3 obstacleRayLeftOrigin;
    Vec3 obstacleRayRightOrigin;
    std::optional<float> obstacleRayLeftDistance;
    std::optional<float> obstacleRayRightDistance;
    bool obstacleRayLeftDetected = false;
    bool obstacleRayRightDetected = false;

    // Manual-validation bugfix: VirtualRobotHardware::bodyCorridorObstacleHazard()
    // - the width-aware corridor hazard signal that, ORed with the three
    // rays above, is what obstacleDetected() itself is actually built
    // from. Distinct from forwardClearanceClear above (Phase 13R's
    // avoidance-release corridor, a different, longer lookahead) - see
    // docs/technical-decisions.md (manual-validation bugfix) for why the
    // two must not be conflated.
    bool bodyCorridorObstacleHazard = false;

    // UX polish: VirtualRobotHardware::obstacleDetected()'s own aggregate
    // result (range rays OR body corridor - the exact same value that
    // actually drives HardwareEventSource/the FSM), supplied directly
    // rather than re-derived here so Renderer3D never needs to OR
    // together obstacleDetected/obstacleRayLeftDetected/
    // obstacleRayRightDetected/bodyCorridorObstacleHazard itself - it
    // only ever selects which already-computed fields to display.
    bool obstacleHazard = false;

    // UX polish: which HUD detail level to draw - Full (default,
    // unchanged existing telemetry) or Compact (a small, high-value
    // operational subset). Presentation-only; see HudMode's own docs.
    HudMode hudMode = HudMode::Full;

    // Phase 13T: HomeNavigator's own state/telemetry - already computed
    // by main3d.cpp's HomeNavigator::update() every frame, regardless of
    // whether it is currently enabled (Inactive is a completely normal,
    // most-of-the-time value, not an error state). Renderer3D never has
    // any notion of the Aligning/Driving/Arrived policy itself - it only
    // ever displays these already-computed values.
    std::string_view homeNavigationStateText;
    float homeNavigationDistance = 0.0F;
    float homeNavigationTargetHeadingDegrees = 0.0F;
    float homeNavigationHeadingErrorDegrees = 0.0F;

    // Phase 13T: true while HomeNavigator is actively steering (Aligning
    // or Driving this frame) - never true for Inactive/Arrived. Drives
    // the optional target-direction guide line in drawScene() below.
    // Computed by main3d.cpp from the real HomeNavigationState enum, not
    // re-derived here from homeNavigationStateText - Renderer3D never
    // calculates navigation decisions, only visualizes already-computed
    // data.
    bool homeNavigationGuideVisible = false;

    // Phase 13U: Mission Control panel telemetry - already-formatted
    // MissionTask text ("NONE"/"ROAM"/"RETURN HOME", see
    // MissionTask.hpp::toString()), whether the robot is currently within
    // HomeZoneMonitor's own exit radius, and the live distance to base.
    // Renderer3D never derives task status or Home Zone membership
    // itself - both are computed once in main3d.cpp and displayed
    // verbatim, exactly like every other VisualTelemetry field.
    std::string_view missionTaskText;
    bool homeZoneInside = true;
    float baseDistance = 0.0F;

    // Final UI/HUD polish: true exactly when missionTaskText corresponds
    // to MissionTask::ReturnHome - a plain bool computed once in main3d.cpp
    // (from the real MissionTask enum it already has in scope) rather than
    // Renderer3D string-comparing missionTaskText's Turkish text, matching
    // every other "main3d.cpp decides, Renderer3D only displays" telemetry
    // field above. Drives the simple/Sade HUD's single optional contextual
    // line (distance-to-base, only shown while actually returning home).
    bool returningHomeTask = false;

    // Final UI/HUD polish: VirtualRobotHardware::batteryLevelPercent(),
    // already-read directly from IRobotHardware exactly like every other
    // telemetry field above - Renderer3D never queries hardware itself.
    // Currently always 100 (see VirtualRobotHardware::batteryLevelPercent()'s
    // own docs: no battery-drain simulation exists yet in the visual
    // simulator), wired through now so the simple/Sade HUD's "Batarya"
    // line reflects the real getter rather than a hardcoded display value.
    int batteryPercent = 100;
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

    // Final Turkish-font polish: font_ owns a loaded GPU texture (see
    // constructor), so Renderer3D is no longer trivially copyable/
    // destructible - UnloadFont() must run before the raylib window
    // closes. Not copyable (there is exactly one instance, constructed
    // once in main3d.cpp - a copy would double-free the same GPU texture
    // on destruction); left movable is unnecessary for the same reason,
    // so move is deleted too, matching "not copyable" for simplicity.
    Renderer3D(const Renderer3D&) = delete;
    Renderer3D& operator=(const Renderer3D&) = delete;
    Renderer3D(Renderer3D&&) = delete;
    Renderer3D& operator=(Renderer3D&&) = delete;
    ~Renderer3D();

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

    // Phase 13U: the compact Mission Control panel (task status, 1/2/3/R
    // key hints, Home Zone status) - deliberately separate from drawHud()'s
    // engineering telemetry panel: positioned top-right (never fighting
    // the existing top-left panel) and drawn unconditionally regardless of
    // HudMode, so task controls stay visible in both Full and Compact.
    void drawMissionControlPanel(const VisualTelemetry& telemetry) const;

    // Final Turkish-font polish: thin wrappers around raylib's
    // DrawTextEx()/MeasureTextEx() using font_ (below) plus a fixed
    // spacing-per-fontSize ratio matching DrawText()/MeasureText()'s own
    // internal default (fontSize/10) - added purely so drawHud()'s and
    // drawMissionControlPanel()'s per-line loops stay simple find/replace
    // changes (DrawText(...) -> drawText(...), MeasureText(...) ->
    // measureTextWidth(...)) with no other logic change. No raylib type
    // appears in either signature, so this adds no new public surface
    // beyond what font_ already requires internally.
    void drawText(const char* text, int x, int y, int fontSize, Color color) const;
    int measureTextWidth(const char* text, int fontSize) const;

    Camera3D camera_;

    // Final Turkish-font polish: the Turkish-capable Unicode font loaded
    // in the constructor (assets/fonts/anonymous_pro_bold.ttf, resolved at
    // runtime relative to this executable's own directory - see the
    // constructor's exeDirectory() helper, CMakeLists.txt's POST_BUILD
    // copy step, and assets/fonts/LICENSE-AnonymousPro.txt) - replaces
    // raylib's built-in
    // default font (Unicode U+0000-U+00FF only) for every string this
    // class draws, so Turkish's four extended-Latin letters (ğ/Ğ, ı, ş/Ş,
    // İ) render correctly instead of as missing glyphs. Falls back to
    // GetFontDefault() if the file cannot be loaded (see the constructor)
    // - Renderer3D never fails to render, only degrades gracefully to the
    // old ASCII-only appearance.
    Font font_;
};

} // namespace robot::visual
