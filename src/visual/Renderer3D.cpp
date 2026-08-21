#include "robot/visual/Renderer3D.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdio>

#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

namespace
{

Vector3 toRaylibVector3(const Vec3& v)
{
    return Vector3{v.x, v.y, v.z};
}

constexpr float kGroundHalfExtent = 10.0F; // matches the grid below (20 slices * 1.0 spacing)
constexpr int kGridSlices = 20;
constexpr float kGridSpacing = 1.0F;

constexpr Color kGroundColor = Color{60, 90, 60, 255};
constexpr Color kObstacleColor = ORANGE;
constexpr Color kObstacleOutlineColor = MAROON;
constexpr Color kBaseColor = Color{80, 140, 220, 255};
constexpr Color kBaseOutlineColor = DARKBLUE;

// Table surface visualization (Phase 13S): a simple raised rectangular
// platform, never a vertical wall - the whole point is that the ground
// plane/grid remains visible beyond its edges, reading as open space
// (conceptually a drop) rather than a barrier. Top surface sits at
// kTableTopY, slightly above the ground plane's own Y 0.0, so the two
// never Z-fight where the table's footprint overlaps the ground.
constexpr Color kTableColor = Color{180, 140, 90, 255};
constexpr Color kTableOutlineColor = Color{110, 80, 40, 255};
constexpr float kTableTopY = 0.02F;
constexpr float kTableThickness = 0.06F;

// Sensor ray colors (Phase 13O): red once the reading is within
// VirtualDistanceSensor::kDetectionDistance (obstacleDetected() true), amber
// for a hit that is merely within sensor range but not yet close enough to
// stop the robot, green when nothing is within range at all - a visually
// clear distinction between "clear", "seen", and "stopping" without
// requiring the HUD text to be read.
constexpr Color kSensorRayDetectedColor = RED;
constexpr Color kSensorRayHitColor = Color{255, 180, 0, 255};
constexpr Color kSensorRayClearColor = LIME;

// Home navigation target-direction guide (Phase 13T) - a simple ground-
// level line from the robot toward the base while HomeNavigator is
// actively steering (Aligning/Driving), distinct from every obstacle-
// sensor ray color above so it reads unambiguously as "where the robot
// is heading," not a sensor reading.
constexpr Color kHomeGuideColor = Color{80, 220, 220, 255};

// HUD readability panel: a semi-transparent dark rectangle behind the
// top-left text so it stays legible against any part of the scene behind
// it, regardless of the ground/sky color at that point.
constexpr int kHudMarginX = 20;
constexpr int kHudMarginY = 20;
constexpr int kHudPadding = 10;
constexpr int kHudLineSpacing = 6;
constexpr Color kHudPanelBackground = Color{0, 0, 0, 150};
constexpr Color kHudTitleColor = RAYWHITE;
constexpr Color kHudTextColor = RAYWHITE;
constexpr Color kHudControlsColor = LIGHTGRAY;

// Mission Control panel (Phase 13U): top-right, deliberately separate
// from the engineering HUD panel's top-left position above - the two
// never overlap regardless of HudMode/window size, since each is sized
// and placed independently.
constexpr int kMissionControlMarginRight = 20;
constexpr int kMissionControlMarginY = 20;
constexpr Color kMissionControlTaskColor = Color{255, 220, 100, 255};

struct HudLine
{
    const char* text;
    int fontSize;
    Color color;
};

} // namespace

Renderer3D::Renderer3D()
    : camera_{}
{
    camera_.position = Vector3{8.0F, 8.0F, 8.0F};
    camera_.target = Vector3{0.0F, 0.0F, 0.0F};
    camera_.up = Vector3{0.0F, 1.0F, 0.0F};
    camera_.fovy = 45.0F;
    camera_.projection = CAMERA_PERSPECTIVE;
}

void Renderer3D::renderFrame(const VirtualWorld& world, bool updateCamera, const VisualTelemetry& telemetry)
{
    if (updateCamera)
    {
        UpdateCamera(&camera_, CAMERA_FREE);
    }

    BeginDrawing();
    ClearBackground(RAYWHITE);

    BeginMode3D(camera_);
    drawScene(world, telemetry);
    EndMode3D();

    drawHud(world, telemetry);
    drawMissionControlPanel(telemetry);

    EndDrawing();
}

void Renderer3D::drawScene(const VirtualWorld& world, const VisualTelemetry& telemetry) const
{
    DrawPlane(Vector3{0.0F, 0.0F, 0.0F}, Vector2{kGroundHalfExtent * 2.0F, kGroundHalfExtent * 2.0F}, kGroundColor);
    DrawGrid(kGridSlices, kGridSpacing);

    // Table surface (Phase 13S) - Renderer3D only ever reads
    // world.tableSurface()'s already-computed rectangle; it never decides
    // safety behavior itself (that is VirtualCliffSensor/
    // TableEdgeSafetyController's job, entirely outside this class).
    const TableSurface& table = world.tableSurface();
    const float tableCenterX = (table.minX + table.maxX) / 2.0F;
    const float tableCenterZ = (table.minZ + table.maxZ) / 2.0F;
    const float tableWidth = table.maxX - table.minX;
    const float tableDepth = table.maxZ - table.minZ;
    const Vector3 tableCenter{tableCenterX, kTableTopY - (kTableThickness / 2.0F), tableCenterZ};
    DrawCube(tableCenter, tableWidth, kTableThickness, tableDepth, kTableColor);
    DrawCubeWires(tableCenter, tableWidth, kTableThickness, tableDepth, kTableOutlineColor);

    for (const BoxObstacle& obstacle : world.obstacles())
    {
        if (!obstacle.enabled)
        {
            continue;
        }
        const Vector3 position = toRaylibVector3(obstacle.position);
        DrawCube(position, obstacle.size.x, obstacle.size.y, obstacle.size.z, kObstacleColor);
        DrawCubeWires(position, obstacle.size.x, obstacle.size.y, obstacle.size.z, kObstacleOutlineColor);
    }

    const BasePlatform& base = world.basePlatform();
    const Vector3 basePosition = toRaylibVector3(base.position);
    DrawCube(basePosition, base.size.x, base.size.y, base.size.z, kBaseColor);
    DrawCubeWires(basePosition, base.size.x, base.size.y, base.size.z, kBaseOutlineColor);

    drawVisualRobot(world.robotPose());

    // Three perception rays - left/center/right (manual-validation
    // bugfix; center is unchanged from Phase 13O). Origin/distance/
    // detected for each come straight from `telemetry`, which main3d
    // fills from the exact same VirtualObstacleSensorArray/
    // VirtualDistanceSensor instances driving
    // VirtualRobotHardware::obstacleDetected() - never recomputed here,
    // so the drawn rays always match the actual sensor readings exactly.
    // Renderer3D only ever draws already-computed geometry/state; it
    // never decides detection itself.
    const auto drawObstacleRay = [&](const Vec3& origin, const std::optional<float>& distance, bool detected) {
        const float rayLength = distance.value_or(telemetry.sensorMaximumRange);
        const Vector3 rayStart = toRaylibVector3(origin);
        const Vector3 rayEnd = Vector3{origin.x + (telemetry.sensorDirection.x * rayLength), origin.y,
                                        origin.z + (telemetry.sensorDirection.z * rayLength)};
        const Color rayColor =
            detected ? kSensorRayDetectedColor : (distance.has_value() ? kSensorRayHitColor : kSensorRayClearColor);
        DrawLine3D(rayStart, rayEnd, rayColor);
    };

    drawObstacleRay(telemetry.obstacleRayLeftOrigin, telemetry.obstacleRayLeftDistance,
                     telemetry.obstacleRayLeftDetected);
    drawObstacleRay(telemetry.sensorOrigin, telemetry.obstacleDistance, telemetry.obstacleDetected);
    drawObstacleRay(telemetry.obstacleRayRightOrigin, telemetry.obstacleRayRightDistance,
                     telemetry.obstacleRayRightDetected);

    // Home navigation target-direction guide (Phase 13T) - purely a
    // visualization of main3d.cpp's already-computed
    // homeNavigationGuideVisible flag; this class never decides whether
    // navigation is active.
    if (telemetry.homeNavigationGuideVisible)
    {
        Vector3 guideEnd = toRaylibVector3(world.basePlatform().position);
        guideEnd.y = world.robotPose().position.y;
        DrawLine3D(toRaylibVector3(world.robotPose().position), guideEnd, kHomeGuideColor);
    }
}

void Renderer3D::drawHud(const VirtualWorld& world, const VisualTelemetry& telemetry) const
{
    const RobotPose& pose = world.robotPose();

    char stateLine[80];
    std::snprintf(stateLine, sizeof(stateLine), "State: %.*s", static_cast<int>(telemetry.stateText.size()),
                   telemetry.stateText.data());

    char commandLine[80];
    std::snprintf(commandLine, sizeof(commandLine), "Command: %.*s", static_cast<int>(telemetry.commandText.size()),
                   telemetry.commandText.data());

    char positionLine[64];
    std::snprintf(positionLine, sizeof(positionLine), "Position: X %.2f  Z %.2f", pose.position.x, pose.position.z);

    char headingLine[64];
    std::snprintf(headingLine, sizeof(headingLine), "Heading: %.1f deg", pose.headingDegrees);

    char obstaclesLine[64];
    std::snprintf(obstaclesLine, sizeof(obstaclesLine), "Obstacles: %d", static_cast<int>(world.obstacles().size()));

    // Manual-validation bugfix: one line per perception ray - "CLEAR"
    // when that ray has not detected (whether or not it merely has a
    // distant hit), or its distance when it has - plus the width-aware
    // corridor hazard signal, replacing the old single "Obstacle
    // distance"/"Obstacle detected" pair (which only ever reflected the
    // center ray alone).
    char obstacleLeftLine[64];
    if (telemetry.obstacleRayLeftDetected)
    {
        std::snprintf(obstacleLeftLine, sizeof(obstacleLeftLine), "Obstacle L: %.2f", *telemetry.obstacleRayLeftDistance);
    }
    else
    {
        std::snprintf(obstacleLeftLine, sizeof(obstacleLeftLine), "Obstacle L: CLEAR");
    }

    char obstacleCenterLine[64];
    if (telemetry.obstacleDetected)
    {
        std::snprintf(obstacleCenterLine, sizeof(obstacleCenterLine), "Obstacle C: %.2f", *telemetry.obstacleDistance);
    }
    else
    {
        std::snprintf(obstacleCenterLine, sizeof(obstacleCenterLine), "Obstacle C: CLEAR");
    }

    char obstacleRightLine[64];
    if (telemetry.obstacleRayRightDetected)
    {
        std::snprintf(obstacleRightLine, sizeof(obstacleRightLine), "Obstacle R: %.2f",
                       *telemetry.obstacleRayRightDistance);
    }
    else
    {
        std::snprintf(obstacleRightLine, sizeof(obstacleRightLine), "Obstacle R: CLEAR");
    }

    char bodyCorridorObstacleLine[64];
    std::snprintf(bodyCorridorObstacleLine, sizeof(bodyCorridorObstacleLine), "Body corridor: %s",
                   telemetry.bodyCorridorObstacleHazard ? "BLOCKED" : "CLEAR");

    char leftWheelLine[64];
    std::snprintf(leftWheelLine, sizeof(leftWheelLine), "Left wheel:  %.2f", telemetry.leftWheelSpeed);

    char rightWheelLine[64];
    std::snprintf(rightWheelLine, sizeof(rightWheelLine), "Right wheel: %.2f", telemetry.rightWheelSpeed);

    char driveAuthorityLine[64];
    std::snprintf(driveAuthorityLine, sizeof(driveAuthorityLine), "Drive authority: %.*s",
                   static_cast<int>(telemetry.driveAuthorityText.size()), telemetry.driveAuthorityText.data());

    char avoidanceLine[64];
    std::snprintf(avoidanceLine, sizeof(avoidanceLine), "Avoidance: %s", telemetry.avoidanceEnabled ? "ON" : "OFF");

    char avoidanceActiveLine[64];
    std::snprintf(avoidanceActiveLine, sizeof(avoidanceActiveLine), "Avoidance active: %s",
                   telemetry.avoidanceActive ? "YES" : "NO");

    char forwardClearanceLine[64];
    std::snprintf(forwardClearanceLine, sizeof(forwardClearanceLine), "Forward clearance: %s",
                   telemetry.forwardClearanceClear ? "CLEAR" : "BLOCKED");

    char clearanceLookaheadLine[64];
    std::snprintf(clearanceLookaheadLine, sizeof(clearanceLookaheadLine), "Clearance lookahead: %.2f",
                   telemetry.clearanceLookahead);

    char cliffFrontLeftLine[64];
    std::snprintf(cliffFrontLeftLine, sizeof(cliffFrontLeftLine), "Cliff FL: %s",
                   telemetry.cliffFrontLeft ? "EDGE" : "SAFE");

    char cliffFrontRightLine[64];
    std::snprintf(cliffFrontRightLine, sizeof(cliffFrontRightLine), "Cliff FR: %s",
                   telemetry.cliffFrontRight ? "EDGE" : "SAFE");

    char cliffRearLeftLine[64];
    std::snprintf(cliffRearLeftLine, sizeof(cliffRearLeftLine), "Cliff RL: %s",
                   telemetry.cliffRearLeft ? "EDGE" : "SAFE");

    char cliffRearRightLine[64];
    std::snprintf(cliffRearRightLine, sizeof(cliffRearRightLine), "Cliff RR: %s",
                   telemetry.cliffRearRight ? "EDGE" : "SAFE");

    char edgeSafetyLine[64];
    std::snprintf(edgeSafetyLine, sizeof(edgeSafetyLine), "Edge safety: %s",
                   telemetry.edgeSafetyActive ? "ACTIVE" : "INACTIVE");

    char edgeRecoveryStateLine[96];
    std::snprintf(edgeRecoveryStateLine, sizeof(edgeRecoveryStateLine), "Edge recovery state: %.*s",
                   static_cast<int>(telemetry.edgeRecoveryStateText.size()), telemetry.edgeRecoveryStateText.data());

    char edgeTargetHeadingLine[64];
    std::snprintf(edgeTargetHeadingLine, sizeof(edgeTargetHeadingLine), "Edge target heading: %.1f",
                   telemetry.edgeTargetHeadingDegrees);

    char edgeHeadingErrorLine[64];
    std::snprintf(edgeHeadingErrorLine, sizeof(edgeHeadingErrorLine), "Edge heading error: %.1f",
                   telemetry.edgeHeadingErrorDegrees);

    char collisionLine[64];
    std::snprintf(collisionLine, sizeof(collisionLine), "Collision: %s", telemetry.collidedLastUpdate ? "YES" : "NO");

    char returnReasonLine[64];
    std::snprintf(returnReasonLine, sizeof(returnReasonLine), "Return reason: %.*s",
                   static_cast<int>(telemetry.returnHomeReasonText.size()), telemetry.returnHomeReasonText.data());

    char homeNavStateLine[80];
    std::snprintf(homeNavStateLine, sizeof(homeNavStateLine), "Home nav: %.*s",
                   static_cast<int>(telemetry.homeNavigationStateText.size()), telemetry.homeNavigationStateText.data());

    char homeNavDistanceLine[64];
    std::snprintf(homeNavDistanceLine, sizeof(homeNavDistanceLine), "Home distance: %.2f",
                   telemetry.homeNavigationDistance);

    char homeNavTargetHeadingLine[64];
    std::snprintf(homeNavTargetHeadingLine, sizeof(homeNavTargetHeadingLine), "Home target heading: %.1f",
                   telemetry.homeNavigationTargetHeadingDegrees);

    char homeNavHeadingErrorLine[64];
    std::snprintf(homeNavHeadingErrorLine, sizeof(homeNavHeadingErrorLine), "Home heading error: %.1f",
                   telemetry.homeNavigationHeadingErrorDegrees);

    // --- Compact-mode lines (UX polish) ---
    //
    // A small, high-value operational subset - never hides an active
    // safety condition (section 6 of the brief): Authority reuses
    // driveAuthorityText verbatim, so it reads "SAFETY" the instant
    // DriveAuthority::Safety is active, exactly like Full mode does.
    char compactSafetyLine[32];
    std::snprintf(compactSafetyLine, sizeof(compactSafetyLine), "Safety: %s",
                   telemetry.edgeSafetyActive ? "ACTIVE" : "SAFE");

    char compactAvoidanceLine[32];
    if (!telemetry.avoidanceEnabled)
    {
        std::snprintf(compactAvoidanceLine, sizeof(compactAvoidanceLine), "Avoidance: OFF");
    }
    else if (telemetry.avoidanceActive)
    {
        std::snprintf(compactAvoidanceLine, sizeof(compactAvoidanceLine), "Avoidance: AVOIDING");
    }
    else
    {
        std::snprintf(compactAvoidanceLine, sizeof(compactAvoidanceLine), "Avoidance: IDLE");
    }

    char compactObstacleLine[32];
    std::snprintf(compactObstacleLine, sizeof(compactObstacleLine), "Obstacle: %s",
                   telemetry.obstacleHazard ? "DETECTED" : "CLEAR");

    // Selects display text from four already-computed booleans - pure
    // presentation (which line to show), never a cliff/edge-safety
    // decision of its own (that remains entirely
    // VirtualCliffSensor/TableEdgeSafetyController's job).
    const bool anyCliffEdge =
        telemetry.cliffFrontLeft || telemetry.cliffFrontRight || telemetry.cliffRearLeft || telemetry.cliffRearRight;
    char compactEdgeLine[32];
    if (telemetry.edgeSafetyActive)
    {
        std::snprintf(compactEdgeLine, sizeof(compactEdgeLine), "Edge: RECOVERING");
    }
    else if (anyCliffEdge)
    {
        std::snprintf(compactEdgeLine, sizeof(compactEdgeLine), "Edge: EDGE");
    }
    else
    {
        std::snprintf(compactEdgeLine, sizeof(compactEdgeLine), "Edge: SAFE");
    }

    // Phase 13T: one concise Home nav line - "-" while Inactive (the
    // common case outside a Return Home mission), otherwise state plus
    // remaining distance. Compact mode never expands into an engineering
    // panel - no target heading/heading error here (see fullLines below
    // for those).
    char compactHomeLine[32];
    if (telemetry.homeNavigationStateText == "Inactive")
    {
        std::snprintf(compactHomeLine, sizeof(compactHomeLine), "Home: -");
    }
    else
    {
        std::snprintf(compactHomeLine, sizeof(compactHomeLine), "Home: %.*s %.1fm",
                       static_cast<int>(telemetry.homeNavigationStateText.size()),
                       telemetry.homeNavigationStateText.data(), telemetry.homeNavigationDistance);
    }

    // A small table of {text, fontSize, color} rather than hand-tracked Y
    // offsets per line - adding/removing a HUD line only ever touches this
    // array, and panel sizing/text drawing below stay generic. Which
    // array is used - and therefore the panel's size - depends entirely
    // on telemetry.hudMode; this is the only behavioral difference
    // between Full and Compact.
    const HudLine fullLines[] = {
        {"Robot Simulator 3D", 20, kHudTitleColor},
        {stateLine, 18, kHudTextColor},
        {commandLine, 18, kHudTextColor},
        {driveAuthorityLine, 18, kHudTextColor},
        {avoidanceLine, 18, kHudTextColor},
        {avoidanceActiveLine, 18, kHudTextColor},
        {forwardClearanceLine, 18, kHudTextColor},
        {clearanceLookaheadLine, 18, kHudTextColor},
        {positionLine, 18, kHudTextColor},
        {headingLine, 18, kHudTextColor},
        {obstaclesLine, 18, kHudTextColor},
        {obstacleLeftLine, 18, kHudTextColor},
        {obstacleCenterLine, 18, kHudTextColor},
        {obstacleRightLine, 18, kHudTextColor},
        {bodyCorridorObstacleLine, 18, kHudTextColor},
        {cliffFrontLeftLine, 18, kHudTextColor},
        {cliffFrontRightLine, 18, kHudTextColor},
        {cliffRearLeftLine, 18, kHudTextColor},
        {cliffRearRightLine, 18, kHudTextColor},
        {edgeSafetyLine, 18, kHudTextColor},
        {edgeRecoveryStateLine, 18, kHudTextColor},
        {edgeTargetHeadingLine, 18, kHudTextColor},
        {edgeHeadingErrorLine, 18, kHudTextColor},
        {leftWheelLine, 18, kHudTextColor},
        {rightWheelLine, 18, kHudTextColor},
        {collisionLine, 18, kHudTextColor},
        {homeNavStateLine, 18, kHudTextColor},
        {homeNavDistanceLine, 18, kHudTextColor},
        {homeNavTargetHeadingLine, 18, kHudTextColor},
        {homeNavHeadingErrorLine, 18, kHudTextColor},
        {returnReasonLine, 18, kHudTextColor},
        {"TAB: capture/release mouse   F11: fullscreen/windowed   SPACE: pause   O: toggle obstacle   Mouse/WASD: camera",
         16, kHudControlsColor},
        {"M: manual drive mode   Arrows: manual forward/reverse/turn   X: stop manual wheels", 16, kHudControlsColor},
        {"A: toggle autonomous obstacle avoidance   R: return home   H: Compact HUD", 16, kHudControlsColor},
    };

    const HudLine compactLines[] = {
        {"Robot Simulator 3D", 20, kHudTitleColor},
        {stateLine, 18, kHudTextColor},
        {driveAuthorityLine, 18, kHudTextColor},
        {compactSafetyLine, 18, kHudTextColor},
        {compactAvoidanceLine, 18, kHudTextColor},
        {compactObstacleLine, 18, kHudTextColor},
        {compactEdgeLine, 18, kHudTextColor},
        {compactHomeLine, 18, kHudTextColor},
        {"R: return home   H: Expand HUD", 16, kHudControlsColor},
    };

    const bool compact = telemetry.hudMode == HudMode::Compact;
    const HudLine* lines = compact ? compactLines : fullLines;
    const std::size_t lineCount = compact ? (sizeof(compactLines) / sizeof(compactLines[0]))
                                           : (sizeof(fullLines) / sizeof(fullLines[0]));

    // Panel sized to fully contain the widest line so contrast holds
    // regardless of content length - a fixed guessed width could leave a
    // line's tail spilling back onto the unshaded scene. Compact mode's
    // much shorter array naturally yields a much smaller panel here, with
    // no separate size-mode logic needed.
    int panelWidth = 0;
    int contentHeight = 0;
    for (std::size_t i = 0; i < lineCount; ++i)
    {
        panelWidth = std::max(panelWidth, MeasureText(lines[i].text, lines[i].fontSize));
        contentHeight += lines[i].fontSize + kHudLineSpacing;
    }
    panelWidth += 2 * kHudPadding;
    const int panelHeight = contentHeight + (2 * kHudPadding) - kHudLineSpacing;

    DrawRectangle(kHudMarginX, kHudMarginY, panelWidth, panelHeight, kHudPanelBackground);

    const int textX = kHudMarginX + kHudPadding;
    int textY = kHudMarginY + kHudPadding;
    for (std::size_t i = 0; i < lineCount; ++i)
    {
        DrawText(lines[i].text, textX, textY, lines[i].fontSize, lines[i].color);
        textY += lines[i].fontSize + kHudLineSpacing;
    }

    DrawFPS(10, GetScreenHeight() - 30);
}

void Renderer3D::drawMissionControlPanel(const VisualTelemetry& telemetry) const
{
    char taskLine[48];
    std::snprintf(taskLine, sizeof(taskLine), "Task: %.*s", static_cast<int>(telemetry.missionTaskText.size()),
                   telemetry.missionTaskText.data());

    char homeZoneLine[48];
    std::snprintf(homeZoneLine, sizeof(homeZoneLine), "Home zone: %s", telemetry.homeZoneInside ? "INSIDE" : "OUTSIDE");

    char baseDistanceLine[48];
    std::snprintf(baseDistanceLine, sizeof(baseDistanceLine), "Base distance: %.2f", telemetry.baseDistance);

    const HudLine lines[] = {
        {"MISSION CONTROL", 20, kHudTitleColor},
        {taskLine, 18, kMissionControlTaskColor},
        {"1  Start Roam", 16, kHudControlsColor},
        {"2  Return Home", 16, kHudControlsColor},
        {"3  Stop Task", 16, kHudControlsColor},
        {"R  Return Home", 16, kHudControlsColor},
        {homeZoneLine, 16, kHudTextColor},
        {baseDistanceLine, 16, kHudTextColor},
    };
    constexpr std::size_t lineCount = sizeof(lines) / sizeof(lines[0]);

    int panelWidth = 0;
    int contentHeight = 0;
    for (std::size_t i = 0; i < lineCount; ++i)
    {
        panelWidth = std::max(panelWidth, MeasureText(lines[i].text, lines[i].fontSize));
        contentHeight += lines[i].fontSize + kHudLineSpacing;
    }
    panelWidth += 2 * kHudPadding;
    const int panelHeight = contentHeight + (2 * kHudPadding) - kHudLineSpacing;

    const int panelX = GetScreenWidth() - kMissionControlMarginRight - panelWidth;
    const int panelY = kMissionControlMarginY;
    DrawRectangle(panelX, panelY, panelWidth, panelHeight, kHudPanelBackground);

    const int textX = panelX + kHudPadding;
    int textY = panelY + kHudPadding;
    for (std::size_t i = 0; i < lineCount; ++i)
    {
        DrawText(lines[i].text, textX, textY, lines[i].fontSize, lines[i].color);
        textY += lines[i].fontSize + kHudLineSpacing;
    }
}

} // namespace robot::visual
