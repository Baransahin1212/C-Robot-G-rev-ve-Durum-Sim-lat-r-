#include "robot/visual/Renderer3D.hpp"

#include <algorithm>
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

// Sensor ray colors (Phase 13O): red once the reading is within
// VirtualDistanceSensor::kDetectionDistance (obstacleDetected() true), amber
// for a hit that is merely within sensor range but not yet close enough to
// stop the robot, green when nothing is within range at all - a visually
// clear distinction between "clear", "seen", and "stopping" without
// requiring the HUD text to be read.
constexpr Color kSensorRayDetectedColor = RED;
constexpr Color kSensorRayHitColor = Color{255, 180, 0, 255};
constexpr Color kSensorRayClearColor = LIME;

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

    EndDrawing();
}

void Renderer3D::drawScene(const VirtualWorld& world, const VisualTelemetry& telemetry) const
{
    DrawPlane(Vector3{0.0F, 0.0F, 0.0F}, Vector2{kGroundHalfExtent * 2.0F, kGroundHalfExtent * 2.0F}, kGroundColor);
    DrawGrid(kGridSlices, kGridSpacing);

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

    // Sensor ray (Phase 13O): origin/direction/length come straight from
    // `telemetry`, which main3d fills from the same VirtualDistanceSensor
    // instance driving obstacleDetected()/obstacleDistance() - never
    // recomputed here, so the drawn ray always matches the actual sensor
    // reading exactly.
    const float rayLength = telemetry.obstacleDistance.value_or(telemetry.sensorMaximumRange);
    const Vector3 rayStart = toRaylibVector3(telemetry.sensorOrigin);
    const Vector3 rayEnd = Vector3{telemetry.sensorOrigin.x + (telemetry.sensorDirection.x * rayLength),
                                    telemetry.sensorOrigin.y + (telemetry.sensorDirection.y * rayLength),
                                    telemetry.sensorOrigin.z + (telemetry.sensorDirection.z * rayLength)};
    const Color rayColor = telemetry.obstacleDetected
                                ? kSensorRayDetectedColor
                                : (telemetry.obstacleDistance.has_value() ? kSensorRayHitColor : kSensorRayClearColor);
    DrawLine3D(rayStart, rayEnd, rayColor);
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

    char obstacleDistanceLine[64];
    if (telemetry.obstacleDistance.has_value())
    {
        std::snprintf(obstacleDistanceLine, sizeof(obstacleDistanceLine), "Obstacle distance: %.2f",
                       *telemetry.obstacleDistance);
    }
    else
    {
        std::snprintf(obstacleDistanceLine, sizeof(obstacleDistanceLine), "Obstacle distance: No hit");
    }

    char obstacleDetectedLine[64];
    std::snprintf(obstacleDetectedLine, sizeof(obstacleDetectedLine), "Obstacle detected: %s",
                   telemetry.obstacleDetected ? "YES" : "NO");

    char leftWheelLine[64];
    std::snprintf(leftWheelLine, sizeof(leftWheelLine), "Left wheel:  %.2f", telemetry.leftWheelSpeed);

    char rightWheelLine[64];
    std::snprintf(rightWheelLine, sizeof(rightWheelLine), "Right wheel: %.2f", telemetry.rightWheelSpeed);

    char driveModeLine[64];
    std::snprintf(driveModeLine, sizeof(driveModeLine), "Drive mode: %.*s",
                   static_cast<int>(telemetry.driveModeText.size()), telemetry.driveModeText.data());

    char collisionLine[64];
    std::snprintf(collisionLine, sizeof(collisionLine), "Collision: %s", telemetry.collidedLastUpdate ? "YES" : "NO");

    // A small table of {text, fontSize, color} rather than hand-tracked Y
    // offsets per line - adding/removing a HUD line only ever touches this
    // array, and panel sizing/text drawing below stay generic.
    const HudLine lines[] = {
        {"Robot Simulator 3D", 20, kHudTitleColor},
        {stateLine, 18, kHudTextColor},
        {commandLine, 18, kHudTextColor},
        {positionLine, 18, kHudTextColor},
        {headingLine, 18, kHudTextColor},
        {obstaclesLine, 18, kHudTextColor},
        {obstacleDistanceLine, 18, kHudTextColor},
        {obstacleDetectedLine, 18, kHudTextColor},
        {leftWheelLine, 18, kHudTextColor},
        {rightWheelLine, 18, kHudTextColor},
        {driveModeLine, 18, kHudTextColor},
        {collisionLine, 18, kHudTextColor},
        {"TAB: capture/release mouse   F11: fullscreen/windowed   SPACE: pause   O: toggle obstacle   Mouse/WASD: camera",
         16, kHudControlsColor},
        {"M: manual drive mode   Arrows: manual forward/reverse/turn   X: stop manual wheels", 16, kHudControlsColor},
    };

    // Panel sized to fully contain the widest line so contrast holds
    // regardless of content length - a fixed guessed width could leave a
    // line's tail spilling back onto the unshaded scene.
    int panelWidth = 0;
    int contentHeight = 0;
    for (const HudLine& line : lines)
    {
        panelWidth = std::max(panelWidth, MeasureText(line.text, line.fontSize));
        contentHeight += line.fontSize + kHudLineSpacing;
    }
    panelWidth += 2 * kHudPadding;
    const int panelHeight = contentHeight + (2 * kHudPadding) - kHudLineSpacing;

    DrawRectangle(kHudMarginX, kHudMarginY, panelWidth, panelHeight, kHudPanelBackground);

    const int textX = kHudMarginX + kHudPadding;
    int textY = kHudMarginY + kHudPadding;
    for (const HudLine& line : lines)
    {
        DrawText(line.text, textX, textY, line.fontSize, line.color);
        textY += line.fontSize + kHudLineSpacing;
    }

    DrawFPS(10, GetScreenHeight() - 30);
}

} // namespace robot::visual
