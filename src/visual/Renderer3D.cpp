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

void Renderer3D::renderFrame(const VirtualWorld& world, bool updateCamera, std::string_view stateText,
                              std::string_view commandText)
{
    if (updateCamera)
    {
        UpdateCamera(&camera_, CAMERA_FREE);
    }

    BeginDrawing();
    ClearBackground(RAYWHITE);

    BeginMode3D(camera_);
    drawScene(world);
    EndMode3D();

    drawHud(world, stateText, commandText);

    EndDrawing();
}

void Renderer3D::drawScene(const VirtualWorld& world) const
{
    DrawPlane(Vector3{0.0F, 0.0F, 0.0F}, Vector2{kGroundHalfExtent * 2.0F, kGroundHalfExtent * 2.0F}, kGroundColor);
    DrawGrid(kGridSlices, kGridSpacing);

    for (const BoxObstacle& obstacle : world.obstacles())
    {
        const Vector3 position = toRaylibVector3(obstacle.position);
        DrawCube(position, obstacle.size.x, obstacle.size.y, obstacle.size.z, kObstacleColor);
        DrawCubeWires(position, obstacle.size.x, obstacle.size.y, obstacle.size.z, kObstacleOutlineColor);
    }

    const BasePlatform& base = world.basePlatform();
    const Vector3 basePosition = toRaylibVector3(base.position);
    DrawCube(basePosition, base.size.x, base.size.y, base.size.z, kBaseColor);
    DrawCubeWires(basePosition, base.size.x, base.size.y, base.size.z, kBaseOutlineColor);

    drawVisualRobot(world.robotPose());
}

void Renderer3D::drawHud(const VirtualWorld& world, std::string_view stateText, std::string_view commandText) const
{
    const RobotPose& pose = world.robotPose();

    char stateLine[80];
    std::snprintf(stateLine, sizeof(stateLine), "State: %.*s", static_cast<int>(stateText.size()), stateText.data());

    char commandLine[80];
    std::snprintf(commandLine, sizeof(commandLine), "Command: %.*s", static_cast<int>(commandText.size()),
                   commandText.data());

    char positionLine[64];
    std::snprintf(positionLine, sizeof(positionLine), "Position: X %.2f  Z %.2f", pose.position.x, pose.position.z);

    char headingLine[64];
    std::snprintf(headingLine, sizeof(headingLine), "Heading: %.1f deg", pose.headingDegrees);

    char obstaclesLine[64];
    std::snprintf(obstaclesLine, sizeof(obstaclesLine), "Obstacles: %d", static_cast<int>(world.obstacles().size()));

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
        {"TAB: capture/release mouse   F11: fullscreen/windowed   Mouse/WASD: camera", 16, kHudControlsColor},
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
