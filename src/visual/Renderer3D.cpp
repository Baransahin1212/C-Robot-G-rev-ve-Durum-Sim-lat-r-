#include "robot/visual/Renderer3D.hpp"

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
constexpr Color kHudPanelBackground = Color{0, 0, 0, 150};
constexpr Color kHudTitleColor = RAYWHITE;
constexpr Color kHudTextColor = RAYWHITE;
constexpr Color kHudControlsColor = LIGHTGRAY;

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

void Renderer3D::renderFrame(const VirtualWorld& world, bool updateCamera)
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

    drawHud(world);

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

void Renderer3D::drawHud(const VirtualWorld& world) const
{
    const RobotPose& pose = world.robotPose();

    char positionLine[64];
    std::snprintf(positionLine, sizeof(positionLine), "Position: X %.2f  Z %.2f", pose.position.x, pose.position.z);

    char headingLine[64];
    std::snprintf(headingLine, sizeof(headingLine), "Heading: %.1f deg", pose.headingDegrees);

    char obstaclesLine[64];
    std::snprintf(obstaclesLine, sizeof(obstaclesLine), "Obstacles: %d", static_cast<int>(world.obstacles().size()));

    const char* controlsLine = "TAB: capture/release mouse   F11: fullscreen/windowed   Mouse/WASD: camera";
    constexpr int kControlsFontSize = 16;

    // Panel sized to fully contain the widest line (the controls line) so
    // contrast holds for every line regardless of its length - a fixed
    // guessed width could leave the tail of the controls line spilling
    // back onto the unshaded scene.
    const int panelWidth = MeasureText(controlsLine, kControlsFontSize) + (2 * kHudPadding);
    constexpr int panelHeight = 96 + kControlsFontSize + (2 * kHudPadding);
    DrawRectangle(kHudMarginX, kHudMarginY, panelWidth, panelHeight, kHudPanelBackground);

    const int textX = kHudMarginX + kHudPadding;
    const int baseY = kHudMarginY + kHudPadding;

    DrawText("Robot Simulator 3D", textX, baseY, 20, kHudTitleColor);
    DrawText(positionLine, textX, baseY + 26, 18, kHudTextColor);
    DrawText(headingLine, textX, baseY + 48, 18, kHudTextColor);
    DrawText(obstaclesLine, textX, baseY + 70, 18, kHudTextColor);
    DrawText(controlsLine, textX, baseY + 96, kControlsFontSize, kHudControlsColor);

    DrawFPS(10, GetScreenHeight() - 30);
}

} // namespace robot::visual
