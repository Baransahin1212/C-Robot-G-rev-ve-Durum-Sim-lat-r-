#include "robot/visual/VisualRobot.hpp"

#include "raylib.h"
#include "rlgl.h"

namespace robot::visual
{

namespace
{

constexpr Color kBodyColor = Color{50, 120, 200, 255};
constexpr Color kBodyOutlineColor = DARKBLUE;
constexpr Color kWheelColor = Color{40, 40, 40, 255};
constexpr Color kMarkerColor = RED;

} // namespace

void drawVisualRobot(const RobotPose& pose)
{
    using namespace RobotDimensions;

    // Draw everything in the robot's own local space (origin at its
    // center, +Z forward), then let the model matrix place/orient it in
    // the world - so heading changes never need per-primitive math.
    rlPushMatrix();
    rlTranslatef(pose.position.x, pose.position.y, pose.position.z);
    rlRotatef(pose.headingDegrees, 0.0F, 1.0F, 0.0F);

    const Vector3 origin{0.0F, 0.0F, 0.0F};
    DrawCube(origin, kBodyWidth, kBodyHeight, kBodyLength, kBodyColor);
    DrawCubeWires(origin, kBodyWidth, kBodyHeight, kBodyLength, kBodyOutlineColor);

    // Left/right wheels, mounted at the body's sides, drawn as a short
    // cylinder extruded along the local X axis (its rolling/rotation
    // axis) via start/end points rather than raylib's default
    // Y-extruded DrawCylinder, which would draw a wheel standing on its
    // edge instead of lying flat against the body.
    const float wheelInnerX = kBodyWidth / 2.0F;
    const float wheelOuterX = wheelInnerX + kWheelThickness;
    DrawCylinderEx(Vector3{-wheelInnerX, 0.0F, 0.0F}, Vector3{-wheelOuterX, 0.0F, 0.0F}, kWheelRadius, kWheelRadius,
                   16, kWheelColor);
    DrawCylinderEx(Vector3{wheelInnerX, 0.0F, 0.0F}, Vector3{wheelOuterX, 0.0F, 0.0F}, kWheelRadius, kWheelRadius, 16,
                   kWheelColor);

    // Small front heading marker at the +Z (forward) edge of the body, so
    // heading is visually obvious even for a stationary robot.
    const Vector3 markerCenter{0.0F, kBodyHeight / 2.0F, kBodyLength / 2.0F};
    DrawCube(markerCenter, kBodyWidth * 0.2F, kBodyHeight * 0.4F, kBodyWidth * 0.2F, kMarkerColor);

    rlPopMatrix();
}

} // namespace robot::visual
