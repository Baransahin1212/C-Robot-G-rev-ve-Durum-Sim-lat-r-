#include "robot/visual/VisualRobot.hpp"

#include "raylib.h"
#include "rlgl.h"

#include "robot/visual/DockChargingContacts.hpp"

namespace robot::visual
{

namespace
{

constexpr Color kBodyColor = Color{50, 120, 200, 255};
constexpr Color kBodyOutlineColor = DARKBLUE;
constexpr Color kWheelColor = Color{40, 40, 40, 255};
constexpr Color kMarkerColor = RED;

// Phase 13Y: darker metallic sockets, deliberately distinct from the
// dock's own gold/brass pins (Renderer3D.cpp's kDockContactColor) - the
// user should visually read these two colors as "plug" vs "socket," never
// as identical/interchangeable parts.
constexpr Color kRearContactColor = Color{70, 74, 82, 255};
constexpr Color kRearContactSurroundColor = Color{30, 30, 34, 255};

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

    // Phase 13Y: two rear charging receiver pads - on the REAR (-Z local)
    // face, never the front, Left/Right-symmetric at the exact same
    // kContactPairSpacingWorldUnits the dock's own pins use
    // (DockChargingContacts.hpp - never a separately-eyeballed spacing),
    // so the two visually/physically line up when DockApproachController
    // reports Docked. `localContactY` converts the shared ABSOLUTE-world
    // kContactHeightWorldUnits both dock pins and robot receivers are
    // defined at (see that constant's own docs) into this draw call's own
    // LOCAL space, since drawVisualRobot() draws everything local-to-the-
    // robot before the model matrix places/orients it in the world - the
    // one place this conversion is needed, since
    // computeRobotRearChargingContacts() (used by DockApproachController's
    // own world-space contact-alignment math) already works in world
    // space directly. A small darker recessed surround (never the dock
    // pins' own gold/brass color) makes clear these are sockets, not
    // plugs.
    const float localContactY = kContactHeightWorldUnits - pose.position.y;
    const float halfContactSpacing = kContactPairSpacingWorldUnits / 2.0F;
    const float rearZ = -(kBodyLength / 2.0F);
    for (const float lateralX : {-halfContactSpacing, halfContactSpacing})
    {
        const Vector3 surroundCenter{lateralX, localContactY, rearZ};
        DrawCube(surroundCenter, kContactRadiusWorldUnits * 2.6F, kContactRadiusWorldUnits * 1.4F,
                  kContactRadiusWorldUnits * 1.2F, kRearContactSurroundColor);
        // Phase 13Y final docking visual-precision polish: drawn at EXACTLY
        // `rearZ` (the same local Z the surround above uses, and the same
        // point computeRobotRearChargingContacts() itself derives - see
        // that function's own docs) - a real, audited discrepancy this fix
        // closes: this sphere previously carried its own extra
        // `-kContactRadiusWorldUnits*0.3F` visual-only offset with no
        // logical counterpart, so what the user saw was never quite what
        // DockApproachController's own alignment math was actually
        // checking. Never a separately-tuned visual position again.
        const Vector3 contactCenter{lateralX, localContactY, rearZ};
        DrawSphere(contactCenter, kContactRadiusWorldUnits, kRearContactColor);
    }

    rlPopMatrix();
}

} // namespace robot::visual
