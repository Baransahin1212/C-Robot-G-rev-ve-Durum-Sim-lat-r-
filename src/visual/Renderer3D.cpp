#include "robot/visual/Renderer3D.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include "robot/visual/DockChargingContacts.hpp"
#include "robot/visual/ExecutableDirectory.hpp"
#include "robot/visual/MapPanelStatus.hpp"
#include "robot/visual/VisualRobot.hpp"

namespace robot::visual
{

namespace
{

Vector3 toRaylibVector3(const Vec3& v)
{
    return Vector3{v.x, v.y, v.z};
}

// Phase 13W human-visual-redesign v2: shrunk from the old 10.0F/20 (sized
// for the old 12x12 square table) so the outside floor/grid no longer
// dwarfs the new, smaller 8x4 desk - still a bit larger than the desk
// itself (context, not a hard wall), but no longer the dominant feature
// of the frame.
constexpr float kGroundHalfExtent = 6.0F; // matches the grid below (12 slices * 1.0 spacing)
constexpr int kGridSlices = 12;
constexpr float kGridSpacing = 1.0F;

// Phase 13W human-visual-redesign v2: darker, lower-contrast than the
// original bright green - human validation flagged the outside
// ground/grid as visually competing with the desk itself. Still present
// (never fully removed - the brief's own "reduce dominance... not
// necessarily remove it completely"), just deliberately recessive now.
constexpr Color kGroundColor = Color{35, 45, 38, 255};

// Phase 13W: desktop-workspace palette - coherent, neutral desk colors
// rather than every object sharing one bright color. Kept dark/neutral
// for the electronics (monitor/keyboard/mouse/lamp/dock), with the mug
// and notebook each carrying one small accent color, matching the
// brief's own "table: wood, monitor: dark gray/black, ... one accent
// color" guidance.
constexpr Color kDeskObjectOutlineColor = Color{20, 20, 22, 255};
constexpr Color kMonitorBodyColor = Color{48, 48, 52, 255};
constexpr Color kMonitorScreenColor = Color{18, 22, 30, 255};
constexpr Color kKeyboardColor = Color{58, 58, 62, 255};
constexpr Color kKeyRowColor = Color{40, 40, 44, 255};
constexpr Color kMouseColor = Color{52, 52, 58, 255};
constexpr Color kMugBodyColor = Color{188, 72, 58, 255};
constexpr Color kMugHandleColor = Color{164, 60, 48, 255};
constexpr Color kNotebookCoverColor = Color{62, 104, 128, 255};
constexpr Color kNotebookPageColor = Color{222, 216, 196, 255};
constexpr Color kLampBaseColor = Color{42, 42, 46, 255};
constexpr Color kLampStemColor = Color{64, 64, 68, 255};
constexpr Color kLampHeadColor = Color{214, 200, 158, 255};

// Charging dock palette (Phase 13W) - dark/neutral housing, matching the
// brief's "dock: dark/black" guidance, with a small brass/gold accent on
// the contact pads so they read as functional detail, not just more dark
// plastic.
constexpr Color kDockPlatformColor = Color{58, 58, 64, 255};
constexpr Color kDockPlatformOutlineColor = Color{30, 30, 34, 255};
constexpr Color kDockHousingColor = Color{34, 34, 38, 255};
constexpr Color kDockGuideArmColor = Color{50, 50, 55, 255};
constexpr Color kDockContactPadColor = Color{196, 168, 64, 255};

// Table surface visualization (Phase 13S): a simple raised rectangular
// platform, never a vertical wall - the whole point is that the ground
// plane/grid remains visible beyond its edges, reading as open space
// (conceptually a drop) rather than a barrier. Top surface sits at
// kTableTopY, slightly above the ground plane's own Y 0.0, so the two
// never Z-fight where the table's footprint overlaps the ground.
constexpr Color kTableColor = Color{180, 140, 90, 255};
constexpr Color kTableOutlineColor = Color{110, 80, 40, 255};
constexpr float kTableTopY = 0.02F;
// Phase 13W human-visual-redesign v2: thickened from 0.06F so the desk
// reads as a solid slab with visible edge thickness from the new elevated
// three-quarter camera angle (brief: "clean rectangular slab, visible
// thickness"), not a paper-thin plane.
constexpr float kTableThickness = 0.15F;

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

// Exploration map panel (Phase 13V): bottom-right, deliberately separate
// from the engineering HUD panel (top-left) and Mission Control panel
// (top-right) above - all three are sized/placed independently and never
// overlap at this project's fixed 1280x720 window size (and stay
// bottom-right-anchored the same way in borderless-fullscreen). Top-down,
// fixed orientation - never rotated by robot heading (Phase 13V brief).
constexpr int kExplorationPanelMarginRight = 20;
constexpr int kExplorationPanelMarginBottom = 20;

// Phase 13W human-visual-redesign v2: the map viewport now preserves the
// desk's own physical aspect ratio (~2:1, width:depth) instead of forcing
// the rectangular 67x33-ish grid into a square drawing area (human
// validation: "the map is visually square instead of matching the desk").
// A fixed WIDTH with height derived from the actual TableSurface aspect
// ratio (never a hardcoded height) keeps this correct if the table's own
// dimensions ever change again.
constexpr int kExplorationGridPixelWidth = 300;
constexpr Color kExplorationUnknownColor = kHudPanelBackground; // "not yet observed" reads as the panel's own background
constexpr Color kExplorationFreeColor = Color{70, 90, 70, 255};
constexpr Color kExplorationOccupiedColor = Color{200, 70, 60, 255};
constexpr Color kExplorationTrailColor = Color{80, 220, 220, 255}; // matches kHomeGuideColor's cyan - "where the robot has been"
// Phase 13X: distinct from kExplorationTrailColor (cyan, "where the robot
// HAS been") - amber/orange reads as "intent/plan," never confusable with
// the travel trail even at this panel's small scale.
constexpr Color kPlannedRouteColor = Color{240, 170, 40, 255};
constexpr Color kExplorationBoundaryColor = LIGHTGRAY;
constexpr Color kExplorationRobotColor = Color{255, 220, 100, 255}; // matches kMissionControlTaskColor
constexpr Color kExplorationHeadingColor = RED;
constexpr Color kExplorationBaseColor = Color{80, 140, 220, 255}; // the map panel's own home/base marker color

struct HudLine
{
    const char* text;
    int fontSize;
    Color color;
};

// Final Turkish-font polish: raylib's own DrawText()/MeasureText() internal
// default spacing (fontSize/10) - replicated here via drawText()/
// measureTextWidth() so switching to a loaded font does not visibly change
// existing letter spacing.
constexpr float kFontSpacingRatio = 0.1F;

// Baked glyph size for the loaded font (see LoadFontEx below) - comfortably
// above the largest size actually drawn (20, the HUD title), since raylib
// bakes each glyph's texture at this size once and DrawTextEx() then scales
// it for other requested sizes; too small would look soft when scaled up.
constexpr int kFontBaseSize = 32;

// The exact codepoint set Renderer3D ever draws: printable ASCII (32-126,
// covers every English/numeric/punctuation character already used) plus
// the full set of Turkish-specific letters (upper/lower) actually used
// across TurkishText.hpp's mappings and this file's own literal HUD text -
// not the font's entire glyph range, kept minimal and explicit per this
// phase's own brief.
std::vector<int> buildFontCodepoints()
{
    std::vector<int> codepoints;
    codepoints.reserve(96 + 12);
    for (int c = 32; c <= 126; ++c)
    {
        codepoints.push_back(c);
    }
    constexpr int kTurkishCodepoints[] = {
        0x00C7, 0x00E7, // Ç ç
        0x011E, 0x011F, // Ğ ğ
        0x0130, 0x0131, // İ ı
        0x00D6, 0x00F6, // Ö ö
        0x015E, 0x015F, // Ş ş
        0x00DC, 0x00FC, // Ü ü
    };
    for (int c : kTurkishCodepoints)
    {
        codepoints.push_back(c);
    }
    return codepoints;
}

} // namespace

Renderer3D::Renderer3D()
    : camera_{}
    , font_{}
{
    // Phase 13W human-visual-redesign v2: elevated three-quarter
    // workstation view, replacing the old distant near-isometric (8,8,8)
    // camera human validation flagged as "too distant... looks like a
    // robotics debug environment, not a computer desk." Derived from
    // VirtualWorld's own table extents (kDeskHalfWidth/kDeskHalfDepth
    // below mirror VirtualWorld.cpp's kTableHalfWidth/kTableHalfDepth -
    // the table is deliberately fixed/deterministic, never runtime-
    // configurable, so this is a documented FORMULA against the current
    // desk size, not a number re-tuned by hand for the old square table)
    // via simple spherical coordinates around the table center: a ~40
    // degree yaw and ~32 degree elevation angle, at a distance
    // proportional to the desk's longer axis - close enough that the
    // rectangular desk fills most of the viewport while the whole
    // tabletop, monitor, keyboard, and dock all stay in frame.
    constexpr float kDeskHalfWidth = 4.0F;
    constexpr float kDeskHalfDepth = 2.0F;
    constexpr float kCameraDistance = std::max(kDeskHalfWidth, kDeskHalfDepth) * 2.0F * 0.85F;
    constexpr float kPi = 3.14159265358979323846F;
    constexpr float kYawDegrees = 40.0F;
    constexpr float kElevationDegrees = 32.0F;
    const float yawRadians = kYawDegrees * (kPi / 180.0F);
    const float elevationRadians = kElevationDegrees * (kPi / 180.0F);
    const float horizontalRadius = kCameraDistance * std::cos(elevationRadians);

    camera_.position = Vector3{horizontalRadius * std::sin(yawRadians), kCameraDistance * std::sin(elevationRadians),
                                horizontalRadius * std::cos(yawRadians)};
    camera_.target = Vector3{0.0F, 0.1F, 0.0F};
    camera_.up = Vector3{0.0F, 1.0F, 0.0F};
    camera_.fovy = 45.0F;
    camera_.projection = CAMERA_PERSPECTIVE;

    // Final font delivery fix: load the repository-local Turkish-capable
    // font (see assets/fonts/LICENSE-AnonymousPro.txt) with exactly the
    // codepoints this class draws, from CMakeLists.txt's POST_BUILD-copied
    // location next to this executable (RobotSimulator3D.exe's own
    // directory + assets/fonts/anonymous_pro_bold.ttf) - never a
    // developer-machine-specific absolute source-tree path. Falls through
    // to raylib's built-in default font (ASCII + Latin-1 only, the
    // pre-this-phase appearance) if the executable directory cannot be
    // resolved or the file cannot be read - Renderer3D must never fail to
    // construct over a missing font asset.
    std::vector<int> codepoints = buildFontCodepoints();
    const std::string exeDir = executableDirectory();
    if (!exeDir.empty())
    {
        const std::string fontPath = exeDir + "\\assets\\fonts\\anonymous_pro_bold.ttf";
        font_ = LoadFontEx(fontPath.c_str(), kFontBaseSize, codepoints.data(), static_cast<int>(codepoints.size()));
    }
    if (font_.texture.id == 0)
    {
        std::fprintf(stderr,
                      "Renderer3D: could not load the Turkish-capable font (expected at "
                      "<executable directory>\\assets\\fonts\\anonymous_pro_bold.ttf) - falling back to raylib's "
                      "built-in default font (Turkish diacritics will not render).\n");
        font_ = GetFontDefault();
    }
    else
    {
        SetTextureFilter(font_.texture, TEXTURE_FILTER_BILINEAR);
    }
}

Renderer3D::~Renderer3D()
{
    // Only unload a font this instance actually loaded itself - never
    // unload GetFontDefault() (raylib owns that one; unloading it would
    // break any other GetFontDefault() user, including raylib's own
    // DrawFPS() call in drawHud() below).
    if (font_.texture.id != GetFontDefault().texture.id)
    {
        UnloadFont(font_);
    }
}

void Renderer3D::drawText(const char* text, int x, int y, int fontSize, Color color) const
{
    DrawTextEx(font_, text, Vector2{static_cast<float>(x), static_cast<float>(y)}, static_cast<float>(fontSize),
               static_cast<float>(fontSize) * kFontSpacingRatio, color);
}

int Renderer3D::measureTextWidth(const char* text, int fontSize) const
{
    const Vector2 size =
        MeasureTextEx(font_, text, static_cast<float>(fontSize), static_cast<float>(fontSize) * kFontSpacingRatio);
    return static_cast<int>(size.x);
}

void Renderer3D::renderFrame(const VirtualWorld& world, bool updateCamera, const VisualTelemetry& telemetry,
                              const ExplorationMap& map, const CoverageTrail& trail,
                              const std::vector<Vec3>& plannedRoute)
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
    drawExplorationMapPanel(world, map, trail, telemetry, plannedRoute);

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

    // Phase 13W human-visual-redesign v2: the old generic-cube demo
    // obstacles are gone entirely - the default workspace's only physical
    // obstacles are the six desk objects (drawn below via
    // drawDeskObject()) and the dock's rear housing (drawChargingDock()).
    // Nothing here draws a plain, undecorated box anymore.
    for (const DeskObject& object : world.deskObjects())
    {
        if (!object.enabled)
        {
            continue;
        }
        drawDeskObject(object);
    }

    drawChargingDock(world);

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

void Renderer3D::drawDeskObject(const DeskObject& object) const
{
    switch (object.type)
    {
        case DeskObjectType::Monitor: drawMonitor(object.position, object.size); break;
        case DeskObjectType::Keyboard: drawKeyboard(object.position, object.size); break;
        case DeskObjectType::Mouse: drawMouse(object.position, object.size); break;
        case DeskObjectType::Mug: drawMug(object.position, object.size); break;
        case DeskObjectType::Notebook: drawNotebook(object.position, object.size); break;
        case DeskObjectType::LampBase: drawLampBase(object.position, object.size); break;
    }
}

// Every drawX() below anchors at `position` (the object's registered
// collision-footprint center - see DeskObject's own docs) and
// `footprint.y`, which is treated as the small slab the object visually
// "sits in" on the desk - anything taller (a monitor's neck/screen, a
// lamp's stem/head) uses its own small set of FIXED internal proportions,
// entirely independent of `footprint`, exactly because the collision
// footprint is deliberately not the full visual volume (see
// RobotCollisionTests.cpp's MonitorFootprintIsStandSizedNotFullScreenVolume).

void Renderer3D::drawMonitor(const Vec3& position, const Vec3& footprint) const
{
    // Phase 13W human-visual-redesign v2: the screen is deliberately the
    // visually DOMINANT desk object (brief: "screen width should be
    // approximately 4.5-5.5x robot width") - scaled up substantially from
    // the v1 redesign's own too-small screen (human validation flagged
    // "the monitor is far too small"). Still entirely independent of
    // `footprint` (the small stand collision box) - see this class' own
    // drawDeskObject() docs for why.
    constexpr float kNeckHeight = 0.22F;
    constexpr float kNeckWidth = 0.08F;
    constexpr float kScreenWidth = 3.0F;
    constexpr float kScreenHeight = 1.65F;
    constexpr float kScreenThickness = 0.15F;
    constexpr float kBezelInset = 0.08F;

    const float baseTopY = position.y + (footprint.y / 2.0F);
    const Vector3 standCenter = toRaylibVector3(position);
    DrawCube(standCenter, footprint.x, footprint.y, footprint.z, kMonitorBodyColor);
    DrawCubeWires(standCenter, footprint.x, footprint.y, footprint.z, kDeskObjectOutlineColor);

    const Vector3 neckCenter{position.x, baseTopY + (kNeckHeight / 2.0F), position.z};
    DrawCube(neckCenter, kNeckWidth, kNeckHeight, kNeckWidth, kMonitorBodyColor);

    const float screenCenterY = baseTopY + kNeckHeight + (kScreenHeight / 2.0F);
    const Vector3 screenCenter{position.x, screenCenterY, position.z};
    DrawCube(screenCenter, kScreenWidth, kScreenHeight, kScreenThickness, kMonitorBodyColor);
    DrawCubeWires(screenCenter, kScreenWidth, kScreenHeight, kScreenThickness, kDeskObjectOutlineColor);

    // Phase 13W human-visual-redesign v2: the screen face points toward
    // +Z (the desk interior/robot workspace, where the keyboard and robot
    // sit - see VirtualWorld.cpp's layout, monitor at the -Z/"rear" edge
    // facing forward) - "the screen should face the desk front/robot
    // workspace... not sideways or away from the default camera."
    const Vector3 faceCenter{position.x, screenCenterY, position.z + (kScreenThickness / 2.0F)};
    DrawCube(faceCenter, kScreenWidth - kBezelInset, kScreenHeight - kBezelInset, 0.01F, kMonitorScreenColor);
}

void Renderer3D::drawKeyboard(const Vec3& position, const Vec3& footprint) const
{
    const Vector3 bodyCenter = toRaylibVector3(position);
    DrawCube(bodyCenter, footprint.x, footprint.y, footprint.z, kKeyboardColor);
    DrawCubeWires(bodyCenter, footprint.x, footprint.y, footprint.z, kDeskObjectOutlineColor);

    // A recognizable silhouette only needs a handful of raised key-row
    // strips, never hundreds of individual keys (this phase's own brief:
    // "do NOT render hundreds of individual keys").
    constexpr int kRowCount = 3;
    constexpr float kRowHeight = 0.015F;
    constexpr float kRowDepth = 0.05F;
    const float rowWidth = footprint.x * 0.85F;
    const float topY = position.y + (footprint.y / 2.0F) + (kRowHeight / 2.0F);
    for (int row = 0; row < kRowCount; ++row)
    {
        const float t = (static_cast<float>(row) + 0.5F) / static_cast<float>(kRowCount);
        const float rowZ = position.z - (footprint.z / 2.0F) + (t * footprint.z);
        DrawCube(Vector3{position.x, topY, rowZ}, rowWidth, kRowHeight, kRowDepth, kKeyRowColor);
    }
}

void Renderer3D::drawMouse(const Vec3& position, const Vec3& footprint) const
{
    const Vector3 bodyCenter = toRaylibVector3(position);
    DrawCube(bodyCenter, footprint.x, footprint.y, footprint.z, kMouseColor);
    DrawCubeWires(bodyCenter, footprint.x, footprint.y, footprint.z, kDeskObjectOutlineColor);

    // A low sphere cap on top approximates a rounded mouse shell - simple
    // primitives only, per this phase's own brief.
    const float capRadius = std::min(footprint.x, footprint.z) * 0.45F;
    const Vector3 capCenter{position.x, position.y + (footprint.y / 2.0F), position.z};
    DrawSphere(capCenter, capRadius, kMouseColor);
}

void Renderer3D::drawMug(const Vec3& position, const Vec3& footprint) const
{
    constexpr int kCylinderSlices = 16;
    const float radius = std::min(footprint.x, footprint.z) / 2.0F;
    const float bodyHeight = footprint.y;
    const Vector3 bottomCenter{position.x, position.y - (bodyHeight / 2.0F), position.z};
    const Vector3 topCenter{position.x, position.y + (bodyHeight / 2.0F), position.z};
    DrawCylinderEx(bottomCenter, topCenter, radius, radius, kCylinderSlices, kMugBodyColor);

    // Simple handle approximation - a single thin box off to one side, per
    // this phase's own brief ("do not make handle collision unnecessarily
    // complex" - this is visual only, no collision of its own).
    const float handleWidth = radius * 0.35F;
    const Vector3 handleCenter{position.x + radius + (handleWidth / 2.0F), position.y, position.z};
    DrawCube(handleCenter, handleWidth, bodyHeight * 0.5F, handleWidth, kMugHandleColor);
}

void Renderer3D::drawNotebook(const Vec3& position, const Vec3& footprint) const
{
    const Vector3 coverCenter = toRaylibVector3(position);
    DrawCube(coverCenter, footprint.x, footprint.y, footprint.z, kNotebookCoverColor);
    DrawCubeWires(coverCenter, footprint.x, footprint.y, footprint.z, kDeskObjectOutlineColor);

    constexpr float kPageInset = 0.04F;
    constexpr float kPageHeightFactor = 0.6F;
    const Vector3 pageCenter{position.x, position.y + (footprint.y / 2.0F) * 0.5F, position.z};
    DrawCube(pageCenter, footprint.x - kPageInset, footprint.y * kPageHeightFactor, footprint.z - kPageInset,
              kNotebookPageColor);
}

void Renderer3D::drawLampBase(const Vec3& position, const Vec3& footprint) const
{
    // Phase 13W human-visual-redesign v2: scaled up alongside the larger
    // (0.8 diameter) base - "stem: visually tall, lamp head: clearly
    // recognizable."
    constexpr int kCylinderSlices = 16;
    constexpr float kStemHeight = 0.7F;
    constexpr float kStemRadius = 0.035F;
    constexpr float kHeadRadius = 0.2F;

    const float baseRadius = std::min(footprint.x, footprint.z) / 2.0F;
    const Vector3 baseBottom{position.x, position.y - (footprint.y / 2.0F), position.z};
    const Vector3 baseTop{position.x, position.y + (footprint.y / 2.0F), position.z};
    DrawCylinderEx(baseBottom, baseTop, baseRadius, baseRadius, kCylinderSlices, kLampBaseColor);

    const Vector3 stemBottom = baseTop;
    const Vector3 stemTop{position.x, baseTop.y + kStemHeight, position.z};
    DrawCylinderEx(stemBottom, stemTop, kStemRadius, kStemRadius, 10, kLampStemColor);

    DrawSphere(stemTop, kHeadRadius, kLampHeadColor);
}

void Renderer3D::drawChargingDock(const VirtualWorld& world) const
{
    const BasePlatform& base = world.basePlatform();
    const Vector3 platformCenter = toRaylibVector3(base.position);
    DrawCube(platformCenter, base.size.x, base.size.y, base.size.z, kDockPlatformColor);
    DrawCubeWires(platformCenter, base.size.x, base.size.y, base.size.z, kDockPlatformOutlineColor);

    // Rear housing - the one physically collidable dock piece (see
    // VirtualWorld::kDockHousingIndex's own docs); drawn at its exact
    // registered collision position, never a separately-eyeballed visual
    // position.
    const BoxObstacle& housing = world.obstacles()[VirtualWorld::kDockHousingIndex];
    const Vector3 housingCenter = toRaylibVector3(housing.position);
    DrawCube(housingCenter, housing.size.x, housing.size.y, housing.size.z, kDockHousingColor);
    DrawCubeWires(housingCenter, housing.size.x, housing.size.y, housing.size.z, kDeskObjectOutlineColor);

    // Two SHORT guide arms flanking the entrance (the platform's +Z/open-
    // desk-interior side, opposite the rear housing at -Z - see
    // VirtualWorld.cpp's kDockHousingZ; Phase 13W final redesign flipped
    // the dock to sit near the desk's REAR edge, so the housing/entrance
    // sides swapped from the earlier v2 pass), and two contact pads on the
    // platform floor near the housing - VISUAL ONLY (this phase's brief,
    // "guide arms may be visual-only" - option B): neither participates in
    // obstacle sensing/collision, so the open parking slot between them
    // always stays physically reachable regardless of approach angle.
    // Rescaled alongside the dock's own smaller footprint (brief's own
    // "guide arms: SHORT... do NOT make a giant floor platform").
    constexpr float kArmWidth = 0.07F;
    constexpr float kArmHeight = 0.08F;
    constexpr float kArmDepth = 0.11F;
    const float armInsetX = (base.size.x / 2.0F) - (kArmWidth / 2.0F);
    const float armCenterZ = base.position.z + (base.size.z / 2.0F) - (kArmDepth / 2.0F);
    const float armCenterY = base.position.y + (base.size.y / 2.0F) + (kArmHeight / 2.0F);
    DrawCube(Vector3{base.position.x - armInsetX, armCenterY, armCenterZ}, kArmWidth, kArmHeight, kArmDepth,
              kDockGuideArmColor);
    DrawCube(Vector3{base.position.x + armInsetX, armCenterY, armCenterZ}, kArmWidth, kArmHeight, kArmDepth,
              kDockGuideArmColor);

    // Phase 13Y: the two dock-side charging pins, drawn at their REAL
    // computed positions (computeDockChargingContacts() -
    // DockChargingContacts.hpp) - never a separately-eyeballed visual
    // guess, exactly like the housing draw above. This is the SAME
    // geometry DockApproachController's own contact-alignment check uses,
    // so what the user sees lining up is exactly what "Docked" actually
    // requires. Small round metallic/gold pins (Phase 13Y brief), never
    // oversized.
    const DockChargingContactPair dockContacts = computeDockChargingContacts(base, world.tableSurface());
    DrawSphere(toRaylibVector3(dockContacts.left.position), dockContacts.left.radius, kDockContactPadColor);
    DrawSphere(toRaylibVector3(dockContacts.right.position), dockContacts.right.radius, kDockContactPadColor);
}

void Renderer3D::drawHud(const VirtualWorld& world, const VisualTelemetry& telemetry) const
{
    const RobotPose& pose = world.robotPose();

    // --- Ayrıntılı (detailed) mode lines ---
    //
    // Final UI/HUD polish: same engineering telemetry content as before
    // this pass (still the full set an engineer/tester would want), only
    // every label - and every literal EN value word this function itself
    // supplies (ON/OFF, YES/NO, CLEAR/BLOCKED, SAFE/EDGE, ACTIVE/INACTIVE)
    // - is now Turkish. Telemetry fields that already carry Turkish text
    // (stateText/commandText/driveAuthorityText/edgeRecoveryStateText/
    // homeNavigationStateText/returnHomeReasonText - see main3d.cpp's
    // turkishText() calls) are placed verbatim, exactly as this function
    // always has.
    char stateLine[80];
    std::snprintf(stateLine, sizeof(stateLine), "Durum: %.*s", static_cast<int>(telemetry.stateText.size()),
                   telemetry.stateText.data());

    char commandLine[80];
    std::snprintf(commandLine, sizeof(commandLine), "Komut: %.*s", static_cast<int>(telemetry.commandText.size()),
                   telemetry.commandText.data());

    char positionLine[64];
    std::snprintf(positionLine, sizeof(positionLine), "Konum: X %.2f  Z %.2f", pose.position.x, pose.position.z);

    char headingLine[64];
    std::snprintf(headingLine, sizeof(headingLine), "Yön: %.1f derece", pose.headingDegrees);

    char obstaclesLine[64];
    std::snprintf(obstaclesLine, sizeof(obstaclesLine), "Engel sayısı: %d", static_cast<int>(world.obstacles().size()));

    // Manual-validation bugfix: one line per perception ray - "AÇIK"
    // when that ray has not detected (whether or not it merely has a
    // distant hit), or its distance when it has - plus the width-aware
    // corridor hazard signal.
    char obstacleLeftLine[64];
    if (telemetry.obstacleRayLeftDetected)
    {
        std::snprintf(obstacleLeftLine, sizeof(obstacleLeftLine), "Engel Sol: %.2f", *telemetry.obstacleRayLeftDistance);
    }
    else
    {
        std::snprintf(obstacleLeftLine, sizeof(obstacleLeftLine), "Engel Sol: AÇIK");
    }

    char obstacleCenterLine[64];
    if (telemetry.obstacleDetected)
    {
        std::snprintf(obstacleCenterLine, sizeof(obstacleCenterLine), "Engel Orta: %.2f", *telemetry.obstacleDistance);
    }
    else
    {
        std::snprintf(obstacleCenterLine, sizeof(obstacleCenterLine), "Engel Orta: AÇIK");
    }

    char obstacleRightLine[64];
    if (telemetry.obstacleRayRightDetected)
    {
        std::snprintf(obstacleRightLine, sizeof(obstacleRightLine), "Engel Sağ: %.2f",
                       *telemetry.obstacleRayRightDistance);
    }
    else
    {
        std::snprintf(obstacleRightLine, sizeof(obstacleRightLine), "Engel Sağ: AÇIK");
    }

    char bodyCorridorObstacleLine[64];
    std::snprintf(bodyCorridorObstacleLine, sizeof(bodyCorridorObstacleLine), "Gövde koridoru: %s",
                   telemetry.bodyCorridorObstacleHazard ? "ENGELLİ" : "AÇIK");

    char leftWheelLine[64];
    std::snprintf(leftWheelLine, sizeof(leftWheelLine), "Sol teker:  %.2f", telemetry.leftWheelSpeed);

    char rightWheelLine[64];
    std::snprintf(rightWheelLine, sizeof(rightWheelLine), "Sağ teker: %.2f", telemetry.rightWheelSpeed);

    char driveAuthorityLine[64];
    std::snprintf(driveAuthorityLine, sizeof(driveAuthorityLine), "Kontrol: %.*s",
                   static_cast<int>(telemetry.driveAuthorityText.size()), telemetry.driveAuthorityText.data());

    char avoidanceLine[64];
    std::snprintf(avoidanceLine, sizeof(avoidanceLine), "Engel kaçınma: %s", telemetry.avoidanceEnabled ? "AÇIK" : "KAPALI");

    char avoidanceActiveLine[64];
    std::snprintf(avoidanceActiveLine, sizeof(avoidanceActiveLine), "Engel kaçınma aktif: %s",
                   telemetry.avoidanceActive ? "EVET" : "HAYIR");

    char avoidanceStateLine[80];
    std::snprintf(avoidanceStateLine, sizeof(avoidanceStateLine), "Kaçınma durumu: %.*s",
                   static_cast<int>(telemetry.avoidanceStateText.size()), telemetry.avoidanceStateText.data());

    char forwardClearanceLine[64];
    std::snprintf(forwardClearanceLine, sizeof(forwardClearanceLine), "Ön açıklık: %s",
                   telemetry.forwardClearanceClear ? "AÇIK" : "ENGELLİ");

    char clearanceLookaheadLine[64];
    std::snprintf(clearanceLookaheadLine, sizeof(clearanceLookaheadLine), "Kaçınma mesafesi: %.2f",
                   telemetry.clearanceLookahead);

    char cliffFrontLeftLine[64];
    std::snprintf(cliffFrontLeftLine, sizeof(cliffFrontLeftLine), "Kenar FL: %s",
                   telemetry.cliffFrontLeft ? "KENAR" : "GÜVENLİ");

    char cliffFrontRightLine[64];
    std::snprintf(cliffFrontRightLine, sizeof(cliffFrontRightLine), "Kenar FR: %s",
                   telemetry.cliffFrontRight ? "KENAR" : "GÜVENLİ");

    char cliffRearLeftLine[64];
    std::snprintf(cliffRearLeftLine, sizeof(cliffRearLeftLine), "Kenar RL: %s",
                   telemetry.cliffRearLeft ? "KENAR" : "GÜVENLİ");

    char cliffRearRightLine[64];
    std::snprintf(cliffRearRightLine, sizeof(cliffRearRightLine), "Kenar RR: %s",
                   telemetry.cliffRearRight ? "KENAR" : "GÜVENLİ");

    char edgeSafetyLine[64];
    std::snprintf(edgeSafetyLine, sizeof(edgeSafetyLine), "Kenar güvenliği: %s",
                   telemetry.edgeSafetyActive ? "AKTİF" : "KAPALI");

    char edgeRecoveryStateLine[96];
    std::snprintf(edgeRecoveryStateLine, sizeof(edgeRecoveryStateLine), "Kurtarma durumu: %.*s",
                   static_cast<int>(telemetry.edgeRecoveryStateText.size()), telemetry.edgeRecoveryStateText.data());

    char edgeTargetHeadingLine[64];
    std::snprintf(edgeTargetHeadingLine, sizeof(edgeTargetHeadingLine), "Kurtarma hedef yönü: %.1f",
                   telemetry.edgeTargetHeadingDegrees);

    char edgeHeadingErrorLine[64];
    std::snprintf(edgeHeadingErrorLine, sizeof(edgeHeadingErrorLine), "Kurtarma yön hatası: %.1f",
                   telemetry.edgeHeadingErrorDegrees);

    char collisionLine[64];
    std::snprintf(collisionLine, sizeof(collisionLine), "Çarpışma: %s", telemetry.collidedLastUpdate ? "EVET" : "HAYIR");

    char returnReasonLine[64];
    std::snprintf(returnReasonLine, sizeof(returnReasonLine), "Dönüş nedeni: %.*s",
                   static_cast<int>(telemetry.returnHomeReasonText.size()), telemetry.returnHomeReasonText.data());

    char homeNavStateLine[80];
    std::snprintf(homeNavStateLine, sizeof(homeNavStateLine), "Eve dönüş: %.*s",
                   static_cast<int>(telemetry.homeNavigationStateText.size()), telemetry.homeNavigationStateText.data());

    char homeNavDistanceLine[64];
    std::snprintf(homeNavDistanceLine, sizeof(homeNavDistanceLine), "Eve uzaklık: %.2f",
                   telemetry.homeNavigationDistance);

    char homeNavTargetHeadingLine[64];
    std::snprintf(homeNavTargetHeadingLine, sizeof(homeNavTargetHeadingLine), "Hedef yön: %.1f",
                   telemetry.homeNavigationTargetHeadingDegrees);

    char homeNavHeadingErrorLine[64];
    std::snprintf(homeNavHeadingErrorLine, sizeof(homeNavHeadingErrorLine), "Yön hatası: %.1f",
                   telemetry.homeNavigationHeadingErrorDegrees);

    // Phase 13X human-validation fix: Ayrıntılı-only breakdown of the
    // HARİTA panel's own single "Keşfedilen" number - "Erişilebilir keşif"
    // (accessible/reachable exploration - uses the exact same
    // displayedExploredPercentage() substitution the map panel itself
    // uses, so it always agrees with what that panel shows) alongside
    // "Ham keşif" (raw, always-truthful ExplorationMap::exploredPercentage(),
    // never substituted) - lets an engineering observer see both numbers
    // at once and understand WHY they can legitimately differ (see
    // MapPanelStatus.hpp's own docs). Presentation-only, exactly like
    // every other line in this panel.
    char accessibleExploredLine[64];
    std::snprintf(accessibleExploredLine, sizeof(accessibleExploredLine), "Erişilebilir keşif: %%%d",
                   displayedExploredPercentage(telemetry.logicalExplorationComplete, telemetry.rawExploredPercentage));

    char rawExploredLine[64];
    std::snprintf(rawExploredLine, sizeof(rawExploredLine), "Ham keşif: %%%d",
                   static_cast<int>(telemetry.rawExploredPercentage));

    const HudLine detailedLines[] = {
        {"ROBOT DURUMU - AYRINTILI", 20, kHudTitleColor},
        {stateLine, 18, kHudTextColor},
        {commandLine, 18, kHudTextColor},
        {driveAuthorityLine, 18, kHudTextColor},
        {avoidanceLine, 18, kHudTextColor},
        {avoidanceActiveLine, 18, kHudTextColor},
        {avoidanceStateLine, 18, kHudTextColor},
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
        {accessibleExploredLine, 18, kHudTextColor},
        {rawExploredLine, 18, kHudTextColor},
        {"TAB Fareyi Yakala/Bırak   F11 Tam Ekran   SPACE Duraklat   O Engeli Aç/Kapat   Fare: kamera",
         16, kHudControlsColor},
        {"M Manuel Kontrol   Ok Tuşları Sürüş   X Manuel Durdur", 16, kHudControlsColor},
        {"A Engel Kaçınma Aç/Kapat   H: Sade Görünüm", 16, kHudControlsColor},
    };

    // --- Sade (simple, default) mode lines ---
    //
    // Final UI/HUD polish: the new default presentation - a small,
    // translated, high-value operational summary (state/task/control
    // authority/obstacle/edge/battery), plus AT MOST ONE optional
    // contextual line, never both - active table-edge safety recovery
    // (the higher-authority fact) takes priority over a plain distance-
    // to-base readout, which itself only appears while a Return Home is
    // actually the current task. This replaces the old engineering-subset
    // "Compact" panel entirely; see the brief's own simplification-
    // priority ordering (state, task, authority, obstacle, edge, battery).
    char simpleStateLine[64];
    std::snprintf(simpleStateLine, sizeof(simpleStateLine), "Durum: %.*s", static_cast<int>(telemetry.stateText.size()),
                   telemetry.stateText.data());

    char simpleTaskLine[64];
    std::snprintf(simpleTaskLine, sizeof(simpleTaskLine), "Görev: %.*s",
                   static_cast<int>(telemetry.missionTaskText.size()), telemetry.missionTaskText.data());

    char simpleAuthorityLine[64];
    std::snprintf(simpleAuthorityLine, sizeof(simpleAuthorityLine), "Kontrol: %.*s",
                   static_cast<int>(telemetry.driveAuthorityText.size()), telemetry.driveAuthorityText.data());

    char simpleObstacleLine[32];
    std::snprintf(simpleObstacleLine, sizeof(simpleObstacleLine), "Engel: %s", telemetry.obstacleHazard ? "Var" : "Yok");

    // Pure presentation selection over four already-computed booleans -
    // never a cliff/edge-safety decision of its own (that remains
    // entirely VirtualCliffSensor/TableEdgeSafetyController's job).
    const bool anyCliffEdge =
        telemetry.cliffFrontLeft || telemetry.cliffFrontRight || telemetry.cliffRearLeft || telemetry.cliffRearRight;
    char simpleEdgeLine[32];
    std::snprintf(simpleEdgeLine, sizeof(simpleEdgeLine), "Kenar: %s",
                   (anyCliffEdge || telemetry.edgeSafetyActive) ? "Tehlike" : "Güvenli");

    char simpleBatteryLine[32];
    std::snprintf(simpleBatteryLine, sizeof(simpleBatteryLine), "Batarya: %%%d", telemetry.batteryPercent);

    char simpleContextLine[48];
    bool hasSimpleContextLine = true;
    if (telemetry.edgeSafetyActive)
    {
        std::snprintf(simpleContextLine, sizeof(simpleContextLine), "Güvenlik: %.*s",
                       static_cast<int>(telemetry.edgeRecoveryStateText.size()), telemetry.edgeRecoveryStateText.data());
    }
    else if (telemetry.returningHomeTask)
    {
        std::snprintf(simpleContextLine, sizeof(simpleContextLine), "Eve uzaklık: %.2f", telemetry.baseDistance);
    }
    else
    {
        hasSimpleContextLine = false;
    }

    HudLine simpleLines[9];
    std::size_t simpleLineCount = 0;
    simpleLines[simpleLineCount++] = {"ROBOT DURUMU", 20, kHudTitleColor};
    simpleLines[simpleLineCount++] = {simpleStateLine, 18, kHudTextColor};
    simpleLines[simpleLineCount++] = {simpleTaskLine, 18, kHudTextColor};
    simpleLines[simpleLineCount++] = {simpleAuthorityLine, 18, kHudTextColor};
    simpleLines[simpleLineCount++] = {simpleObstacleLine, 18, kHudTextColor};
    simpleLines[simpleLineCount++] = {simpleEdgeLine, 18, kHudTextColor};
    simpleLines[simpleLineCount++] = {simpleBatteryLine, 18, kHudTextColor};
    if (hasSimpleContextLine)
    {
        simpleLines[simpleLineCount++] = {simpleContextLine, 18, kHudTextColor};
    }
    simpleLines[simpleLineCount++] = {"H: Ayrıntılı Bilgi", 16, kHudControlsColor};

    // Which array is used - and therefore the panel's size - depends
    // entirely on telemetry.hudMode; this is the only behavioral
    // difference between the two modes. HudMode::Compact now backs the
    // Sade (simple, default) panel; HudMode::Full now backs the
    // Ayrıntılı (detailed) panel - see Renderer3D.hpp's own HudMode docs.
    const bool simple = telemetry.hudMode == HudMode::Compact;
    const HudLine* lines = simple ? simpleLines : detailedLines;
    const std::size_t lineCount =
        simple ? simpleLineCount : (sizeof(detailedLines) / sizeof(detailedLines[0]));

    // Panel sized to fully contain the widest line so contrast holds
    // regardless of content length - a fixed guessed width could leave a
    // line's tail spilling back onto the unshaded scene. Sade mode's much
    // shorter array naturally yields a much smaller panel here, with no
    // separate size-mode logic needed. measureTextWidth()/drawText() (not
    // raylib's own MeasureText()/DrawText()) so both Turkish diacritics
    // and the loaded font's own metrics are measured/drawn consistently -
    // see Renderer3D.hpp's own font_ docs.
    int panelWidth = 0;
    int contentHeight = 0;
    for (std::size_t i = 0; i < lineCount; ++i)
    {
        panelWidth = std::max(panelWidth, measureTextWidth(lines[i].text, lines[i].fontSize));
        contentHeight += lines[i].fontSize + kHudLineSpacing;
    }
    panelWidth += 2 * kHudPadding;
    const int panelHeight = contentHeight + (2 * kHudPadding) - kHudLineSpacing;

    DrawRectangle(kHudMarginX, kHudMarginY, panelWidth, panelHeight, kHudPanelBackground);

    const int textX = kHudMarginX + kHudPadding;
    int textY = kHudMarginY + kHudPadding;
    for (std::size_t i = 0; i < lineCount; ++i)
    {
        drawText(lines[i].text, textX, textY, lines[i].fontSize, lines[i].color);
        textY += lines[i].fontSize + kHudLineSpacing;
    }

    DrawFPS(10, GetScreenHeight() - 30);
}

void Renderer3D::drawMissionControlPanel(const VisualTelemetry& telemetry) const
{
    // Final UI/HUD polish: translated title/labels; "Base distance" is
    // deliberately removed from this panel (drawHud()'s Sade mode already
    // shows an equivalent "Eve uzaklık" contextual line while a Return
    // Home is the active task - showing the same distance in both panels
    // would be duplicate telemetry, which the brief explicitly calls out
    // to avoid). taskLine reuses telemetry.missionTaskText verbatim - it
    // is already Turkish (main3d.cpp's turkishText(MissionTask) call).
    // Phase 13V human-validation fix: the former "Ev bölgesi: İçeride/
    // Dışarıda" line is removed - it no longer affects any behavior
    // (HomeZoneMonitor's automatic-return wiring was removed entirely),
    // so keeping it would only be presentation clutter, not a decision
    // in scope of the panel's own preferred appearance (see main3d.cpp's
    // own docs and docs/technical-decisions.md for the full rationale).
    char taskLine[48];
    std::snprintf(taskLine, sizeof(taskLine), "Görev: %.*s", static_cast<int>(telemetry.missionTaskText.size()),
                   telemetry.missionTaskText.data());

    const HudLine lines[] = {
        {"GÖREV KONTROLÜ", 20, kHudTitleColor},
        {taskLine, 18, kMissionControlTaskColor},
        {"1  Haritalamayı Başlat", 16, kHudControlsColor},
        {"2  Eve Dön", 16, kHudControlsColor},
        {"3  Görevi Durdur", 16, kHudControlsColor},
        {"R  Eve Dön", 16, kHudControlsColor},
    };
    constexpr std::size_t lineCount = sizeof(lines) / sizeof(lines[0]);

    int panelWidth = 0;
    int contentHeight = 0;
    for (std::size_t i = 0; i < lineCount; ++i)
    {
        panelWidth = std::max(panelWidth, measureTextWidth(lines[i].text, lines[i].fontSize));
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
        drawText(lines[i].text, textX, textY, lines[i].fontSize, lines[i].color);
        textY += lines[i].fontSize + kHudLineSpacing;
    }
}

void Renderer3D::drawExplorationMapPanel(const VirtualWorld& world, const ExplorationMap& map,
                                          const CoverageTrail& trail, const VisualTelemetry& telemetry,
                                          const std::vector<Vec3>& plannedRoute) const
{
    // Phase 13X human-validation fix: the displayed percentage substitutes
    // a clean 100 once exploration is LOGICALLY complete (no reachable
    // frontier remains), even though raw ExplorationMap::exploredPercentage()
    // truthfully stays below 100 forever for interior/occluded cells - see
    // MapPanelStatus.hpp's own docs. map.exploredPercentage() itself is
    // never mutated by this - only the number drawn here changes.
    char exploredLine[48];
    std::snprintf(exploredLine, sizeof(exploredLine), "Keşfedilen: %%%d",
                   displayedExploredPercentage(telemetry.logicalExplorationComplete, map.exploredPercentage()));

    // Phase 13X human-validation fix: four-way status (NewMap/Mapping/
    // Loaded/Completed - see MapPanelStatus.hpp's own docs on why
    // "Tamamlandı" must win even the instant Haritalama itself stops being
    // the active task) replaces the old two-way Yüklendi/Oluşturuluyor
    // choice, which could not distinguish "still mapping" from "mapping
    // just finished" - the exact human-observed ambiguity this fix
    // resolves. Renderer3D decides the Turkish text for this one panel
    // line directly (matching this line's own pre-existing local pattern,
    // unlike most other telemetry text which main3d.cpp pre-translates -
    // see VisualTelemetry::logicalExplorationComplete's own docs).
    const MapPanelStatus panelStatus =
        deriveMapPanelStatus(telemetry.logicalExplorationComplete, telemetry.explorationActive, telemetry.mapWasLoaded);
    const char* statusText = "Yeni Harita";
    switch (panelStatus)
    {
        case MapPanelStatus::NewMap: statusText = "Yeni Harita"; break;
        case MapPanelStatus::Mapping: statusText = "Haritalanıyor"; break;
        case MapPanelStatus::Loaded: statusText = "Yüklendi"; break;
        case MapPanelStatus::Completed: statusText = "Tamamlandı"; break;
    }
    char statusLine[48];
    std::snprintf(statusLine, sizeof(statusLine), "Durum: %s", statusText);

    const HudLine headerLines[] = {
        {"HARİTA", 20, kHudTitleColor},
        {exploredLine, 16, kHudTextColor},
        {statusLine, 16, kHudTextColor},
    };
    constexpr std::size_t headerLineCount = sizeof(headerLines) / sizeof(headerLines[0]);

    // Phase 13W human-visual-redesign v2: the grid viewport's height is
    // derived from the map's own real TableSurface aspect ratio (never a
    // second, hardcoded height) - a wide, short viewport for the current
    // ~2:1 desk, but this keeps matching whatever the table's actual
    // proportions are if they ever change again.
    const TableSurface& gridBounds = map.bounds();
    const float gridWorldWidth = gridBounds.maxX - gridBounds.minX;
    const float gridWorldHeight = gridBounds.maxZ - gridBounds.minZ;
    const int gridPixelWidth = kExplorationGridPixelWidth;
    const int gridPixelHeight = (gridWorldWidth > 0.0F)
                                     ? std::max(1, static_cast<int>(static_cast<float>(gridPixelWidth) *
                                                                     (gridWorldHeight / gridWorldWidth)))
                                     : gridPixelWidth;

    int panelWidth = gridPixelWidth;
    int headerHeight = 0;
    for (std::size_t i = 0; i < headerLineCount; ++i)
    {
        panelWidth = std::max(panelWidth, measureTextWidth(headerLines[i].text, headerLines[i].fontSize));
        headerHeight += headerLines[i].fontSize + kHudLineSpacing;
    }
    panelWidth += 2 * kHudPadding;
    const int panelHeight = headerHeight + gridPixelHeight + (3 * kHudPadding);

    const int panelX = GetScreenWidth() - kExplorationPanelMarginRight - panelWidth;
    const int panelY = GetScreenHeight() - kExplorationPanelMarginBottom - panelHeight;
    DrawRectangle(panelX, panelY, panelWidth, panelHeight, kHudPanelBackground);

    const int textX = panelX + kHudPadding;
    int textY = panelY + kHudPadding;
    for (std::size_t i = 0; i < headerLineCount; ++i)
    {
        drawText(headerLines[i].text, textX, textY, headerLines[i].fontSize, headerLines[i].color);
        textY += headerLines[i].fontSize + kHudLineSpacing;
    }

    // --- Grid: top-down, fixed orientation (never rotates with the
    // robot) - column increases with world X (left->right), row
    // increases with world Z (top->bottom), matching
    // ExplorationMap::worldToCell()'s own convention exactly (Phase 13V
    // brief: "do not accidentally mirror or rotate the map"). Only
    // Free/Occupied cells are drawn - Unknown cells are left as the
    // panel's own background color (kExplorationUnknownColor IS
    // kHudPanelBackground), both correctly representing "not observed
    // yet" and avoiding ~10000 redundant background-colored draw calls
    // every frame for a freshly-started map.
    const int gridX = textX;
    const int gridY = textY + kHudPadding;
    const TableSurface& bounds = gridBounds;
    const float pixelsPerCellX = static_cast<float>(gridPixelWidth) / static_cast<float>(map.width());
    const float pixelsPerCellZ = static_cast<float>(gridPixelHeight) / static_cast<float>(map.height());

    const auto worldToPanel = [&](float worldX, float worldZ) {
        const float normX = (worldX - bounds.minX) / gridWorldWidth;
        const float normZ = (worldZ - bounds.minZ) / gridWorldHeight;
        return Vector2{static_cast<float>(gridX) + (normX * static_cast<float>(gridPixelWidth)),
                        static_cast<float>(gridY) + (normZ * static_cast<float>(gridPixelHeight))};
    };

    const std::vector<MapCell>& cells = map.cells();
    for (int row = 0; row < map.height(); ++row)
    {
        for (int col = 0; col < map.width(); ++col)
        {
            const MapCell cell = cells[(static_cast<std::size_t>(row) * static_cast<std::size_t>(map.width())) +
                                        static_cast<std::size_t>(col)];
            if (cell == MapCell::Unknown)
            {
                continue;
            }
            const Color cellColor = (cell == MapCell::Occupied) ? kExplorationOccupiedColor : kExplorationFreeColor;
            const int cellX = gridX + static_cast<int>(static_cast<float>(col) * pixelsPerCellX);
            const int cellY = gridY + static_cast<int>(static_cast<float>(row) * pixelsPerCellZ);
            // ceil-rounded width/height so adjacent cells fully tile the
            // grid with no visible seam from float->int truncation.
            const int cellWidth = static_cast<int>(std::ceil(pixelsPerCellX));
            const int cellHeight = static_cast<int>(std::ceil(pixelsPerCellZ));
            DrawRectangle(cellX, cellY, cellWidth, cellHeight, cellColor);
        }
    }

    // --- Travel trail: the actual physical path recorded by
    // CoverageTrail, drawn as connected line segments between
    // consecutive sampled points - Explore/Return Home/Safety-recovery/
    // Avoidance/manual movement all contribute to the same one trail
    // (Phase 13V brief, "trail lifecycle").
    const std::vector<Vec3>& trailPoints = trail.points();
    for (std::size_t i = 1; i < trailPoints.size(); ++i)
    {
        const Vector2 from = worldToPanel(trailPoints[i - 1].x, trailPoints[i - 1].z);
        const Vector2 to = worldToPanel(trailPoints[i].x, trailPoints[i].z);
        DrawLineEx(from, to, 2.0F, kExplorationTrailColor);
    }

    // --- Planned route (Phase 13X): the currently intended route
    // (GridPathPlanner + simplifyPath, already fully computed by
    // main3d.cpp - this class performs no A*/planning of its own), drawn
    // in a visually distinct, restrained style so it never reads as the
    // travel trail above - trail = where the robot HAS been, planned
    // route = where it currently INTENDS to go. Drawn on the HARİTA panel
    // unconditionally (this panel itself is not gated by HudMode), so the
    // route stays visible regardless of Sade/Ayrıntılı.
    for (std::size_t i = 1; i < plannedRoute.size(); ++i)
    {
        const Vector2 from = worldToPanel(plannedRoute[i - 1].x, plannedRoute[i - 1].z);
        const Vector2 to = worldToPanel(plannedRoute[i].x, plannedRoute[i].z);
        DrawLineEx(from, to, 2.0F, kPlannedRouteColor);
    }

    // --- Frontier target marker (Phase 13X): a small ring around the
    // current exploration target, only while one is actually held (never
    // shown during Return Home or once mapping is complete).
    if (telemetry.frontierTargetVisible)
    {
        const Vector2 targetPanel =
            worldToPanel(telemetry.frontierTargetPosition.x, telemetry.frontierTargetPosition.z);
        DrawCircleLines(static_cast<int>(targetPanel.x), static_cast<int>(targetPanel.y), 5.0F, kPlannedRouteColor);
    }

    // --- Base marker: BasePlatform's own live position - never a
    // duplicated coordinate (Phase 13V brief, "do not duplicate its
    // coordinates"). Phase 13W human-visual-redesign v2: kept compact
    // (6x6, down from 8x8) now that the panel itself is shorter (150px
    // vs the old 220px square) - "home/dock marker should also be
    // compact."
    const Vector2 basePanel = worldToPanel(world.basePlatform().position.x, world.basePlatform().position.z);
    DrawRectangle(static_cast<int>(basePanel.x) - 3, static_cast<int>(basePanel.y) - 3, 6, 6, kExplorationBaseColor);

    // --- Robot marker: small filled circle at the current position plus
    // a short heading-direction line, using this project's one heading
    // convention (0 = +Z, +90 = +X - VisualMath.hpp's forwardDirection(),
    // matching the grid's own row/column axes exactly, so the marker
    // never visually disagrees with which way the grid itself is
    // oriented). Phase 13W human-visual-redesign v2: radius kept small
    // (3px) relative to the new, shorter panel - never a significant
    // fraction of the 67x33-ish grid.
    const RobotPose& pose = world.robotPose();
    const Vector2 robotPanel = worldToPanel(pose.position.x, pose.position.z);
    DrawCircleV(robotPanel, 3.0F, kExplorationRobotColor);
    constexpr float kHeadingMarkerLengthPixels = 8.0F;
    constexpr float kPi = 3.14159265358979323846F;
    const float headingRadians = pose.headingDegrees * (kPi / 180.0F);
    const Vector2 headingEnd{robotPanel.x + (std::sin(headingRadians) * kHeadingMarkerLengthPixels),
                              robotPanel.y + (std::cos(headingRadians) * kHeadingMarkerLengthPixels)};
    DrawLineEx(robotPanel, headingEnd, 2.0F, kExplorationHeadingColor);

    // --- Table boundary outline, drawn last so it stays visible over any
    // cell/trail/marker drawing near the grid's own edge.
    DrawRectangleLines(gridX, gridY, gridPixelWidth, gridPixelHeight, kExplorationBoundaryColor);
}

} // namespace robot::visual
