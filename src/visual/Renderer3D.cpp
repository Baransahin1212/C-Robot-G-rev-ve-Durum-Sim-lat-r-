#include "robot/visual/Renderer3D.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

#include "robot/visual/VisualRobot.hpp"

// Final font delivery fix: GetModuleFileNameA() resolves RobotSimulator3D's
// own executable directory at runtime (see exeDirectory() below), replacing
// the earlier absolute compile-time FONT_PATH (which baked this developer
// machine's source-tree location into the binary - not portable to a
// checkout built elsewhere). raylib.h is already included above (via
// Renderer3D.hpp), so windows.h must come after it; NOGDI/NOUSER avoid the
// well-known raylib/Windows.h symbol clashes (Rectangle, CloseWindow,
// ShowCursor, DrawText, ...) - GetModuleFileNameA lives outside both the
// GDI and USER subsystems, so neither macro affects it. NOMINMAX prevents
// windows.h's own min/max macros from shadowing std::max() used below in
// drawHud()/drawMissionControlPanel(). Scoped to this one translation unit
// only, never a project-wide Windows dependency.
#define NOGDI
#define NOUSER
#define NOMINMAX
#include <windows.h>

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

// Final font delivery fix: the directory RobotSimulator3D.exe itself is
// running from - GetModuleFileNameA(nullptr, ...) asks Windows for the
// current process's own module (.exe) path, so this works regardless of
// the process's current working directory or where the source tree
// happens to live on this machine. Returns an empty string (never throws
// or crashes) if the OS call fails for any reason - the constructor below
// treats that exactly like a missing font file and falls back to the
// default font.
std::string exeDirectory()
{
    char buffer[MAX_PATH] = {};
    const DWORD length = GetModuleFileNameA(nullptr, buffer, MAX_PATH);
    if (length == 0 || length == MAX_PATH)
    {
        return std::string();
    }
    const std::string path(buffer, length);
    const std::size_t lastSlash = path.find_last_of("\\/");
    return (lastSlash == std::string::npos) ? std::string() : path.substr(0, lastSlash);
}

} // namespace

Renderer3D::Renderer3D()
    : camera_{}
    , font_{}
{
    camera_.position = Vector3{8.0F, 8.0F, 8.0F};
    camera_.target = Vector3{0.0F, 0.0F, 0.0F};
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
    const std::string exeDir = exeDirectory();
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

    const HudLine detailedLines[] = {
        {"ROBOT DURUMU - AYRINTILI", 20, kHudTitleColor},
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
    char taskLine[48];
    std::snprintf(taskLine, sizeof(taskLine), "Görev: %.*s", static_cast<int>(telemetry.missionTaskText.size()),
                   telemetry.missionTaskText.data());

    char homeZoneLine[48];
    std::snprintf(homeZoneLine, sizeof(homeZoneLine), "Ev bölgesi: %s",
                   telemetry.homeZoneInside ? "İçeride" : "Dışarıda");

    const HudLine lines[] = {
        {"GÖREV KONTROLÜ", 20, kHudTitleColor},
        {taskLine, 18, kMissionControlTaskColor},
        {"1  Gezinmeyi Başlat", 16, kHudControlsColor},
        {"2  Eve Dön", 16, kHudControlsColor},
        {"3  Görevi Durdur", 16, kHudControlsColor},
        {"R  Eve Dön", 16, kHudControlsColor},
        {homeZoneLine, 16, kHudTextColor},
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

} // namespace robot::visual
