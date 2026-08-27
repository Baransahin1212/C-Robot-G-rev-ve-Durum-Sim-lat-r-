#pragma once

#include <string_view>

#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/DockApproachController.hpp"
#include "robot/visual/HomeNavigator.hpp"
#include "robot/visual/MissionTask.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/TableEdgeSafetyController.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"
#include "robot/visual/WaypointNavigator.hpp"

// Final UI/HUD polish (presentation-only): Turkish display-string mapping
// for RobotSimulator3D's HUD/Mission Control panel. Every function here is
// a pure `enum -> Turkish std::string_view` mapping - never a rename of any
// internal domain/FSM/visual-simulation identifier. RobotState, EventType,
// ReturnHomeReason, MissionTask, DriveAuthority, VirtualDriveCommand,
// HomeNavigationState, and TableEdgeSafetyController::RecoveryState remain
// exactly as defined in their own headers, and their existing English
// toString() overloads are completely untouched and still used wherever
// English output is required (RobotSimulator's CLI report output, and any
// test that asserts on those exact strings) - this header is included ONLY
// by main3d.cpp, never by RobotSimulator/robot_app or any core/test target.
//
// Deliberately included by main3d.cpp only, not by Renderer3D.cpp/.hpp:
// Renderer3D is documented to have "no knowledge of RobotStateMachine,
// RobotRuntime, RobotController, or IRobotHardware" and only ever displays
// already-formatted VisualTelemetry text - this header is exactly the kind
// of FSM/hardware-aware type knowledge that boundary exists to keep out of
// robot_visual. main3d.cpp already legitimately depends on every type named
// below (it constructs all of them), so this is the correct - and only -
// place enum -> Turkish text conversion belongs.
//
// Final Turkish-font polish: these strings use real Turkish orthography
// (ç/Ç, ğ/Ğ, ı, i/İ, ö/Ö, ş/Ş, ü/Ü) - an earlier pass of this same phase
// used an ASCII-folded fallback (e.g. "Gorev") because Renderer3D only had
// raylib's built-in default font (fixed at Unicode U+0000-U+00FF) to draw
// with, which cannot render Turkish's four extended-Latin-only letters
// (ğ/Ğ, ı, ş/Ş, İ) at all. Renderer3D now loads a bundled Turkish-capable
// font instead (assets/fonts/anonymous_pro_bold.ttf via LoadFontEx - see
// Renderer3D.cpp's constructor and assets/fonts/LICENSE-AnonymousPro.txt),
// so full diacritics render correctly; CMakeLists.txt's /utf-8 flag
// (scoped to robot_visual/RobotSimulator3D only) ensures these literals'
// compiled bytes are genuine UTF-8, matching what that font/raylib's
// decoder expect - see CMakeLists.txt's own comment for the full reasoning.

namespace robot::visual
{

constexpr std::string_view turkishText(RobotState value) noexcept
{
    switch (value)
    {
        case RobotState::Idle: return "Bekliyor";
        case RobotState::Ready: return "Hazır";
        case RobotState::Moving: return "Hareket Ediyor";
        case RobotState::WaitingForObstacleClear: return "Engel Bekleniyor";
        case RobotState::ReturningHome: return "Eve Dönüyor";
        case RobotState::Completed: return "Tamamlandı";
        case RobotState::Aborted: return "İptal Edildi";
        case RobotState::EmergencyStopped: return "Acil Durdurma";
        case RobotState::Error: return "Hata";
    }
    return "Bilinmiyor";
}

constexpr std::string_view turkishText(ReturnHomeReason value) noexcept
{
    switch (value)
    {
        case ReturnHomeReason::None: return "-";
        case ReturnHomeReason::MissionAbort: return "Görev İptali";
        case ReturnHomeReason::UserRequest: return "Kullanıcı İsteği";
    }
    return "Bilinmiyor";
}

constexpr std::string_view turkishText(MissionTask value) noexcept
{
    switch (value)
    {
        case MissionTask::None: return "YOK";
        case MissionTask::Roam: return "GEZİNME";
        case MissionTask::ReturnHome: return "EVE DÖNÜŞ";
    }
    return "BİLİNMİYOR";
}

constexpr std::string_view turkishText(DriveAuthority value) noexcept
{
    switch (value)
    {
        case DriveAuthority::Fsm: return "Normal";
        case DriveAuthority::Navigation: return "Eve Dönüş";
        case DriveAuthority::AutonomousAvoidance: return "Engel Kaçınma";
        case DriveAuthority::Manual: return "Manuel";
        case DriveAuthority::Safety: return "Güvenlik";
    }
    return "Bilinmiyor";
}

constexpr std::string_view turkishText(VirtualDriveCommand value) noexcept
{
    switch (value)
    {
        case VirtualDriveCommand::Stopped: return "Durdu";
        case VirtualDriveCommand::MoveForward: return "İleri";
        case VirtualDriveCommand::ReturnToBase: return "Eve Dönüyor";
    }
    return "Bilinmiyor";
}

constexpr std::string_view turkishText(HomeNavigationState value) noexcept
{
    switch (value)
    {
        case HomeNavigationState::Inactive: return "Kapalı";
        case HomeNavigationState::Aligning: return "Yöneliyor";
        case HomeNavigationState::Driving: return "Eve Gidiyor";
        case HomeNavigationState::Arrived: return "Eve Ulaştı";
    }
    return "Bilinmiyor";
}

// Phase 13X: map-aware waypoint-following - the same production/planning
// state now behind BOTH Return Home and frontier exploration navigation.
// "Rota İzleniyor" (following the route) is deliberately generic enough to
// read correctly for either - main3d.cpp's own Mission Control panel text
// (missionTaskText, MissionTask.hpp's own turkishText()) already
// disambiguates "Haritalama"/"Keşif" vs. "Eve Dönüş" for the user.
constexpr std::string_view turkishText(WaypointNavigatorState value) noexcept
{
    switch (value)
    {
        case WaypointNavigatorState::Inactive: return "Kapalı";
        case WaypointNavigatorState::Following: return "Rota İzleniyor";
        case WaypointNavigatorState::Arrived: return "Ulaşıldı";
        case WaypointNavigatorState::Failed: return "Rota Bulunamadı";
    }
    return "Bilinmiyor";
}

// Phase 13X final-approach fix, extended by Phase 13Y's precision reverse
// docking: Stage 2 docking-phase state - shown on the HUD in place of
// WaypointNavigatorState's own text once Stage 1 hands off (see
// main3d.cpp's own telemetry assembly), so the HUD never misleadingly
// reads "Ulaşıldı" (Arrived) while the robot is still visibly aligning/
// reversing the last short stretch into the dock. Phase 13Y brief's own
// suggested Ayrıntılı-HUD phase labels (Yaklaşıyor/Hizalanıyor/Geri Park
// Ediyor/Park Edildi) - "Temas" (contact) is not a separate state in this
// controller's own state machine (contact alignment IS the Docked
// transition condition, never a distinct intermediate state), so it is
// not a separate HUD value here.
constexpr std::string_view turkishText(DockApproachState value) noexcept
{
    switch (value)
    {
        case DockApproachState::Inactive: return "Kapalı";
        case DockApproachState::NavigateToStagingPoint: return "Yaklaşıyor";
        case DockApproachState::AlignForReverse: return "Hizalanıyor";
        case DockApproachState::ReverseApproach: return "Geri Park Ediyor";
        case DockApproachState::Docked: return "Park Edildi";
        case DockApproachState::Failed: return "Yanaşma Başarısız";
    }
    return "Bilinmiyor";
}

constexpr std::string_view turkishText(TableEdgeSafetyController::RecoveryState value) noexcept
{
    switch (value)
    {
        case TableEdgeSafetyController::RecoveryState::Inactive: return "Kapalı";
        case TableEdgeSafetyController::RecoveryState::BackingAway: return "Geri Çekiliyor";
        case TableEdgeSafetyController::RecoveryState::MovingForwardFromRearEdge: return "İleri Çıkıyor";
        case TableEdgeSafetyController::RecoveryState::Turning: return "İçeri Dönüyor";
        case TableEdgeSafetyController::RecoveryState::AdvancingInward: return "Güvenli Alana Giriyor";
    }
    return "Bilinmiyor";
}

constexpr std::string_view turkishText(AvoidanceState value) noexcept
{
    switch (value)
    {
        case AvoidanceState::Inactive: return "Kapalı";
        case AvoidanceState::TurnAway: return "Engelden Dönüyor";
        case AvoidanceState::AdvanceClear: return "Engeli Geçiyor";
    }
    return "Bilinmiyor";
}

// Phase 13W: desktop-workspace object names. No current HUD panel
// displays an object list (per this phase's own brief - "avoid new UI
// clutter"), so nothing calls this yet; it exists so a future panel that
// DOES want to name a desk object has the correct Turkish terms ready,
// rather than inventing/translating them ad hoc later. "Şarj İstasyonu"
// (charging dock/station) has no DeskObjectType of its own (the dock is
// drawn straight from BasePlatform - see Renderer3D::drawChargingDock())
// - noted here as the term to use if the dock is ever named in the UI.
constexpr std::string_view turkishText(DeskObjectType value) noexcept
{
    switch (value)
    {
        case DeskObjectType::Monitor: return "Monitör";
        case DeskObjectType::Keyboard: return "Klavye";
        case DeskObjectType::Mouse: return "Fare";
        case DeskObjectType::Mug: return "Kupa";
        case DeskObjectType::Notebook: return "Defter";
        case DeskObjectType::LampBase: return "Masa Lambası";
    }
    return "Bilinmiyor";
}

} // namespace robot::visual
