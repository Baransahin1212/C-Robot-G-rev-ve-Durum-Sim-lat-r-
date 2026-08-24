#pragma once

#include <string_view>

#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/visual/HomeNavigator.hpp"
#include "robot/visual/MissionTask.hpp"
#include "robot/visual/ReactiveObstacleAvoidance.hpp"
#include "robot/visual/TableEdgeSafetyController.hpp"
#include "robot/visual/VirtualRobotHardware.hpp"

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

} // namespace robot::visual
