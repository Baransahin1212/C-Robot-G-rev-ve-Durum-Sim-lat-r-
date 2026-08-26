#pragma once

#include <string_view>

namespace robot::visual
{

// Phase 13X human-validation fix: the HARİTA (map) panel's own small,
// presentation-only status classification - distinct from MissionTask
// (which describes what the ROBOT is currently doing) and from
// VisualTelemetry::mapWasLoaded (which only ever describes how THIS
// SESSION started, forever, for the whole session - see that field's own
// docs in Renderer3D.hpp). A human-validation GUI pass found the map panel
// still saying "Durum: Yüklendi" even after the current mapping session
// had already logically completed and automatic Return Home had started -
// this enum/derive function exists purely to disambiguate that, matching
// the human-validation brief's own four requested states. Never a new
// mapping/FSM authority of its own, and never mutates
// ExplorationMap/coverage data in any way (see displayedExploredPercentage()
// below for the same guarantee on the companion percentage value) - purely
// a display classification computed fresh each frame from three already-
// computed booleans, exactly like MissionTask.hpp's own
// deriveMissionTask().
enum class MapPanelStatus
{
    NewMap,
    Mapping,
    Loaded,
    Completed
};

// Visual-only, not part of any FSM/RobotState convention - mirrors every
// other small presentation enum in this codebase (MissionTask::toString(),
// DriveAuthority::toString(), ...): an English toString() here, with the
// Turkish presentation text decided by the caller (Renderer3D.cpp), never
// by this header - keeps this file free of any UI-text/locale concern of
// its own.
constexpr std::string_view toString(MapPanelStatus status) noexcept
{
    switch (status)
    {
        case MapPanelStatus::NewMap: return "NewMap";
        case MapPanelStatus::Mapping: return "Mapping";
        case MapPanelStatus::Loaded: return "Loaded";
        case MapPanelStatus::Completed: return "Completed";
    }
    return "Unknown";
}

// Precedence (highest first): Completed > Mapping > Loaded > NewMap -
// logical completion (`logicalExplorationComplete`, main3d.cpp's
// ExplorationCompletionSignal::complete - true once no reachable frontier
// remains, see ExplorationCompletionEventSource.hpp) is the single most
// important fact to surface, so it wins even on the very frame Return Home
// takes over and `explorationActive` (MissionTask::Roam) has already
// flipped false for the same frame - a completed session must never
// regress to reading "Yeni Harita"/"Loaded" the instant Haritalama itself
// stops being the active task.
constexpr MapPanelStatus deriveMapPanelStatus(bool logicalExplorationComplete, bool explorationActive,
                                                bool mapWasLoadedAtStartup) noexcept
{
    if (logicalExplorationComplete)
    {
        return MapPanelStatus::Completed;
    }
    if (explorationActive)
    {
        return MapPanelStatus::Mapping;
    }
    return mapWasLoadedAtStartup ? MapPanelStatus::Loaded : MapPanelStatus::NewMap;
}

// Presentation-only displayed coverage percentage: raw
// ExplorationMap::exploredPercentage() (the caller's own already-computed
// float, passed in - this function never touches ExplorationMap itself,
// so it is structurally incapable of mutating map/coverage data) is only
// ever read, never written - the DISPLAYED number substitutes a clean 100
// once logical completion is true, matching the human-validation brief's
// own explicit "Haritalama: %100 / Durum: Tamamlandı" requirement even
// when raw coverage truthfully stays below 100 forever (interior/occluded
// cells - e.g. directly beneath desk objects - can never be observed by
// any sensor ray, so 100% raw coverage is not a reachable, or even
// meaningful, target). Callers that need the truthful raw number too
// (e.g. the Ayrıntılı HUD's "Ham keşif" line) must read
// ExplorationMap::exploredPercentage()/VisualTelemetry::rawExploredPercentage
// directly - this function only ever answers "what should the primary,
// user-facing number say."
constexpr int displayedExploredPercentage(bool logicalExplorationComplete, float rawExploredPercentage) noexcept
{
    return logicalExplorationComplete ? 100 : static_cast<int>(rawExploredPercentage);
}

} // namespace robot::visual
