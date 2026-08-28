#include "robot/visual/ExplorationSessionReset.hpp"

#include <limits>

#include "robot/visual/ExplorationMapStorage.hpp"

namespace robot::visual
{

void resetExplorationSession(const std::string& mapStoragePath, bool mapPersistenceAvailable,
                              ExplorationSessionState& session)
{
    if (mapPersistenceAvailable)
    {
        ExplorationMapStorage::remove(mapStoragePath);
    }

    // Fresh, all-Unknown grid at the map's own current bounds/resolution
    // (never a second, hardcoded derivation) - replaces `session.map`'s
    // CONTENTS in place via setCells(), never the object itself, so any
    // existing reference into it stays valid (see this function's own
    // header docs).
    const ExplorationMap freshMap(session.map.bounds());
    session.map.setCells(freshMap.cells());

    session.trail.clear();
    session.navigator.reset();
    session.completionSignal.complete = false;

    session.currentFrontierTarget.reset();
    session.frontierBlacklist.clear();
    session.framesSinceLastFrontierAttempt = 0;
    session.consecutiveNoTargetFound = 0;

    session.returnHomeBlockedCells.clear();
    session.previousNavOutput = WaypointNavigatorOutput{};
    session.bestDistanceToHomeThisSession = std::numeric_limits<float>::infinity();
    session.framesSinceDistanceImproved = 0;

    session.mapAlreadyCompleteNoticeActive = false;
    session.mapSaveTimer = 0.0F;
    session.mapWasLoadedAtStartup = false;
}

} // namespace robot::visual
