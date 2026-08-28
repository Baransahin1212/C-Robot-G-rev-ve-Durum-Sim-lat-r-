#pragma once

#include <vector>

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Phase 13V: records the actual physical path the robot has travelled -
// raylib-free, no FSM/Event/IRobotHardware/MissionTask knowledge of its
// own. Deliberately position-only sampling (heading is never consulted):
// update() is intended to be called every frame regardless of what is
// currently driving the wheels (FSM/Explore, HomeNavigator/Return Home,
// ReactiveObstacleAvoidance, TableEdgeSafetyController/Safety recovery,
// or manual drive mode - see this phase's own "trail lifecycle" brief),
// so the trail represents physical travel, not "travel while a
// particular task happened to be active." Distance-based sampling (not
// once-per-frame) keeps the point count bounded and keeps turning in
// place from spamming near-duplicate points, since position barely
// changes while only heading does.
class CoverageTrail
{
public:
    // World-unit distance the robot must move (in X/Z) from the last
    // recorded point before a new one is appended - chosen close to
    // ExplorationMap::kCellSizeWorldUnits (0.12F) so the trail's visual
    // resolution roughly matches the map's own grid resolution: dense
    // enough that the drawn path reads as a continuous line at this
    // project's scale, sparse enough that a multi-minute Explore/Return
    // Home session does not accumulate an unbounded number of points.
    static constexpr float kTrailSampleDistanceWorldUnits = 0.15F;

    // Appends `pose.position` if this is the very first update() call
    // (there is no "last point" yet to compare against), or if its
    // distance from the most recently recorded point is >=
    // kTrailSampleDistanceWorldUnits; otherwise a no-op. Only X/Z
    // distance is considered - Y and headingDegrees are never read, so
    // turning in place (position unchanged) never appends a point no
    // matter how many consecutive update() calls occur.
    void update(const RobotPose& pose);

    // Discards every recorded point, resetting to the StartsEmpty state.
    // Phase 13V brief: Stop Task/Start Explore/Return Home/HomeReached/
    // manual mode must never clear the trail - none of those call this.
    // Phase 13Z: New Map (`N`) is the one deliberate exception - see
    // ExplorationSessionReset.hpp's resetExplorationSession(), the first
    // production caller of this method - a fresh mapping session must not
    // keep showing where the OLD map's session travelled.
    void clear() noexcept;

    const std::vector<Vec3>& points() const noexcept;

    // Replaces the entire recorded trail in one call - the bulk-load
    // counterpart to ExplorationMap::setCells(), used only by
    // ExplorationMapStorage::load() to restore a persisted trail.
    void loadPoints(std::vector<Vec3> points);

    // True since the last consumeDirty() call (or construction) if
    // update()/loadPoints() actually appended/replaced points - never set
    // by a no-op update() call. Drives main3d.cpp's periodic-save-when-
    // dirty policy, the same pattern ExplorationMap::consumeDirty() uses.
    bool consumeDirty() noexcept;

private:
    std::vector<Vec3> points_;
    bool dirty_ = false;
};

} // namespace robot::visual
