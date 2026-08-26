#include "robot/visual/WaypointNavigator.hpp"

#include <cmath>

#include "robot/visual/GridPathPlanner.hpp"

namespace robot::visual
{

namespace
{
// Below this world-unit distance, two goal points are considered "the
// same goal" - avoids treating floating-point jitter in a caller-supplied
// goal (e.g. a frontier target re-evaluated with a slightly different
// float each call) as a materially new goal requiring a fresh plan.
constexpr float kGoalChangeEpsilonWorldUnits = 0.02F;

float distanceWorld(const Vec3& a, const Vec3& b) noexcept
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dz * dz));
}
} // namespace

WaypointNavigatorOutput WaypointNavigator::update(const RobotPose& pose, const ExplorationMap& map,
                                                    const Vec3& goalWorld, bool enabled, bool forceReplan,
                                                    const std::vector<GridCoord>& extraBlockedCells) noexcept
{
    if (!enabled)
    {
        reset();
        return WaypointNavigatorOutput{WheelSpeeds{0.0F, 0.0F}, state_, route_, waypointIndex_, false, false};
    }

    const bool freshStart = (state_ == WaypointNavigatorState::Inactive);
    const bool goalChanged = !hasLastGoal_ || distanceWorld(goalWorld, lastGoal_) > kGoalChangeEpsilonWorldUnits;
    bool needsReplan = freshStart || goalChanged || forceReplan || (state_ == WaypointNavigatorState::Failed);

    if (!needsReplan && state_ == WaypointNavigatorState::Following)
    {
        if (!routeStillClear(map))
        {
            needsReplan = true;
        }
        else
        {
            progressTracker_.update(pose);
            if (progressTracker_.isStuck())
            {
                needsReplan = true;
                progressTracker_.reset();
            }
        }
    }

    if (needsReplan)
    {
        planRoute(pose, map, goalWorld, extraBlockedCells);
        lastGoal_ = goalWorld;
        hasLastGoal_ = true;
        progressTracker_.reset();
    }

    if (state_ == WaypointNavigatorState::Failed)
    {
        return WaypointNavigatorOutput{WheelSpeeds{0.0F, 0.0F}, state_, route_, waypointIndex_, false, true};
    }

    // Drive toward the current waypoint, advancing through any waypoints
    // reached this same frame (bounded by route_.size() so a degenerate
    // zero-distance waypoint sequence can never loop forever).
    for (std::size_t guard = 0; guard <= route_.size(); ++guard)
    {
        const BasePlatform syntheticTarget{route_[waypointIndex_], Vec3{}};
        const HomeNavigationOutput localOutput = localSteering_.update(pose, syntheticTarget, true);

        if (!localOutput.arrived)
        {
            state_ = WaypointNavigatorState::Following;
            return WaypointNavigatorOutput{localOutput.wheelSpeeds, state_, route_, waypointIndex_, false, false};
        }

        if (waypointIndex_ + 1 >= route_.size())
        {
            state_ = WaypointNavigatorState::Arrived;
            return WaypointNavigatorOutput{WheelSpeeds{0.0F, 0.0F}, state_, route_, waypointIndex_, true, false};
        }

        ++waypointIndex_;
        localSteering_.reset();
    }

    // Unreachable in practice (the loop above always returns before
    // exhausting its guard bound for any non-empty route_), but keeps this
    // function total.
    state_ = WaypointNavigatorState::Arrived;
    return WaypointNavigatorOutput{WheelSpeeds{0.0F, 0.0F}, state_, route_, waypointIndex_, true, false};
}

void WaypointNavigator::reset() noexcept
{
    state_ = WaypointNavigatorState::Inactive;
    route_.clear();
    waypointIndex_ = 0;
    hasLastGoal_ = false;
    localSteering_.reset();
    progressTracker_.reset();
}

WaypointNavigatorState WaypointNavigator::state() const noexcept
{
    return state_;
}

const std::vector<Vec3>& WaypointNavigator::currentRoute() const noexcept
{
    return route_;
}

void WaypointNavigator::planRoute(const RobotPose& pose, const ExplorationMap& map, const Vec3& goalWorld,
                                   const std::vector<GridCoord>& extraBlockedCells)
{
    const GridPathPlanner planner(map, extraBlockedCells);
    const PathPlanResult result = planner.planPath(pose.position, goalWorld);

    localSteering_.reset();
    waypointIndex_ = 0;

    if (!result.success || result.waypoints.empty())
    {
        state_ = WaypointNavigatorState::Failed;
        route_.clear();
        return;
    }

    route_ = result.waypoints;
    state_ = WaypointNavigatorState::Following;
}

namespace
{
// Samples along a straight world-space segment at roughly one
// ExplorationMap cell per step - checks the actual physical corridor the
// robot is about to drive, not merely the (sparse, simplified) waypoint
// vertices themselves, so a newly-discovered Occupied cell anywhere along
// a long straight leg is caught immediately, not only once the robot's
// own NavigationProgressTracker eventually notices a lack of progress.
bool segmentStillClear(const ExplorationMap& map, const Vec3& from, const Vec3& to)
{
    const float dx = to.x - from.x;
    const float dz = to.z - from.z;
    const float length = std::sqrt((dx * dx) + (dz * dz));
    const int steps = std::max(1, static_cast<int>(std::ceil(length / map.cellSize())));
    for (int step = 0; step <= steps; ++step)
    {
        const float t = static_cast<float>(step) / static_cast<float>(steps);
        const Vec3 sample{from.x + (dx * t), 0.0F, from.z + (dz * t)};
        int col = -1;
        int row = -1;
        if (map.worldToCell(sample, col, row) && map.cellAt(col, row) == MapCell::Occupied)
        {
            return false;
        }
    }
    return true;
}
} // namespace

bool WaypointNavigator::routeStillClear(const ExplorationMap& map) const
{
    if (route_.empty())
    {
        return true;
    }
    // Starts one segment BEFORE waypointIndex_ (when one exists) so the
    // leg currently being driven - from the previous waypoint into the
    // current one - is checked too, not only legs strictly ahead of it.
    const std::size_t startIndex = (waypointIndex_ > 0) ? waypointIndex_ - 1 : 0;
    for (std::size_t i = startIndex; i + 1 < route_.size(); ++i)
    {
        if (!segmentStillClear(map, route_[i], route_[i + 1]))
        {
            return false;
        }
    }
    return true;
}

} // namespace robot::visual
