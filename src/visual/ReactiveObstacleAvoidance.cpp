#include "robot/visual/ReactiveObstacleAvoidance.hpp"

#include <cmath>

namespace robot::visual
{

const float ReactiveObstacleAvoidance::kMinimumBypassDistanceWorldUnits = 2.0F * kRobotCollisionRadius;

namespace
{
float distance(const Vec3& a, const Vec3& b) noexcept
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dz * dz);
}
} // namespace

// Chooses which way to turn for a NEW incident: turning away from
// whichever side reports a CLOSER (smaller-distance) obstacle steers the
// robot's body away from the tighter hazard first. Left closer than right
// -> turn right (positive sign, matching the class' documented
// {-k,+k}*sign wheel convention: positive sign spins the same direction
// as Phase 13R's original single fixed turn). Right closer than left ->
// turn left (negative sign). Equal, center-only, or no side information at
// all (both left/right absent) -> deterministic default of +1.0F,
// preserving Phase 13R's original always-turn-this-way behavior exactly
// for the symmetric/no-information case, so existing single-obstacle-
// dead-ahead scenarios keep behaving identically to before this fix.
float ReactiveObstacleAvoidance::chooseTurnSign(const ObstacleHazardSample& hazard) const noexcept
{
    if (hazard.leftDistance.has_value() && hazard.rightDistance.has_value())
    {
        if (*hazard.leftDistance < *hazard.rightDistance)
        {
            return 1.0F;
        }
        if (*hazard.rightDistance < *hazard.leftDistance)
        {
            return -1.0F;
        }
        return 1.0F;
    }
    if (hazard.leftDistance.has_value())
    {
        return 1.0F;
    }
    if (hazard.rightDistance.has_value())
    {
        return -1.0F;
    }
    return 1.0F;
}

void ReactiveObstacleAvoidance::update(bool enabled, bool triggerAvoidance, bool forwardCorridorClear,
                                        const RobotPose& pose, const ObstacleHazardSample& hazard) noexcept
{
    if (!enabled)
    {
        state_ = AvoidanceState::Inactive;
        return;
    }

    if (state_ == AvoidanceState::Inactive)
    {
        if (triggerAvoidance)
        {
            state_ = AvoidanceState::TurnAway;
            latchedTurnSign_ = chooseTurnSign(hazard);
        }
        return;
    }

    if (state_ == AvoidanceState::TurnAway)
    {
        if (forwardCorridorClear)
        {
            state_ = AvoidanceState::AdvanceClear;
            advanceStartPosition_ = pose.position;
        }
        return;
    }

    // state_ == AdvanceClear
    if (!forwardCorridorClear)
    {
        state_ = AvoidanceState::TurnAway;
        return;
    }

    const float travelled = distance(pose.position, advanceStartPosition_);
    if (travelled >= kMinimumBypassDistanceWorldUnits)
    {
        state_ = AvoidanceState::Inactive;
    }
}

bool ReactiveObstacleAvoidance::active() const noexcept
{
    return state_ != AvoidanceState::Inactive;
}

AvoidanceState ReactiveObstacleAvoidance::state() const noexcept
{
    return state_;
}

WheelSpeeds ReactiveObstacleAvoidance::wheelSpeeds() const noexcept
{
    switch (state_)
    {
        case AvoidanceState::TurnAway:
            return WheelSpeeds{-kTurnWheelSpeed * latchedTurnSign_, kTurnWheelSpeed * latchedTurnSign_};
        case AvoidanceState::AdvanceClear:
            return WheelSpeeds{kAdvanceWheelSpeed, kAdvanceWheelSpeed};
        case AvoidanceState::Inactive:
        default:
            return WheelSpeeds{0.0F, 0.0F};
    }
}

} // namespace robot::visual
