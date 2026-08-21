#include "robot/visual/TableEdgeSafetyController.hpp"

#include <cmath>

#include "robot/visual/VisualMath.hpp"

namespace robot::visual
{

void TableEdgeSafetyController::beginRecovery(const RobotPose& pose, const TableSurface& table) noexcept
{
    const float tableCenterX = (table.minX + table.maxX) / 2.0F;
    const float tableCenterZ = (table.minZ + table.maxZ) / 2.0F;
    const float directionX = tableCenterX - pose.position.x;
    const float directionZ = tableCenterZ - pose.position.z;
    // Normalized into [0, 360) - matches RobotPose::headingDegrees'
    // existing representation everywhere else in this codebase
    // (DifferentialDrive.cpp's wrapHeadingDegrees()), rather than
    // headingDegreesFromDirection()'s raw atan2 range of (-180, 180].
    targetRecoveryHeadingDegrees_ = normalizeHeadingDegrees(headingDegreesFromDirection(directionX, directionZ));
}

void TableEdgeSafetyController::update(const CliffSensorReadings& readings, const RobotPose& pose,
                                        const TableSurface& table) noexcept
{
    if (state_ == RecoveryState::Inactive)
    {
        if (readings.anyFrontCliff())
        {
            beginRecovery(pose, table);
            state_ = RecoveryState::BackingAway;
        }
        else if (readings.anyRearCliff())
        {
            beginRecovery(pose, table);
            state_ = RecoveryState::MovingForwardFromRearEdge;
        }
        return;
    }

    // A recovery target exists for the remainder of this function (the
    // Inactive case above always returns) - keep the live heading-error
    // telemetry current every call, not only while actually Turning, so
    // it is meaningful HUD telemetry throughout BackingAway/
    // MovingForwardFromRearEdge too.
    currentHeadingErrorDegrees_ = shortestSignedHeadingErrorDegrees(pose.headingDegrees, targetRecoveryHeadingDegrees_);

    // Two independent conditions (table-edge recovery bugfix #2) - never
    // collapsed into one boolean: headingSafe answers "is the robot
    // pointed the right way," supportSafe answers "is the WHOLE footprint
    // robustly back on the table," a strictly different, position-based
    // question pure rotation cannot answer on its own. Computed once per
    // call since both Turning and AdvancingInward need them.
    const bool headingSafe = std::fabs(currentHeadingErrorDegrees_) <= kRecoveryHeadingToleranceDegrees;
    const bool supportSafe = areAllCornersSafelyInsideTable(pose, table, kRecoverySupportMargin);

    switch (state_)
    {
        case RecoveryState::BackingAway:
            if (!readings.anyFrontCliff())
            {
                state_ = RecoveryState::Turning;
            }
            break;

        case RecoveryState::MovingForwardFromRearEdge:
            if (!readings.anyRearCliff())
            {
                state_ = RecoveryState::Turning;
            }
            break;

        case RecoveryState::Turning:
            // BUGFIX (Phase 13S manual validation): releasing on
            // !readings.anyCliff() alone let Turning exit after a
            // trivial, sometimes-zero rotation on a straight edge - see
            // this class's docs for the full before/after. Heading
            // alignment is now checked first...
            if (headingSafe)
            {
                // BUGFIX #2: ...but heading alignment alone does not mean
                // the robot is done - if the (stricter, margin-based)
                // support check is not yet satisfied, pure rotation
                // cannot fix that (it never translates the center), so
                // hand off to AdvancingInward instead of continuing to
                // spin in place. Only release directly if support is
                // ALSO already fine.
                state_ = supportSafe ? RecoveryState::Inactive : RecoveryState::AdvancingInward;
            }
            // else: remain Turning - recoveryWheelSpeeds() keeps rotating
            // toward the target heading via the shortest-path direction.
            break;

        case RecoveryState::AdvancingInward:
            if (!headingSafe)
            {
                // Heading drifted outside tolerance while driving forward
                // (e.g. the table-support/collision guard rejected a
                // translation on an angled approach, or accumulated
                // float drift) - realign before continuing inward, per
                // the brief's explicit robustness requirement, rather
                // than risk driving inward at a poor angle.
                state_ = RecoveryState::Turning;
            }
            else if (supportSafe)
            {
                state_ = RecoveryState::Inactive;
            }
            // else: remain AdvancingInward - recoveryWheelSpeeds() keeps
            // driving straight forward.
            break;

        case RecoveryState::Inactive:
            break; // Unreachable (handled above) - kept for an exhaustive switch.
    }
}

bool TableEdgeSafetyController::active() const noexcept
{
    return state_ != RecoveryState::Inactive;
}

TableEdgeSafetyController::RecoveryState TableEdgeSafetyController::state() const noexcept
{
    return state_;
}

WheelSpeeds TableEdgeSafetyController::recoveryWheelSpeeds() const noexcept
{
    switch (state_)
    {
        case RecoveryState::Inactive: return WheelSpeeds{0.0F, 0.0F};
        case RecoveryState::BackingAway: return WheelSpeeds{-kRecoveryLinearSpeed, -kRecoveryLinearSpeed};
        case RecoveryState::MovingForwardFromRearEdge: return WheelSpeeds{kRecoveryLinearSpeed, kRecoveryLinearSpeed};
        case RecoveryState::Turning:
        {
            // Shorter-path turn direction (Phase 13S bugfix) - positive
            // error turns the same direction ReactiveObstacleAvoidance
            // always uses (omega > 0); negative error turns the opposite
            // way (omega < 0). See this class's docs for the full
            // rationale.
            const float directionSign = (currentHeadingErrorDegrees_ >= 0.0F) ? 1.0F : -1.0F;
            return WheelSpeeds{-kRecoveryTurnSpeed * directionSign, kRecoveryTurnSpeed * directionSign};
        }
        case RecoveryState::AdvancingInward: return WheelSpeeds{kRecoveryInwardSpeed, kRecoveryInwardSpeed};
    }
    return WheelSpeeds{0.0F, 0.0F};
}

float TableEdgeSafetyController::targetRecoveryHeadingDegrees() const noexcept
{
    return targetRecoveryHeadingDegrees_;
}

float TableEdgeSafetyController::currentHeadingErrorDegrees() const noexcept
{
    return currentHeadingErrorDegrees_;
}

} // namespace robot::visual
