#include "robot/visual/TableEdgeSafetyController.hpp"

#include <cmath>

#include "robot/visual/VisualMath.hpp"

namespace robot::visual
{

namespace
{

// Table-edge recovery bugfix #3: false only when there IS a genuine
// aggregate overhang right now (currentOverhang > 0 - if the footprint is
// already fully supported, translating further is never itself unsafe,
// regardless of direction) AND one more kSupportCheckLookaheadDistance-
// sized step in `travelDirection` (a unit vector - reverse of
// forwardDirection() for BackingAway, forwardDirection() itself for
// MovingForwardFromRearEdge/AdvancingInward) would not shrink it. False
// means continuing to translate this way is not addressing the actual
// problem (the triggering overhang is on an axis this direction barely
// moves the robot along, or this direction is actively making it worse) -
// the caller should reorient toward the target heading instead of
// blindly continuing. Deliberately NOT `projected < current` alone: at
// exactly zero overhang that comparison is always false (0 is never less
// than 0), which would wrongly block perfectly safe forward motion once
// the footprint is already fully supported but the (separate, margin-
// based) release condition has not yet fired.
bool translatingWouldNotHelp(const RobotPose& pose, const TableSurface& table, const Vec3& travelDirection) noexcept
{
    const float currentOverhang = aggregateTableOverhang(pose, table);
    if (currentOverhang <= 0.0F)
    {
        return false;
    }
    const RobotPose projectedPose{
        Vec3{pose.position.x + (travelDirection.x * TableEdgeSafetyController::kSupportCheckLookaheadDistance),
             pose.position.y,
             pose.position.z + (travelDirection.z * TableEdgeSafetyController::kSupportCheckLookaheadDistance)},
        pose.headingDegrees};
    return aggregateTableOverhang(projectedPose, table) >= currentOverhang;
}

// Phase 13X blocker fix: plain 2D (x/z) world-space distance between two
// positions - used only by the new bounded recovery-stall check below
// (kMaxRecoveryStallFrames's own docs), which needs the robot's ACTUAL
// committed displacement between calls, never a hypothetical projection
// (that is what translatingWouldNotHelp() above already covers).
float distanceWorld(const Vec3& a, const Vec3& b) noexcept
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dz * dz));
}

} // namespace

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
    recoveryBlockedThisUpdate_ = false;

    if (state_ == RecoveryState::Inactive)
    {
        if (readings.anyFrontCliff())
        {
            beginRecovery(pose, table);
            state_ = RecoveryState::BackingAway;
            stallAnchorPosition_ = pose.position;
            framesSinceStallProgress_ = 0;
        }
        else if (readings.anyRearCliff())
        {
            beginRecovery(pose, table);
            state_ = RecoveryState::MovingForwardFromRearEdge;
            stallAnchorPosition_ = pose.position;
            framesSinceStallProgress_ = 0;
        }
        return;
    }

    // Phase 13X blocker fix: bounded recovery-stall check - see
    // kMaxRecoveryStallFrames's own docs. Only the three TRANSLATING
    // states can stall this way (Turning never translates the robot's
    // center at all, so "position frozen" is its own normal operation,
    // not a defect). Checked BEFORE the transition switch below, using
    // the ACTUAL pose this call was handed - if it fires, this call is
    // done: the switch below is skipped entirely (state_ is already
    // Inactive) and the caller sees recoveryBlockedThisUpdate() true this
    // one time.
    if (state_ == RecoveryState::BackingAway || state_ == RecoveryState::MovingForwardFromRearEdge ||
        state_ == RecoveryState::AdvancingInward)
    {
        if (distanceWorld(pose.position, stallAnchorPosition_) > kStallProgressEpsilon)
        {
            stallAnchorPosition_ = pose.position;
            framesSinceStallProgress_ = 0;
        }
        else
        {
            ++framesSinceStallProgress_;
            if (framesSinceStallProgress_ >= kMaxRecoveryStallFrames)
            {
                state_ = RecoveryState::Inactive;
                recoveryBlockedThisUpdate_ = true;
                return;
            }
        }
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
            else if (translatingWouldNotHelp(pose, table, Vec3{-forwardDirection(pose).x, 0.0F, -forwardDirection(pose).z}))
            {
                // Bugfix #3: backing away further is not actually helping
                // (the triggering overhang is on an axis this reverse
                // direction barely moves along, or is being made worse) -
                // reorient toward the known-safe target instead of
                // continuing to drive further off table.
                state_ = RecoveryState::Turning;
            }
            break;

        case RecoveryState::MovingForwardFromRearEdge:
            if (!readings.anyRearCliff())
            {
                state_ = RecoveryState::Turning;
            }
            else if (translatingWouldNotHelp(pose, table, forwardDirection(pose)))
            {
                // Bugfix #3: same proposed-motion safety check, mirrored
                // for the forward-away-from-a-rear-edge direction.
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
                if (state_ == RecoveryState::AdvancingInward)
                {
                    // Fresh stall window for the newly-entered translating
                    // state (Phase 13X blocker fix) - never inherits
                    // BackingAway/MovingForwardFromRearEdge's own counter,
                    // which may already have been close to its bound.
                    stallAnchorPosition_ = pose.position;
                    framesSinceStallProgress_ = 0;
                }
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
            else if (translatingWouldNotHelp(pose, table, forwardDirection(pose)))
            {
                // Bugfix #3: heading reads "safe" (within tolerance of
                // the target), but one more forward step would not
                // actually reduce the aggregate overhang - the recovery
                // target itself may be stale/imprecise this close to the
                // tolerance boundary. Reorient (Turning recomputes the
                // shortest-path direction toward the same fixed target
                // every call) rather than keep driving a direction that
                // is not helping.
                state_ = RecoveryState::Turning;
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

bool TableEdgeSafetyController::recoveryBlockedThisUpdate() const noexcept
{
    return recoveryBlockedThisUpdate_;
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
