#include "robot/visual/DockApproachController.hpp"

#include <cmath>

#include "robot/visual/RobotCollision.hpp"
#include "robot/visual/VisualMath.hpp"

namespace robot::visual
{

namespace
{
float distanceWorld(const Vec3& a, const Vec3& b) noexcept
{
    const float dx = a.x - b.x;
    const float dz = a.z - b.z;
    return std::sqrt((dx * dx) + (dz * dz));
}

// The staging offset magnitude - shared by computeDockStagingPoint(),
// isDockStagingGeometryValid(), and the reentry-distance check below, so
// none of them can ever independently drift from the others.
float stagingOffsetMagnitude(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    const Vec3 inward = dockInwardDirection(base, tableSurface);
    // inward is always exactly axis-aligned (+-1 on one axis, 0 on the
    // other - see dockInwardDirection()'s own docs), so this reliably
    // picks the platform's depth along whichever axis the dock actually
    // faces, never guessed.
    const float depthAlongAxis = (std::fabs(inward.x) > 0.5F) ? base.size.x : base.size.z;
    return (depthAlongAxis / 2.0F) + kRobotCollisionRadius + kDockStagingMarginWorldUnits;
}
} // namespace

Vec3 computeDockStagingPoint(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    const Vec3 inward = dockInwardDirection(base, tableSurface);
    const float offset = stagingOffsetMagnitude(base, tableSurface);
    return Vec3{base.position.x + (inward.x * offset), base.position.y, base.position.z + (inward.z * offset)};
}

float computeDockReverseHeadingDegrees(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    // Forward heading points the same way `inward` does - toward the desk
    // interior, away from the dock - so reverse (negative) wheel motion
    // drives the robot toward the dock.
    const Vec3 inward = dockInwardDirection(base, tableSurface);
    return normalizeHeadingDegrees(headingDegreesFromDirection(inward.x, inward.z));
}

bool isDockStagingGeometryValid(const BasePlatform& base, const TableSurface& tableSurface) noexcept
{
    const Vec3 stagingPoint = computeDockStagingPoint(base, tableSurface);
    const float offset = stagingOffsetMagnitude(base, tableSurface);
    if (!(offset > 0.0F) || !std::isfinite(offset))
    {
        return false;
    }
    // Comfortably inside the table boundary - at least one full robot
    // collision radius of margin past the staging point itself, so the
    // robot's own footprint (centered there) never hangs off the table
    // edge during AlignForReverse.
    if (stagingPoint.x - tableSurface.minX < kRobotCollisionRadius ||
        tableSurface.maxX - stagingPoint.x < kRobotCollisionRadius ||
        stagingPoint.z - tableSurface.minZ < kRobotCollisionRadius ||
        tableSurface.maxZ - stagingPoint.z < kRobotCollisionRadius)
    {
        return false;
    }
    return true;
}

DockApproachOutput DockApproachController::update(const RobotPose& pose, const BasePlatform& base,
                                                    const TableSurface& tableSurface, bool enabled,
                                                    bool arrivedAtStagingPoint) noexcept
{
    if (!enabled)
    {
        state_ = DockApproachState::Inactive;
        everArrivedAtStaging_ = false;
        reverseFallbackStreak_ = 0;
        precisionCorrectionCount_ = 0;
        inPrecisionZoneLatched_ = false;
        return DockApproachOutput{WheelSpeeds{0.0F, 0.0F}, state_, false, false};
    }

    // Phase 13Y dock-capture LATCH fix - see DockApproachOutput::captured's
    // own docs. `arrivedAtStagingPoint` only ever GRANTS entry here (once
    // latched, never re-checked as a continuation requirement below) -
    // deliberately updated before the Inactive->NavigateToStagingPoint
    // transition and the Failed/geometry checks, so it is never possible
    // for a single frame to both grant entry AND have that exact grant
    // silently discarded by an early return.
    if (arrivedAtStagingPoint)
    {
        everArrivedAtStaging_ = true;
    }

    if (state_ == DockApproachState::Inactive)
    {
        state_ = DockApproachState::NavigateToStagingPoint;
    }

    if (state_ == DockApproachState::Failed)
    {
        return DockApproachOutput{WheelSpeeds{0.0F, 0.0F}, state_, false, false};
    }

    if (!isDockStagingGeometryValid(base, tableSurface))
    {
        state_ = DockApproachState::Failed;
        return DockApproachOutput{WheelSpeeds{0.0F, 0.0F}, state_, false, false};
    }

    const Vec3 stagingPoint = computeDockStagingPoint(base, tableSurface);
    const float reverseHeading = computeDockReverseHeadingDegrees(base, tableSurface);
    const float offsetMagnitude = stagingOffsetMagnitude(base, tableSurface);
    const float reentryDistance = offsetMagnitude * kReentryDistanceMultiplier;

    const float distanceToStaging = distanceWorld(pose.position, stagingPoint);

    // Lateral error off the dock's own centerline - the perpendicular
    // component of (pose.position - base.position) along the dock's fixed
    // lateral axis (the same axis DockChargingContacts.hpp's own contact
    // placement uses - see dockInwardDirection()'s own docs for why this
    // never independently drifts from the contact geometry).
    const Vec3 inward = dockInwardDirection(base, tableSurface);
    const Vec3 lateralAxis = rightDirection(RobotPose{Vec3{}, normalizeHeadingDegrees(headingDegreesFromDirection(
                                                                   -inward.x, -inward.z))});
    const float dxFromBase = pose.position.x - base.position.x;
    const float dzFromBase = pose.position.z - base.position.z;
    const float lateralError = (dxFromBase * lateralAxis.x) + (dzFromBase * lateralAxis.z);

    const DockChargingContactPair dockContacts = computeDockChargingContacts(base, tableSurface);
    const DockChargingContactPair robotContacts = computeRobotRearChargingContacts(pose);

    // Phase 13Y final docking visual-precision polish: per-side contact
    // errors (see DockChargingContactErrors's own docs) - `maxError` both
    // decides the precision zone (see kPrecisionZoneEntryDistanceWorldUnits's
    // own docs) and, together with the tighter final-contact heading/
    // lateral tolerances below, the actual Docked acceptance rule -
    // replacing the older, purely boolean chargingContactsAligned() check
    // for that one decision (chargingContactsAligned() itself is untouched
    // and still used elsewhere/by tests unaffected by this fix).
    const DockChargingContactErrors contactErrors = computeChargingContactErrors(dockContacts, robotContacts);
    // Latched (see inPrecisionZoneLatched_'s own docs) rather than a raw
    // per-frame re-derivation: contactErrors.maxError depends on BOTH
    // position AND heading (robot contacts rotate with heading), so a
    // brief heading wobble mid-correction could otherwise flicker this
    // back to false for a single frame - and since NavigateToStagingPoint's
    // own tightened stopping criterion (effectiveLateralTolerance below)
    // depends on it too, that flicker would let the creep exit
    // IMMEDIATELY (the looser general tolerance already satisfied) without
    // actually correcting anything, a real, traced defect this latch
    // closes.
    if (contactErrors.maxError <= kPrecisionZoneEntryDistanceWorldUnits)
    {
        inPrecisionZoneLatched_ = true;
    }
    const bool inPrecisionZone = inPrecisionZoneLatched_;
    // NavigateToStagingPoint's own precision creep (see that state's own
    // docs) tightens its lateral stopping criterion to
    // kFinalLateralToleranceWorldUnits, rather than the looser
    // kReverseLateralErrorToleranceWorldUnits, once already inside the
    // precision zone - otherwise a precision-zone lateral micro-correction
    // (see ReverseApproach's own docs) would hand off to this state only
    // for it to immediately hand straight back with the SAME still-too-
    // loose lateral error, never actually converging.
    const float effectiveLateralTolerance =
        inPrecisionZone ? kFinalLateralToleranceWorldUnits : kReverseLateralErrorToleranceWorldUnits;

    const float headingErrorToReverse = shortestSignedHeadingErrorDegrees(pose.headingDegrees, reverseHeading);
    const float absHeadingErrorToReverse = std::fabs(headingErrorToReverse);

    // See DockApproachOutput::needsStage1Replan's own docs - set true
    // below on the exact frame this controller hands back to Stage 1
    // because the robot is not precisely enough positioned for pure-
    // rotation AlignForReverse to ever fix.
    bool needsStage1Replan = false;

    if (state_ == DockApproachState::NavigateToStagingPoint)
    {
        // Always align first - contacts/heading are never assumed correct
        // just because entry into this session was granted (Phase 13Y
        // brief: "do not consider docking successful from center-distance
        // alone"). AlignForReverse is PURELY rotational (see that state's
        // own docs) and cannot correct lateral error, so this only advances
        // once the robot is actually close enough to the centerline -
        // lateralError alone (NOT distanceToStaging/the fixed staging point
        // coordinate) is deliberately the right check here: ReverseApproach
        // itself tolerates starting from any DEPTH along the corridor (it
        // simply keeps reversing until contacts align, whatever that takes -
        // see that state's own docs), so requiring the robot to first reach
        // the exact staging point's own fixed depth would be over-
        // constraining. The precision creep below (see its own docs) was
        // fixed to target a DYNAMIC nearest-point-on-centerline instead of
        // the fixed staging point for exactly this reason - see that fix's
        // own docs for the real, traced depth-chasing tug-of-war it closes.
        // Gated on `everArrivedAtStaging_` (the session LATCH - see
        // DockApproachOutput::captured's own docs), never the raw per-frame
        // `arrivedAtStagingPoint` - once entry has ever been granted this
        // session, a momentary flicker of the caller's own entry predicate
        // must never re-block this transition. Uses effectiveLateralTolerance
        // (tightens automatically inside the precision zone - see that
        // variable's own docs) rather than the general tolerance directly.
        if (everArrivedAtStaging_ && std::fabs(lateralError) <= effectiveLateralTolerance)
        {
            state_ = DockApproachState::AlignForReverse;
        }
    }
    else if (state_ == DockApproachState::AlignForReverse || state_ == DockApproachState::ReverseApproach ||
             state_ == DockApproachState::Docked)
    {
        // Phase 13Y (mirrors Phase 13X's own real-GUI-traced fix):
        // contacts/heading are re-evaluated EVERY frame even while
        // already Docked - never a one-way "docked, done forever" latch.
        // A parked robot can legitimately be displaced again (e.g.
        // ordinary obstacle-triggered avoidance firing the instant the
        // suppression below lapses, or a Safety recovery); without this,
        // DockApproachController would freeze permanently the moment
        // anything ever moved the robot away from an already-docked
        // position, producing zero wheels forever and never re-docking.
        // Phase 13Y final docking visual-precision polish: Docked now
        // requires the TIGHTER final-contact tolerances (kFinalContact
        // HeadingToleranceDegrees/kFinalLateralToleranceWorldUnits), never
        // merely the general reverse-entry ones - a visually diagonal or
        // laterally-offset "close enough by the old, looser check" pose is
        // no longer accepted. Both individual contact errors (never merely
        // their max, and never center-distance alone) must each be within
        // tolerance too.
        if (contactErrors.leftError <= kContactAlignmentToleranceWorldUnits &&
            contactErrors.rightError <= kContactAlignmentToleranceWorldUnits &&
            absHeadingErrorToReverse <= kFinalContactHeadingToleranceDegrees &&
            std::fabs(lateralError) <= kFinalLateralToleranceWorldUnits)
        {
            state_ = DockApproachState::Docked;
            reverseFallbackStreak_ = 0;
            precisionCorrectionCount_ = 0;
        }
        else if (distanceToStaging > reentryDistance)
        {
            // Displaced too far from the known-safe lane (e.g. a Safety
            // recovery mid-approach) - hand back to Stage 1 rather than
            // continuing to chase a fixed-heading lane that may no longer
            // pass through the robot's current position.
            state_ = DockApproachState::NavigateToStagingPoint;
            needsStage1Replan = true;
            // Phase 13Y dock-capture LATCH fix: this is the one genuine
            // "explicit safety-driven restart" the session latch (see
            // DockApproachOutput::captured's own docs) must still release
            // on - a big displacement can legitimately move the robot far
            // from the dock entirely (e.g. mid-exploration avoidance
            // detours after a Safety recovery), and the latch must not
            // keep dock-hazard suppression pinned active somewhere
            // completely unrelated to the dock for the rest of the
            // session. Clearing it here requires a FRESH, re-validated
            // entry (WaypointNavigatorState::Arrived or a fresh
            // DockCaptureRegion::isDockCaptureEligible() pass) before this
            // controller resumes driving - never assumed still valid from
            // however long ago entry was first granted.
            everArrivedAtStaging_ = false;
            reverseFallbackStreak_ = 0;
            precisionCorrectionCount_ = 0;
            inPrecisionZoneLatched_ = false;
        }
        else if (state_ == DockApproachState::Docked)
        {
            // Just displaced out of contact alignment while still in the
            // lane - always re-verify heading before resuming, never
            // straight back into ReverseApproach blindly.
            state_ = DockApproachState::AlignForReverse;
        }
        else if (state_ == DockApproachState::AlignForReverse)
        {
            if (absHeadingErrorToReverse <= kReverseDockHeadingToleranceDegrees)
            {
                state_ = DockApproachState::ReverseApproach;
            }
        }
        else // ReverseApproach
        {
            // Phase 13Y final docking visual-precision polish: once inside
            // the precision zone (see kPrecisionZoneEntryDistanceWorldUnits's
            // own docs), the TIGHTER final-contact tolerances govern the
            // fallback decision instead of the general reverse-approach
            // ones below - "do not continue reversing while visibly
            // diagonal" (this fix's own brief). Checked FIRST/separately
            // from the general fallback, with its OWN bounded-retry counter
            // (precisionCorrectionCount_) and a harsher terminal outcome
            // (Failed, not a hand-back to Stage 1) once exhausted - a
            // precision-zone drift that cannot be corrected after this many
            // tries signals a genuinely unrecoverable final geometry, not a
            // transient one Stage 1 could fix by re-approaching.
            const bool precisionHeadingExceeded = inPrecisionZone && absHeadingErrorToReverse > kFinalContactHeadingToleranceDegrees;
            const bool precisionLateralExceeded = inPrecisionZone && std::fabs(lateralError) > kFinalLateralToleranceWorldUnits;
            if (precisionHeadingExceeded || precisionLateralExceeded)
            {
                ++precisionCorrectionCount_;
                if (precisionCorrectionCount_ > kMaxPrecisionDockRetries)
                {
                    state_ = DockApproachState::Failed;
                }
                else if (precisionLateralExceeded)
                {
                    // Deterministic micro-correction, lateral case: a pure
                    // straight reverse never changes lateral offset (the
                    // same fact kReverseLateralErrorToleranceWorldUnits's
                    // own docs establish), so AlignForReverse's rotate-only
                    // behavior CANNOT correct this by itself - a real,
                    // traced defect an earlier version of this exact fix
                    // hit (perpetually cycling AlignForReverse<->
                    // ReverseApproach at a fixed residual lateral offset,
                    // never converging, until this bounded-retry counter
                    // gave up). Falls back to NavigateToStagingPoint
                    // instead, whose own precision creep (see that state's
                    // own docs) drives a short FORWARD correction toward
                    // the nearest point on the centerline at the robot's
                    // CURRENT depth - this fix's own brief's "move slightly
                    // forward out of the contact zone... re-align" step,
                    // reusing the SAME existing mechanism rather than a new
                    // one. Never a teleport, never random - a small,
                    // deterministic, bounded correction.
                    state_ = DockApproachState::NavigateToStagingPoint;
                }
                else
                {
                    // Deterministic micro-correction, heading-only case:
                    // AlignForReverse's own existing rotate-in-place
                    // behavior (now at kPrecisionAligningTurnSpeed - see
                    // that constant's own docs) already corrects this
                    // without ever committing to a curved/forward maneuver
                    // of its own - consistent with this class's own "no
                    // complicated curved reverse parking" V1 simplification.
                    state_ = DockApproachState::AlignForReverse;
                }
            }
            else if (absHeadingErrorToReverse >= kReverseDockHeadingReleaseToleranceDegrees ||
                std::fabs(lateralError) > kReverseLateralErrorToleranceWorldUnits)
            {
                // Falls back to AlignForReverse to zero the heading out to
                // the tighter kReverseDockHeadingToleranceDegrees before
                // resuming (a genuine heading release), or because lateral
                // error crept past tolerance while heading itself was still
                // within the looser release tolerance - gradual drift
                // accumulated from a small residual heading error over the
                // reverse distance covered so far (a true large/systemic
                // offset is caught by NavigateToStagingPoint's own
                // precision creep before AlignForReverse/ReverseApproach
                // ever begin - see that state's own docs). Per this
                // controller's own class docs, "no complicated curved
                // reverse parking," a deliberate V1 simplification (never a
                // curved in-lane correction of its own).
                //
                // Phase 13Y dock-capture LATCH fix (real-GUI-traced): a
                // real, traced integration run showed this exact fallback
                // can deterministically repeat forever without ever
                // resolving - re-aligning heading to the SAME value each
                // time, drifting laterally the SAME small amount each
                // ReverseApproach attempt, in a fixed cycle - once the
                // robot has settled right at the dock housing's own
                // physical collision boundary (a real position a pure
                // rotate-then-straight-reverse maneuver can reach without
                // ever colliding outright, yet never quite satisfying
                // contact alignment either). kMaxReverseFallbackStreak
                // consecutive fallbacks without ever reaching Docked is
                // this controller's own equivalent of
                // NavigationProgressTracker's "stuck" detection (see that
                // class's own docs) - releases the session latch and hands
                // back to Stage 1 for a fresh, re-validated precision
                // re-approach, exactly like the big-displacement reentry
                // case above, rather than cycling in place indefinitely.
                ++reverseFallbackStreak_;
                if (reverseFallbackStreak_ > kMaxReverseFallbackStreak)
                {
                    state_ = DockApproachState::NavigateToStagingPoint;
                    needsStage1Replan = true;
                    everArrivedAtStaging_ = false;
                    reverseFallbackStreak_ = 0;
                    inPrecisionZoneLatched_ = false;
                }
                else
                {
                    state_ = DockApproachState::AlignForReverse;
                }
            }
        }
    }

    WheelSpeeds speeds{0.0F, 0.0F};
    bool driving = false;
    switch (state_)
    {
        case DockApproachState::AlignForReverse:
        {
            // Slows to kPrecisionAligningTurnSpeed once inside the
            // precision zone (see that constant's own docs for why this is
            // a real necessity, not merely stylistic - kAligningTurnSpeed's
            // own per-frame step would otherwise reliably overshoot the
            // much narrower final-contact heading window every correction
            // attempt).
            const float turnSpeed = inPrecisionZone ? kPrecisionAligningTurnSpeed : kAligningTurnSpeed;
            const float directionSign = (headingErrorToReverse >= 0.0F) ? 1.0F : -1.0F;
            speeds = WheelSpeeds{-turnSpeed * directionSign, turnSpeed * directionSign};
            driving = true;
            break;
        }
        case DockApproachState::ReverseApproach:
        {
            // Equal NEGATIVE wheel speeds - physically drives the robot
            // backward through DifferentialDrive, never a faked/pose-set
            // reverse (Phase 13Y brief: "must physically drive the robot
            // backward"). Slows to kPrecisionReverseDockSpeed once inside
            // the precision zone (see that constant's own docs) - the
            // final, most delicate stretch gets the most deliberate speed.
            const float reverseSpeed = inPrecisionZone ? kPrecisionReverseDockSpeed : kReverseDockSpeed;
            speeds = WheelSpeeds{-reverseSpeed, -reverseSpeed};
            driving = true;
            break;
        }
        case DockApproachState::NavigateToStagingPoint:
        {
            // See this state's own header docs for why this precision
            // creep exists: Stage 1 (WaypointNavigator/HomeNavigator) hard-
            // stops at its own much looser arrival radius (0.40) in
            // whatever direction it was approaching from, which says
            // nothing about lateral (centerline) position. Only engages
            // once entry has ever been granted this session
            // (`everArrivedAtStaging_` - see DockApproachOutput::captured's
            // own docs, never the raw per-frame `arrivedAtStagingPoint`)
            // AND the robot is still off the centerline (effectiveLateralTolerance -
            // tightens automatically inside the precision zone, see that
            // variable's own docs) - otherwise Stage 1 is still driving and
            // this controller stays silent (zero speeds, driving=false)
            // exactly as before.
            if (everArrivedAtStaging_ && std::fabs(lateralError) > effectiveLateralTolerance)
            {
                // Phase 13Y dock-capture LATCH fix (real-GUI-traced): drives
                // toward the NEAREST point on the dock's own centerline to
                // the robot's CURRENT position - i.e. a pure lateral
                // correction, exactly zeroing `lateralError` and nothing
                // else - rather than the earlier design's fixed
                // computeDockStagingPoint() coordinate. A real, traced
                // integration run showed the fixed-point version could
                // deadlock once `everArrivedAtStaging_` made the
                // NavigateToStagingPoint<->AlignForReverse/ReverseApproach
                // cycle reachable from ANY depth along the corridor, not
                // merely from wherever Stage 1's own arrival first left it:
                // ReverseApproach's own fallback (see that state's own
                // .cpp docs) can push the robot CLOSER to the dock (deeper)
                // than the fixed staging point sits, and driving all the
                // way back OUT to that fixed point every time (only to
                // reverse right back in) produced a genuine, observed
                // tug-of-war that never converged. Since ReverseApproach
                // itself tolerates starting from any depth (it simply
                // keeps reversing until contacts align), correcting ONLY
                // the lateral component here - never re-chasing a fixed
                // depth - is both sufficient and exactly what removes the
                // tug-of-war.
                const Vec3 lateralCorrectionTarget{pose.position.x - (lateralError * lateralAxis.x), pose.position.y,
                                                    pose.position.z - (lateralError * lateralAxis.z)};
                const float dxToTarget = lateralCorrectionTarget.x - pose.position.x;
                const float dzToTarget = lateralCorrectionTarget.z - pose.position.z;
                const float distanceToTarget = std::sqrt((dxToTarget * dxToTarget) + (dzToTarget * dzToTarget));
                // Guards the heading computation below - lateralError could
                // not exceed tolerance at true zero distance, so this only
                // ever skips a degenerate near-zero-distance edge case.
                if (distanceToTarget > 0.001F)
                {
                    const float headingToTarget =
                        normalizeHeadingDegrees(headingDegreesFromDirection(dxToTarget, dzToTarget));
                    const float headingErrorToTarget =
                        shortestSignedHeadingErrorDegrees(pose.headingDegrees, headingToTarget);
                    if (std::fabs(headingErrorToTarget) > kStagingApproachHeadingToleranceDegrees)
                    {
                        const float directionSign = (headingErrorToTarget >= 0.0F) ? 1.0F : -1.0F;
                        speeds =
                            WheelSpeeds{-kAligningTurnSpeed * directionSign, kAligningTurnSpeed * directionSign};
                    }
                    else
                    {
                        speeds = WheelSpeeds{kStagingApproachSpeed, kStagingApproachSpeed};
                    }
                    driving = true;
                }
            }
            break;
        }
        case DockApproachState::Inactive:
        case DockApproachState::Docked:
        case DockApproachState::Failed:
            speeds = WheelSpeeds{0.0F, 0.0F};
            driving = false;
            break;
    }

    return DockApproachOutput{speeds, state_, state_ == DockApproachState::Docked, driving, needsStage1Replan,
                               everArrivedAtStaging_};
}

void DockApproachController::reset() noexcept
{
    state_ = DockApproachState::Inactive;
    everArrivedAtStaging_ = false;
    reverseFallbackStreak_ = 0;
    precisionCorrectionCount_ = 0;
    inPrecisionZoneLatched_ = false;
}

DockApproachState DockApproachController::state() const noexcept
{
    return state_;
}

} // namespace robot::visual
