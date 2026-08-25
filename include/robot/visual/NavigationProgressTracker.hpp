#pragma once

#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Deterministic, raylib-free, stateful stuck/no-progress detector (Phase
// 13X) - the anti-360-spin guarantee's own bookkeeping, extracted as a
// standalone, independently testable component rather than inline state
// inside WaypointNavigator. Advanced once per simulated frame (like every
// other stateful component in this codebase - update() is never tied to
// wall-clock time), it tracks two independent signals since the last
// reset()/construction:
//   - net positional displacement from the pose at that reset ("has the
//     robot actually gone anywhere")
//   - accumulated ABSOLUTE heading change ("how much has it rotated, with
//     NO cancellation between opposite turns" - two 180-degree turns in
//     opposite directions still add to 360 degrees of accumulated
//     rotation, they never net to zero)
//
// isStuck() reports true only once BOTH:
//   - at least kStuckSampleWindow update() calls have happened since the
//     last reset(), AND
//   - accumulated absolute rotation exceeds kStuckRotationThresholdDegrees
//     while net displacement has stayed below
//     kStuckDisplacementThreshold the whole time
//
// - i.e. exactly the human-observed Return Home defect's own signature
// (repeated large rotation, no net translation), never merely "hasn't
// moved in N frames" (a robot correctly Aligning in place before Driving
// is not stuck) and never merely "is rotating" (turning while also
// translating is completely normal navigation, not spinning). See
// docs/technical-decisions.md (Phase 13X, "anti-spin / progress
// guarantee") for the full derivation and the regression test this exists
// to make pass.
class NavigationProgressTracker
{
public:
    // Below this net displacement (world units) from the anchor pose, the
    // robot is considered to have made no meaningful positional progress.
    static constexpr float kStuckDisplacementThreshold = 0.10F;

    // Above this accumulated ABSOLUTE heading change (degrees), the robot
    // has rotated at least two full turns without translating -
    // deliberately a large multiple of 360 (never a single turn, which a
    // legitimate Aligning phase can genuinely reach) so a real Aligning-
    // then-Driving sequence is never mistaken for spinning.
    static constexpr float kStuckRotationThresholdDegrees = 720.0F;

    // Minimum number of update() calls since the last reset() before
    // isStuck() can report true - avoids a false positive from a single
    // large heading jump immediately after reset() (e.g. the very first
    // frame of a fresh Aligning phase).
    static constexpr int kStuckSampleWindow = 60;

    // Advances the tracker by one frame - call exactly once per frame
    // with the robot's current pose, only while actively navigating
    // toward a goal (the caller decides when tracking is meaningful; this
    // class has no notion of navigation state itself).
    void update(const RobotPose& pose) noexcept;

    // Re-anchors to the NEXT update() call's pose and clears every
    // accumulator - callers invoke this whenever a legitimate reason for
    // the window to restart occurs (a new waypoint/goal, a fresh route,
    // or immediately after acting on a true isStuck() reading), so a
    // single detection is never re-reported forever without the tracker
    // getting a fresh chance to prove progress.
    void reset() noexcept;

    bool isStuck() const noexcept;

private:
    bool anchored_ = false;
    Vec3 anchorPosition_{};
    Vec3 latestPosition_{};
    float previousHeadingDegrees_ = 0.0F;
    float accumulatedAbsoluteRotationDegrees_ = 0.0F;
    int samplesSinceAnchor_ = 0;
};

} // namespace robot::visual
