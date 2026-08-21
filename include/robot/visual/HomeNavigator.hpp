#pragma once

#include <string_view>

#include "robot/visual/DifferentialDrive.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// V1 point-to-point navigation states (Phase 13T) - NOT RobotStateMachine
// states, and never exposed to/consumed by RobotStateMachine/RobotController
// in any way, exactly like TableEdgeSafetyController::RecoveryState's own
// command-vs-effective-actuator distinction. RobotState can legitimately
// still read ReturningHome while this is Aligning/Driving/Arrived.
enum class HomeNavigationState
{
    Inactive,
    Aligning,
    Driving,
    Arrived
};

// Visual-only, not part of any FSM/RobotState convention - mirrors
// TableEdgeSafetyController::RecoveryState's own toString() shape.
constexpr std::string_view toString(HomeNavigationState state) noexcept
{
    switch (state)
    {
        case HomeNavigationState::Inactive: return "Inactive";
        case HomeNavigationState::Aligning: return "Aligning";
        case HomeNavigationState::Driving: return "Driving";
        case HomeNavigationState::Arrived: return "Arrived";
    }
    return "Unknown";
}

// Everything one update() call reports - wheel speeds to apply plus the
// telemetry driving them, so a caller (main3d.cpp, HomeArrivalEventSource)
// never needs to recompute any of this independently.
struct HomeNavigationOutput
{
    WheelSpeeds wheelSpeeds;
    HomeNavigationState state = HomeNavigationState::Inactive;
    float distanceToHome = 0.0F;
    float targetHeadingDegrees = 0.0F;
    float headingErrorDegrees = 0.0F;
    bool arrived = false;
};

// Deterministic, raylib-free, stateful V1 reactive point-to-point
// navigation toward VirtualWorld's BasePlatform (Phase 13T). This is NOT
// path planning - no A*, Dijkstra, occupancy grid, SLAM, waypoint graph,
// or docking vision - it continuously recomputes the direct straight-line
// direction to the base's center from the robot's CURRENT pose every
// update() call (never a cached/stale target, unlike
// TableEdgeSafetyController's fixed-per-incident target heading - see
// docs/technical-decisions.md, Phase 13T, for why continuous recomputation
// is the correct choice here) and steers toward it with a simple align-
// then-drive policy. Has no knowledge of RobotStateMachine, Event,
// IRobotHardware, or RobotController - the caller (main3d.cpp) decides
// *whether* navigation is currently the FSM/controller's intent (via the
// `enabled` argument, driven by VirtualRobotHardware::currentCommand() ==
// VirtualDriveCommand::ReturnToBase) and applies wheelSpeeds via
// VirtualRobotHardware's navigation override API - this class never
// decides on its own that the mission should return home.
class HomeNavigator
{
public:
    // Arrival tolerance, in world units - "reached home" uses a radius,
    // never exact coordinate equality. VirtualWorld's demo BasePlatform is
    // 1.5x1.5 (half-width 0.75F) - 0.40F keeps arrival comfortably inside
    // the platform's own footprint (not triggering from unreasonably far
    // away) while not requiring pixel-perfect centering, roughly on the
    // same scale as the robot's own collision radius
    // (RobotCollision::kRobotCollisionRadius, 0.5F) - "close enough that
    // the robot's body is essentially on the platform."
    static constexpr float kHomeArrivalRadius = 0.40F;

    // Heading hysteresis (Phase 13T brief) - two distinct thresholds so
    // Aligning/Driving cannot rapidly oscillate on a single noisy reading
    // near one boundary. Aligning -> Driving once |error| <= this value;
    // 8.0F is tight enough that "driving" means genuinely pointed at the
    // base, comparable in spirit to TableEdgeSafetyController's own
    // kRecoveryHeadingToleranceDegrees (10.0F) release tolerance.
    static constexpr float kStartDrivingHeadingToleranceDegrees = 8.0F;

    // Driving -> Aligning once |error| >= this value - deliberately
    // larger than kStartDrivingHeadingToleranceDegrees (15.0F > 8.0F) so
    // there is a genuine dead zone between the two thresholds: an error
    // oscillating anywhere in [8, 15) degrees keeps whichever state was
    // already active, instead of flipping every frame a noisy reading
    // crosses one single boundary.
    static constexpr float kStopDrivingHeadingToleranceDegrees = 15.0F;

    // Forward speed while Driving, in world units/second - close to
    // VirtualRobotHardware::kForwardWheelSpeed (1.0F, normal FSM
    // MoveForward speed) but slightly more conservative, since navigation
    // is steering toward a specific target rather than driving blind.
    static constexpr float kNavigationForwardSpeed = 0.8F;

    // In-place turn speed magnitude while Aligning - matches
    // ReactiveObstacleAvoidance::kTurnWheelSpeed/
    // TableEdgeSafetyController::kRecoveryTurnSpeed (both 0.6F) so every
    // "autonomous-authority turning" maneuver in this codebase reads
    // consistently in the HUD/telemetry.
    static constexpr float kNavigationTurnSpeed = 0.6F;

    // Advances the navigator by one frame/step.
    //
    // `enabled` is the caller's signal that FSM/controller intent is
    // currently ReturnToBase/ReturningHome (see this class's own docs
    // above) - when false, the navigator immediately resets to Inactive
    // and reports zero wheel speeds, regardless of what state it was
    // previously in. This is the entire lifecycle contract: enabled
    // becomes false the moment RobotController's mapping stops wanting
    // ReturnToBase (e.g. WaitingForObstacleClear's stop()), and becomes
    // true again once it resumes - naturally resetting/re-arming
    // navigation with NO extra bookkeeping needed by the caller, and
    // naturally recomputing a fresh target heading from wherever the
    // robot ended up (see docs/technical-decisions.md, Phase 13T,
    // "navigation lifecycle").
    //
    // When newly (re-)enabled (state was Inactive): if the robot is
    // already within kHomeArrivalRadius of the base, reports Arrived
    // immediately - it never rotates or drives away first (Phase 13T
    // brief, "base start condition").
    //
    // Otherwise: Aligning rotates in place toward the target heading
    // (shortest-path direction, like TableEdgeSafetyController's own
    // Turning) until the heading-hysteresis start threshold is reached,
    // then Driving drives straight forward until either the stop
    // threshold is exceeded (back to Aligning) or the arrival radius is
    // reached (Arrived).
    HomeNavigationOutput update(const RobotPose& pose, const BasePlatform& base, bool enabled) noexcept;

    // Explicit reset to Inactive - equivalent to the next update() call
    // with enabled=false, exposed directly for callers/tests that want to
    // force a clean state without waiting a frame.
    void reset() noexcept;

    // The state as of the most recent update() call - Inactive before the
    // first call. Exposed so a caller (HomeArrivalEventSource) can sample
    // "did we just arrive" without needing update()'s full return value
    // threaded through separately.
    HomeNavigationState state() const noexcept;

private:
    HomeNavigationState state_ = HomeNavigationState::Inactive;
};

} // namespace robot::visual
