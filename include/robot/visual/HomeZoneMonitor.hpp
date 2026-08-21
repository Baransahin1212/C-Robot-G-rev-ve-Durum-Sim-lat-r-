#pragma once

#include <optional>

#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"
#include "robot/visual/VirtualWorld.hpp"

namespace robot::visual
{

// Phase 13U: automatic "come home" trigger for Roam - raylib-free,
// implements IPollingEventSource directly (Option A from the brief: its
// own trigger concept is inherently edge-triggered/single-shot, exactly
// like a poll, so a separate wrapper class would only add a file without
// adding clarity - unlike HomeNavigator/HomeArrivalEventSource, which
// stayed split because HomeNavigator's per-frame steering output is
// genuinely useful to callers on its own).
//
// A radius around VirtualWorld::basePlatform()'s own center (read fresh
// every update() call - never a duplicated copy of the base's
// coordinates). Only ever relevant while the current task is Roam
// (`roamActive`, supplied by the caller each frame - this class holds no
// RobotStateMachine/MissionTask knowledge of its own, matching every
// other event source in this codebase).
//
// Hysteresis, not a single threshold, so the boundary can never chatter:
// two named radii, kHomeZoneExitRadius > kHomeZoneRearmRadius, with a
// simple Armed/Disarmed latch between them:
//
//   Armed  -distance > exit radius->  Disarmed (ReturnHomeRequested,
//                                      exactly once)
//   Disarmed  -distance <= rearm radius->  Armed again (no event - only
//                                           re-arms, does not itself
//                                           request anything)
//
// Never spams while remaining outside (Disarmed suppresses further
// triggers until re-armed), and never triggers/tracks anything at all
// while `roamActive` is false - including discarding any not-yet-polled
// pending trigger, since a task change (Stop Task, arrival, a Return Home
// already in progress) makes it stale; see update()'s own docs.
class HomeZoneMonitor : public IPollingEventSource
{
public:
    // Chosen against this project's actual demo geometry (VirtualWorld.cpp),
    // not the brief's own illustrative numbers - see
    // docs/technical-decisions.md (Phase 13U) for the full derivation.
    // The demo robot's own start position is ~7.615F from the demo base
    // (kRobotStartX/Z vs kBaseX/Z) - kHomeZoneExitRadius sits comfortably
    // above that so a freshly started Roam is never immediately outside
    // the zone before it has actually travelled anywhere.
    static constexpr float kHomeZoneExitRadius = 9.0F;

    // Matches VirtualWorld.cpp's own kTableHalfExtent (6.0F) - "back
    // within one table-half-extent of home" is a natural, already-
    // meaningful geometric reference for "safely near base again," and
    // leaves a full 3.0F hysteresis gap below the exit radius.
    static constexpr float kHomeZoneRearmRadius = 6.0F;

    // Advances the monitor by one frame. Returns true exactly on the
    // frame the robot's distance from `base` first crosses from
    // Armed/inside to outside kHomeZoneExitRadius while `roamActive` is
    // true - the same frame a ReturnHomeRequested Event becomes available
    // from pollEvent(). While `roamActive` is false, this call does
    // nothing but discard any not-yet-polled pending trigger (it would
    // otherwise fire once Roam resumes at some unrelated later point,
    // against a task the user may have already cancelled or changed -
    // see docs/technical-decisions.md, Phase 13U) and returns false
    // without touching the armed/disarmed latch, so re-arming resumes
    // exactly where it left off the next time `roamActive` is true.
    bool update(const RobotPose& pose, const BasePlatform& base, bool roamActive) noexcept;

    // True while the monitor is armed (ready to trigger on the next exit-
    // radius crossing) - Visual-simulator-only telemetry/test getter.
    bool armed() const noexcept;

    // Returns the queued ReturnHomeRequested Event from the most recent
    // triggering update() call, or std::nullopt - at most once per
    // trigger, exactly like every other IPollingEventSource in this
    // codebase.
    std::optional<Event> pollEvent() override;

private:
    bool armed_ = true;
    bool pendingEvent_ = false;
};

} // namespace robot::visual
