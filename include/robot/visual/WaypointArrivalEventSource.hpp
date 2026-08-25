#pragma once

#include <optional>

#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"
#include "robot/visual/WaypointNavigator.hpp"

namespace robot::visual
{

// Turns WaypointNavigator's own Arrived state into exactly one HomeReached
// Event per arrival (Phase 13X) - edge-triggered on
// WaypointNavigatorState::Arrived going false -> true, the map-aware
// counterpart to HomeArrivalEventSource (Phase 13T), which observed plain
// HomeNavigator directly. HomeArrivalEventSource/HomeNavigator themselves
// are UNCHANGED and remain fully compiled/tested (see
// docs/technical-decisions.md, Phase 13X) - they are simply no longer
// main3d.cpp's production Return Home arrival signal now that
// WaypointNavigator (which owns its own internal HomeNavigator for
// per-waypoint local steering) is the thing actually driving Return Home,
// exactly like HomeZoneMonitor's own precedent (Phase 13V human-validation
// fix) of a component staying compiled/tested after its production wiring
// changes.
//
// Deliberately fires only once the FULL route is complete (the final
// waypoint), never once per intermediate waypoint - WaypointNavigator's
// own state() already encodes this distinction (see WaypointNavigator.hpp)
// - so this class needs no route-length/index awareness of its own,
// exactly mirroring HomeArrivalEventSource's own "just observe the already
// -computed state" simplicity.
//
// Re-arms automatically: WaypointNavigator resets to
// WaypointNavigatorState::Inactive whenever `enabled` goes false (see
// WaypointNavigator::update()'s own docs), which happens the moment
// RobotController stops requesting the underlying goal after HomeReached
// is consumed - the next distinct arrival always starts from a fresh
// false -> true edge, with no manual reset needed by any caller.
//
// Holds only a const WaypointNavigator& - it never itself decides whether
// the robot has arrived; it purely observes the state WaypointNavigator
// already computed.
class WaypointArrivalEventSource : public IPollingEventSource
{
public:
    explicit WaypointArrivalEventSource(const WaypointNavigator& navigator) noexcept;

    std::optional<Event> pollEvent() override;

private:
    const WaypointNavigator& navigator_;
    bool wasArrived_ = false;
};

} // namespace robot::visual
