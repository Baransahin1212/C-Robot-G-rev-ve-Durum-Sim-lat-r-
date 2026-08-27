#pragma once

#include <optional>

#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"
#include "robot/visual/DockApproachController.hpp"

namespace robot::visual
{

// Turns DockApproachController's own Arrived state into exactly one
// HomeReached Event per arrival (Phase 13X final-approach fix) - edge-
// triggered on DockApproachState::Arrived going false -> true, mirroring
// WaypointArrivalEventSource/HomeArrivalEventSource's own identical shape
// exactly (see either for the full "why edge-triggered" reasoning - not
// repeated here). This is the production Return Home arrival signal as of
// the two-stage Return Home fix: WaypointArrivalEventSource's own Arrived
// (WaypointNavigator reaching computeDockApproachPoint(), Stage 1 only)
// must NEVER itself fire HomeReached anymore - that would end the mission
// the moment the robot reaches the approach point, before it has actually
// docked. WaypointArrivalEventSource remains fully wired for its other,
// unchanged role (frontier-target arrival during Haritalama, read directly
// via WaypointNavigator::state() - never via its own Event - see
// main3d.cpp's frontier-selection block).
//
// Holds only a const DockApproachController& - it never itself decides
// whether the robot has arrived; it purely observes the state
// DockApproachController already computed.
class DockApproachArrivalEventSource : public IPollingEventSource
{
public:
    explicit DockApproachArrivalEventSource(const DockApproachController& controller) noexcept;

    std::optional<Event> pollEvent() override;

private:
    const DockApproachController& controller_;
    bool wasArrived_ = false;
};

} // namespace robot::visual
