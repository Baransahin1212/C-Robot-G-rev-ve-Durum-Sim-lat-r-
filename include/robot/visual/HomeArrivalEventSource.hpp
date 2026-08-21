#pragma once

#include <optional>

#include "robot/Event.hpp"
#include "robot/IPollingEventSource.hpp"
#include "robot/visual/HomeNavigator.hpp"

namespace robot::visual
{

// Turns HomeNavigator's own Arrived state into exactly one HomeReached
// Event per arrival (Phase 13T) - edge-triggered on
// HomeNavigationState::Arrived going false -> true, mirroring
// HardwareEventSource's own edge-triggered pattern (see
// HardwareEventSource.hpp) rather than re-emitting HomeReached on every
// poll while the robot simply remains parked at the base. This is how
// HomeReached enters the system: never via a direct
// stateMachine.handleEvent(HomeReached) call from main3d.cpp - only
// through this IPollingEventSource, exactly like every other Event.
//
// Re-arms automatically: HomeNavigator::update() resets to
// HomeNavigationState::Inactive whenever `enabled` goes false (see
// HomeNavigator.hpp), which happens the moment RobotController stops
// requesting ReturnToBase after HomeReached is consumed (ReturningHome +
// HomeReached -> Aborted, RobotController::applyState(Aborted) calls
// hardware_.stop()). The next distinct arrival - a brand new Return Home
// mission - therefore always starts from a fresh false -> true edge, with
// no manual reset needed by any caller.
//
// Holds only a const HomeNavigator& - it never itself decides whether the
// robot has arrived; it purely observes the state HomeNavigator already
// computed, exactly like HomeNavigator itself never decides whether a
// Return Home mission should start.
class HomeArrivalEventSource : public IPollingEventSource
{
public:
    explicit HomeArrivalEventSource(const HomeNavigator& navigator) noexcept;

    std::optional<Event> pollEvent() override;

private:
    const HomeNavigator& navigator_;
    bool wasArrived_ = false;
};

} // namespace robot::visual
