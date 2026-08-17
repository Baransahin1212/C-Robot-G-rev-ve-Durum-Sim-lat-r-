#pragma once

#include <optional>

#include "robot/Event.hpp"

namespace robot
{

// Abstraction over anything that can be polled once per cycle for at most
// one currently available Event: live sensor input today, potentially a
// network feed or other real-time source later. Deliberately a separate
// interface from IEventSource rather than reusing it, because the two have
// genuinely different std::nullopt semantics that must not be conflated:
//
//   IEventSource::nextEvent()        -> nullopt means the source is
//                                        exhausted; Simulator::run() stops
//                                        for good and does not call it
//                                        again.
//   IPollingEventSource::pollEvent() -> nullopt means nothing is available
//                                        THIS cycle; the source may still
//                                        produce events on a later poll.
//
// A finite scenario file naturally satisfies the first contract; live
// sensor input naturally satisfies the second. HardwareEventSource
// implements both interfaces over the same underlying edge-triggered
// queue - the returned Event (or lack of one) is identical either way,
// only the caller's interpretation of nullopt differs. RobotRuntime is the
// polling-side counterpart to Simulator: it depends on this interface, not
// on any concrete polling source.
class IPollingEventSource
{
public:
    virtual ~IPollingEventSource() = default;

    // Returns one currently available event, or std::nullopt if nothing is
    // available during this polling cycle. std::nullopt does NOT mean the
    // source is permanently exhausted - a later call may still return an
    // event.
    virtual std::optional<Event> pollEvent() = 0;
};

} // namespace robot
