#pragma once

#include <optional>

#include "robot/Event.hpp"

namespace robot
{

// Abstraction over anything that can supply a sequence of robot events:
// a JSON scenario file today, potentially a real sensor or network feed
// later. Implementations decide how events are produced; callers only
// pull events one at a time.
class IEventSource
{
public:
    virtual ~IEventSource() = default;

    // Returns the next event, or std::nullopt once the source is exhausted.
    virtual std::optional<Event> nextEvent() = 0;
};

} // namespace robot
