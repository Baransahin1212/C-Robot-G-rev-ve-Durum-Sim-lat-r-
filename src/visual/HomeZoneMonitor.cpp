#include "robot/visual/HomeZoneMonitor.hpp"

#include <cmath>

namespace robot::visual
{

bool HomeZoneMonitor::update(const RobotPose& pose, const BasePlatform& base, bool roamActive) noexcept
{
    if (!roamActive)
    {
        // Not currently Roaming - discard any not-yet-consumed pending
        // trigger from a moment ago (it is now stale: the task changed
        // before pollEvent() delivered it, e.g. the user pressed Stop
        // Task) so it can never fire once Roam resumes at some unrelated
        // later point. The armed/disarmed latch itself is left untouched
        // - it resumes exactly where it left off the next time
        // `roamActive` is true.
        pendingEvent_ = false;
        return false;
    }

    const float dx = base.position.x - pose.position.x;
    const float dz = base.position.z - pose.position.z;
    const float distance = std::sqrt((dx * dx) + (dz * dz));

    if (armed_ && distance > kHomeZoneExitRadius)
    {
        armed_ = false;
        pendingEvent_ = true;
        return true;
    }

    if (!armed_ && distance <= kHomeZoneRearmRadius)
    {
        // Re-arms only - does not itself request anything. The next exit-
        // radius crossing is what triggers again (see
        // LeavingAgainTriggersSecondTime).
        armed_ = true;
    }

    return false;
}

bool HomeZoneMonitor::armed() const noexcept
{
    return armed_;
}

std::optional<Event> HomeZoneMonitor::pollEvent()
{
    if (!pendingEvent_)
    {
        return std::nullopt;
    }

    pendingEvent_ = false;
    return Event{EventType::ReturnHomeRequested, 0, std::nullopt};
}

} // namespace robot::visual
