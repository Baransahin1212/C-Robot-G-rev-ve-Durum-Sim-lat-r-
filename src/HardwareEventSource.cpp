#include "robot/HardwareEventSource.hpp"

namespace robot
{

HardwareEventSource::HardwareEventSource(IRobotHardware& hardware)
    : hardware_(hardware)
{
}

std::optional<Event> HardwareEventSource::nextEvent()
{
    if (pendingEvents_.empty())
    {
        sampleAndEnqueueEdges();
    }

    if (pendingEvents_.empty())
    {
        return std::nullopt;
    }

    const Event event = pendingEvents_.front();
    pendingEvents_.pop_front();
    return event;
}

void HardwareEventSource::sampleAndEnqueueEdges()
{
    const bool emergencyNow = hardware_.emergencyStopPressed();
    const int batteryNow = hardware_.batteryLevelPercent();
    const bool batteryCriticalNow = batteryNow <= kCriticalBatteryPercent;
    const bool obstacleNow = hardware_.obstacleDetected();

    // Deterministic safety priority: emergency stop, then battery, then
    // obstacle - queued in this order so a caller draining pendingEvents_
    // always sees the most urgent condition first, even when several
    // sensors changed between snapshots.
    if (emergencyNow && !wasEmergencyStopPressed_)
    {
        pendingEvents_.push_back(makeEvent(EventType::EmergencyStop));
    }

    if (batteryCriticalNow && !wasBatteryCritical_)
    {
        pendingEvents_.push_back(makeEvent(EventType::BatteryCritical, static_cast<double>(batteryNow)));
    }

    if (obstacleNow && !wasObstacleDetected_)
    {
        pendingEvents_.push_back(makeEvent(EventType::ObstacleDetected));
    }
    else if (!obstacleNow && wasObstacleDetected_)
    {
        pendingEvents_.push_back(makeEvent(EventType::ObstacleCleared));
    }

    wasEmergencyStopPressed_ = emergencyNow;
    wasBatteryCritical_ = batteryCriticalNow;
    wasObstacleDetected_ = obstacleNow;
}

Event HardwareEventSource::makeEvent(EventType type, std::optional<double> value)
{
    return Event{type, nextTimestampMs_++, value};
}

} // namespace robot
