#include "robot/StreamSimulationLogger.hpp"

#include <sstream>

namespace robot
{

StreamSimulationLogger::StreamSimulationLogger(std::ostream& out)
    : out_(out)
{
}

void StreamSimulationLogger::logEventReceived(const Event& event)
{
    std::ostringstream message;
    message << "Event: " << toString(event.type);
    if (event.value.has_value())
    {
        message << " value=" << *event.value;
    }
    writeLine(LogLevel::Info, event.timestampMs, message.str());
}

void StreamSimulationLogger::logTransitionSucceeded(std::uint64_t timestampMs, RobotState fromState, RobotState toState)
{
    std::ostringstream message;
    message << "State: " << toString(fromState) << " -> " << toString(toState);
    writeLine(LogLevel::Info, timestampMs, message.str());
}

void StreamSimulationLogger::logTransitionRejected(std::uint64_t timestampMs, RobotState state, EventType eventType)
{
    std::ostringstream message;
    message << "Transition rejected: " << toString(state) << " + " << toString(eventType);
    writeLine(LogLevel::Warning, timestampMs, message.str());
}

void StreamSimulationLogger::writeLine(LogLevel level, std::uint64_t timestampMs, const std::string& message)
{
    out_ << "[t=" << timestampMs << "ms] [" << toString(level) << "] " << message << "\n";
}

} // namespace robot
