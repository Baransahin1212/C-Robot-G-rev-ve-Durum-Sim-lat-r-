#pragma once

#include <cstdint>
#include <ostream>
#include <string>

#include "robot/ISimulationLogger.hpp"

namespace robot
{

// Writes human-readable log lines to a caller-supplied std::ostream, so the
// same implementation works for std::cout, std::ofstream, or
// std::ostringstream. The stream is a non-owning reference: the caller
// creates it, keeps it alive for as long as the logger is used, and closes
// or destroys it afterward. StreamSimulationLogger never opens, closes, or
// takes ownership of any file or stream.
class StreamSimulationLogger : public ISimulationLogger
{
public:
    explicit StreamSimulationLogger(std::ostream& out);

    void logEventReceived(const Event& event) override;
    void logTransitionSucceeded(std::uint64_t timestampMs, RobotState fromState, RobotState toState) override;
    void logTransitionRejected(std::uint64_t timestampMs, RobotState state, EventType eventType) override;

private:
    void writeLine(LogLevel level, std::uint64_t timestampMs, const std::string& message);

    std::ostream& out_;
};

} // namespace robot
