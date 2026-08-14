#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "robot/MissionOutcome.hpp"
#include "robot/RobotState.hpp"
#include "robot/Simulator.hpp"

namespace robot
{

// Human-facing summary of one Simulator::run(), derived from a
// SimulationResult. Kept separate from SimulationResult itself so the
// Simulator's raw counters stay a pure orchestration output, independent of
// how (or whether) they are ever reported.
struct SimulationReport
{
    MissionOutcome outcome;
    RobotState finalState;
    std::size_t eventsProcessed;
    std::size_t successfulTransitions;
    std::size_t rejectedTransitions;
    std::optional<std::uint64_t> lastEventTimestampMs;
};

inline SimulationReport makeSimulationReport(const SimulationResult& result)
{
    SimulationReport report;
    report.outcome = missionOutcomeFromState(result.finalState);
    report.finalState = result.finalState;
    report.eventsProcessed = result.eventsProcessed;
    report.successfulTransitions = result.successfulTransitions;
    report.rejectedTransitions = result.rejectedTransitions;
    report.lastEventTimestampMs = result.lastEventTimestampMs;
    return report;
}

} // namespace robot
