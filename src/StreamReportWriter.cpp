#include "robot/StreamReportWriter.hpp"

namespace robot
{

StreamReportWriter::StreamReportWriter(std::ostream& out)
    : out_(out)
{
}

void StreamReportWriter::write(const SimulationReport& report)
{
    out_ << "Simulation Result\n";
    out_ << "-----------------\n";
    out_ << "Mission outcome: " << toString(report.outcome) << "\n";
    out_ << "Final state: " << toString(report.finalState) << "\n";
    out_ << "Events processed: " << report.eventsProcessed << "\n";
    out_ << "Successful transitions: " << report.successfulTransitions << "\n";
    out_ << "Rejected transitions: " << report.rejectedTransitions << "\n";

    if (report.lastEventTimestampMs.has_value())
    {
        out_ << "Simulation end time: " << *report.lastEventTimestampMs << " ms\n";
    }
}

} // namespace robot
