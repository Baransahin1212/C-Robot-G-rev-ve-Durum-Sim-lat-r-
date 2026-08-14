#pragma once

#include <ostream>

#include "robot/SimulationReport.hpp"

namespace robot
{

// Writes a human-readable SimulationReport to a caller-supplied
// std::ostream (std::cout, std::ofstream, std::ostringstream, ...). The
// stream is a non-owning reference: the caller creates it, keeps it alive
// for as long as the writer is used, and closes or destroys it afterward.
// StreamReportWriter never opens, closes, or takes ownership of any file or
// stream.
class StreamReportWriter
{
public:
    explicit StreamReportWriter(std::ostream& out);

    void write(const SimulationReport& report);

private:
    std::ostream& out_;
};

} // namespace robot
