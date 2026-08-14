#pragma once

#include <filesystem>
#include <ostream>
#include <string>
#include <vector>

namespace robot::app
{

// Process exit codes. 0 means the simulation executed successfully -
// regardless of MissionOutcome, since Aborted/EmergencyStopped/Error are
// simulation results, not application failures. Non-zero is reserved for
// application-level failures: a bad command line, a scenario file that
// could not be read, or an output file that could not be written.
constexpr int kExitSuccess = 0;
constexpr int kExitUsageError = 1;
constexpr int kExitScenarioError = 2;
constexpr int kExitIOError = 3;

enum class ArgumentAction
{
    Help,
    Run,
    MissingScenario,
    TooManyArguments
};

struct ParsedArguments
{
    ArgumentAction action;
    std::string scenarioPath; // meaningful only when action == Run
};

// Pure, testable argument parser - no I/O, no process exit.
ParsedArguments parseArguments(const std::vector<std::string>& args);

void printUsage(std::ostream& out);

// Runs one scenario end-to-end: JsonScenarioSource -> RobotStateMachine ->
// StreamSimulationLogger -> Simulator -> SimulationResult ->
// SimulationReport -> StreamReportWriter. Writes simulation.log and
// simulation_report.txt under the given directories (created if needed)
// and a summary to `out`. Returns kExitSuccess on a successfully executed
// simulation, or a specific non-zero code for a scenario-load or I/O
// failure (with a message written to `err`).
int runSimulation(const std::string& scenarioPath,
                   const std::filesystem::path& logsDir,
                   const std::filesystem::path& reportsDir,
                   std::ostream& out,
                   std::ostream& err);

// Full application logic over an already-split argument list (excludes the
// program name), so tests can drive it directly without touching argv or
// spawning a subprocess.
int runApplication(const std::vector<std::string>& args,
                    const std::filesystem::path& logsDir,
                    const std::filesystem::path& reportsDir,
                    std::ostream& out,
                    std::ostream& err);

// Convenience overload for main(): splits argv into args and delegates.
int runApplication(int argc,
                    char** argv,
                    const std::filesystem::path& logsDir,
                    const std::filesystem::path& reportsDir,
                    std::ostream& out,
                    std::ostream& err);

} // namespace robot::app
