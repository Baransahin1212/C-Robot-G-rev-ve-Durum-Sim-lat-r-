#pragma once

#include <cstddef>
#include <filesystem>
#include <ostream>
#include <string>
#include <vector>

namespace robot
{
class IRobotHardware;
} // namespace robot

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
    RunLive,
    MissingScenario,
    TooManyArguments,
    InvalidLiveArguments
};

struct ParsedArguments
{
    ArgumentAction action;
    std::string scenarioPath;       // meaningful only when action == Run
    std::size_t liveCycleCount = 0; // meaningful only when action == RunLive
    std::string sensorScriptPath;   // meaningful only when action == RunLive; empty means no script
    std::string commandScriptPath;  // meaningful only when action == RunLive; empty means no script
};

// Pure, testable argument parser - no I/O, no process exit.
ParsedArguments parseArguments(const std::vector<std::string>& args);

void printUsage(std::ostream& out);

// Runs one scenario end-to-end: JsonScenarioSource -> RobotStateMachine ->
// StreamSimulationLogger -> RobotController -> Simulator -> SimulationResult
// -> SimulationReport -> StreamReportWriter. Writes simulation.log and
// simulation_report.txt under the given directories (created if needed)
// and a summary to `out`. Returns kExitSuccess on a successfully executed
// simulation, or a specific non-zero code for a scenario-load or I/O
// failure (with a message written to `err`).
//
// Uses a default-constructed SimulatedRobotHardware, since the CLI is a
// desktop simulator - delegates to the IRobotHardware& overload below.
int runSimulation(const std::string& scenarioPath,
                   const std::filesystem::path& logsDir,
                   const std::filesystem::path& reportsDir,
                   std::ostream& out,
                   std::ostream& err);

// Same as above, but drives the caller-supplied IRobotHardware instead of
// constructing a SimulatedRobotHardware internally. Application still
// constructs the RobotController that maps FSM states to hardware commands
// - callers only ever supply hardware, never a controller - so this is the
// extension point a future real-hardware CLI path, or a test that wants to
// observe actuator commands, uses without touching FSM or Simulator code.
int runSimulation(const std::string& scenarioPath,
                   const std::filesystem::path& logsDir,
                   const std::filesystem::path& reportsDir,
                   std::ostream& out,
                   std::ostream& err,
                   IRobotHardware& hardware);

// Runs cycleCount deterministic live-polling cycles end-to-end:
// SimulatedRobotHardware -> HardwareEventSource -> RobotStateMachine ->
// RobotController -> RobotRuntime -> LiveRuntimeRunner. All objects are
// stack-local to this call; nothing is persisted between invocations. This
// is a separate orchestration path from runSimulation() - live mode never
// touches JsonScenarioSource or Simulator. Always returns kExitSuccess;
// there is no failure mode with default-safe simulated sensors and a
// bounded, in-process cycle count.
int runLiveSimulation(std::size_t cycleCount, std::ostream& out, std::ostream& err);

// Same live pipeline as above, but with an optional CommandScript and/or
// optional SensorScript layered in via ScriptedLiveRuntimeRunner instead
// of LiveRuntimeRunner - see docs/technical-decisions.md (Phase 13I/13J).
// An empty path means "not provided" for that axis (at least one of the
// two is expected to be non-empty when this overload is used - the
// no-argument overload above covers the "neither" case more directly).
// commandScriptPath entries are delivered through a
// ScriptedCommandEventSource composed ahead of HardwareEventSource via
// CompositePollingEventSource (command events take priority within a
// cycle - see CompositePollingEventSource); sensorScriptPath entries
// mutate SimulatedRobotHardware exactly as in Phase 13I. Each script is
// parsed eagerly before any cycle runs; a load/parse failure on either one
// returns kExitScenarioError with a message on `err` and runs zero cycles,
// mirroring how runSimulation() treats a bad scenario file.
int runLiveSimulation(std::size_t cycleCount,
                       const std::string& commandScriptPath,
                       const std::string& sensorScriptPath,
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
