#include "robot/Application.hpp"

#include <cstdlib>
#include <fstream>
#include <limits>
#include <optional>
#include <stdexcept>

#include "robot/HardwareEventSource.hpp"
#include "robot/JsonScenarioSource.hpp"
#include "robot/LiveRuntimeRunner.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotRuntime.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/ScriptedLiveRuntimeRunner.hpp"
#include "robot/SensorScript.hpp"
#include "robot/SimulatedRobotHardware.hpp"
#include "robot/SimulationReport.hpp"
#include "robot/Simulator.hpp"
#include "robot/StreamReportWriter.hpp"
#include "robot/StreamSimulationLogger.hpp"

namespace robot::app
{

namespace
{

// Parses a non-negative cycle count that must consume the entire input
// string and fit in std::size_t. Rejects a leading '-' explicitly because
// std::stoull silently accepts "-1" (wrapping it into a huge unsigned
// value), rejects any non-numeric or trailing-garbage input by checking
// that the whole string was consumed, and rejects out-of-range input via
// the caught exceptions.
std::optional<std::size_t> parseCycleCount(const std::string& text)
{
    if (text.empty() || text.front() == '-')
    {
        return std::nullopt;
    }

    std::size_t consumed = 0;
    unsigned long long value = 0;
    try
    {
        value = std::stoull(text, &consumed);
    }
    catch (const std::exception&)
    {
        return std::nullopt;
    }

    if (consumed != text.size())
    {
        return std::nullopt;
    }

    if (value > std::numeric_limits<std::size_t>::max())
    {
        return std::nullopt;
    }

    return static_cast<std::size_t>(value);
}

void printLiveSummary(std::ostream& out, const RuntimeRunSummary& summary, RobotState finalState)
{
    out << "Live runtime completed\n"
        << "Cycles executed: " << summary.cyclesExecuted << "\n"
        << "No-event cycles: " << summary.noEventCycles << "\n"
        << "Accepted transitions: " << summary.acceptedTransitions << "\n"
        << "Rejected transitions: " << summary.rejectedTransitions << "\n"
        << "Final state: " << toString(finalState) << "\n";
}

} // namespace

ParsedArguments parseArguments(const std::vector<std::string>& args)
{
    if (args.empty())
    {
        return ParsedArguments{ArgumentAction::MissingScenario, {}, {}};
    }

    if (args.size() == 1 && (args[0] == "--help" || args[0] == "-h"))
    {
        return ParsedArguments{ArgumentAction::Help, {}, {}};
    }

    if (args[0] == "--live")
    {
        if (args.size() < 3 || args[1] != "--cycles")
        {
            return ParsedArguments{ArgumentAction::InvalidLiveArguments, {}, {}, {}};
        }

        const std::optional<std::size_t> cycleCount = parseCycleCount(args[2]);
        if (!cycleCount.has_value())
        {
            return ParsedArguments{ArgumentAction::InvalidLiveArguments, {}, {}, {}};
        }

        if (args.size() == 3)
        {
            return ParsedArguments{ArgumentAction::RunLive, {}, *cycleCount, {}};
        }

        if (args.size() == 5 && args[3] == "--sensor-script")
        {
            return ParsedArguments{ArgumentAction::RunLive, {}, *cycleCount, args[4]};
        }

        return ParsedArguments{ArgumentAction::InvalidLiveArguments, {}, {}, {}};
    }

    if (args.size() > 1)
    {
        return ParsedArguments{ArgumentAction::TooManyArguments, {}, {}};
    }

    return ParsedArguments{ArgumentAction::Run, args[0], {}};
}

void printUsage(std::ostream& out)
{
    out << "RobotSimulator - C++ Robot Task and State Simulator\n\n"
        << "Usage:\n"
        << "  RobotSimulator <scenario-file>\n"
        << "  RobotSimulator --live --cycles <N>\n"
        << "  RobotSimulator --live --cycles <N> --sensor-script <file>\n"
        << "  RobotSimulator --help\n\n"
        << "Scenario mode runs a finite JSON scenario end-to-end.\n"
        << "Live mode runs N deterministic polling cycles against\n"
        << "simulated hardware. --sensor-script optionally replays\n"
        << "deterministic sensor changes at specific cycles.\n\n"
        << "Examples:\n"
        << "  RobotSimulator scenarios/normal_mission.json\n"
        << "  RobotSimulator --live --cycles 100\n"
        << "  RobotSimulator --live --cycles 100 --sensor-script sensors.txt\n";
}

int runSimulation(const std::string& scenarioPath,
                   const std::filesystem::path& logsDir,
                   const std::filesystem::path& reportsDir,
                   std::ostream& out,
                   std::ostream& err)
{
    SimulatedRobotHardware hardware;
    return runSimulation(scenarioPath, logsDir, reportsDir, out, err, hardware);
}

int runSimulation(const std::string& scenarioPath,
                   const std::filesystem::path& logsDir,
                   const std::filesystem::path& reportsDir,
                   std::ostream& out,
                   std::ostream& err,
                   IRobotHardware& hardware)
{
    std::optional<JsonScenarioSource> source;
    try
    {
        source.emplace(scenarioPath);
    }
    catch (const ScenarioParseError& e)
    {
        err << "Error loading scenario: " << e.what() << "\n";
        return kExitScenarioError;
    }

    std::ofstream logFile;
    std::ofstream reportFile;
    std::filesystem::path reportPath;
    try
    {
        std::filesystem::create_directories(logsDir);
        std::filesystem::create_directories(reportsDir);

        const std::filesystem::path logPath = logsDir / "simulation.log";
        logFile.open(logPath);
        if (!logFile.is_open())
        {
            throw std::runtime_error("could not open " + logPath.string() + " for writing");
        }

        reportPath = reportsDir / "simulation_report.txt";
        reportFile.open(reportPath);
        if (!reportFile.is_open())
        {
            throw std::runtime_error("could not open " + reportPath.string() + " for writing");
        }
    }
    catch (const std::exception& e)
    {
        err << "Error creating output files: " << e.what() << "\n";
        return kExitIOError;
    }

    RobotStateMachine machine;
    StreamSimulationLogger logger(logFile);
    RobotController controller(hardware);
    Simulator simulator(*source, machine, &logger, &controller);

    const SimulationResult result = simulator.run();
    const SimulationReport report = makeSimulationReport(result);

    StreamReportWriter fileWriter(reportFile);
    fileWriter.write(report);

    out << "Scenario: " << scenarioPath << "\n\n";
    StreamReportWriter consoleWriter(out);
    consoleWriter.write(report);

    return kExitSuccess;
}

int runLiveSimulation(std::size_t cycleCount, std::ostream& out, std::ostream& err)
{
    (void)err;

    SimulatedRobotHardware hardware;
    HardwareEventSource eventSource(hardware);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(eventSource, stateMachine, controller);
    LiveRuntimeRunner runner(runtime);

    const RuntimeRunSummary summary = runner.runCycles(cycleCount);

    printLiveSummary(out, summary, stateMachine.currentState());
    return kExitSuccess;
}

int runLiveSimulation(std::size_t cycleCount,
                       const std::string& sensorScriptPath,
                       std::ostream& out,
                       std::ostream& err)
{
    std::optional<SensorScript> script;
    try
    {
        script.emplace(sensorScriptPath);
    }
    catch (const SensorScriptParseError& e)
    {
        err << "Error loading sensor script: " << e.what() << "\n";
        return kExitScenarioError;
    }

    SimulatedRobotHardware hardware;
    HardwareEventSource eventSource(hardware);
    RobotStateMachine stateMachine;
    RobotController controller(hardware);
    RobotRuntime runtime(eventSource, stateMachine, controller);
    ScriptedLiveRuntimeRunner runner(runtime, hardware, *script);

    const RuntimeRunSummary summary = runner.runCycles(cycleCount);

    out << "Sensor script: " << sensorScriptPath << "\n";
    printLiveSummary(out, summary, stateMachine.currentState());
    return kExitSuccess;
}

int runApplication(const std::vector<std::string>& args,
                    const std::filesystem::path& logsDir,
                    const std::filesystem::path& reportsDir,
                    std::ostream& out,
                    std::ostream& err)
{
    const ParsedArguments parsed = parseArguments(args);

    switch (parsed.action)
    {
        case ArgumentAction::Help:
            printUsage(out);
            return kExitSuccess;

        case ArgumentAction::MissingScenario:
            err << "Error: no scenario file provided.\n\n";
            printUsage(err);
            return kExitUsageError;

        case ArgumentAction::TooManyArguments:
            err << "Error: too many arguments.\n\n";
            printUsage(err);
            return kExitUsageError;

        case ArgumentAction::InvalidLiveArguments:
            err << "Error: invalid live mode arguments. Expected: --live --cycles <N>\n\n";
            printUsage(err);
            return kExitUsageError;

        case ArgumentAction::Run:
            return runSimulation(parsed.scenarioPath, logsDir, reportsDir, out, err);

        case ArgumentAction::RunLive:
            if (parsed.sensorScriptPath.empty())
            {
                return runLiveSimulation(parsed.liveCycleCount, out, err);
            }
            return runLiveSimulation(parsed.liveCycleCount, parsed.sensorScriptPath, out, err);
    }

    return kExitUsageError;
}

int runApplication(int argc,
                    char** argv,
                    const std::filesystem::path& logsDir,
                    const std::filesystem::path& reportsDir,
                    std::ostream& out,
                    std::ostream& err)
{
    const std::vector<std::string> args(argv + 1, argv + argc);
    return runApplication(args, logsDir, reportsDir, out, err);
}

} // namespace robot::app
