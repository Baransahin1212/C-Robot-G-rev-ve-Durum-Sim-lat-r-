#include "robot/Application.hpp"

#include <fstream>
#include <optional>
#include <stdexcept>

#include "robot/JsonScenarioSource.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/SimulationReport.hpp"
#include "robot/Simulator.hpp"
#include "robot/StreamReportWriter.hpp"
#include "robot/StreamSimulationLogger.hpp"

namespace robot::app
{

ParsedArguments parseArguments(const std::vector<std::string>& args)
{
    if (args.empty())
    {
        return ParsedArguments{ArgumentAction::MissingScenario, {}};
    }

    if (args.size() == 1 && (args[0] == "--help" || args[0] == "-h"))
    {
        return ParsedArguments{ArgumentAction::Help, {}};
    }

    if (args.size() > 1)
    {
        return ParsedArguments{ArgumentAction::TooManyArguments, {}};
    }

    return ParsedArguments{ArgumentAction::Run, args[0]};
}

void printUsage(std::ostream& out)
{
    out << "RobotSimulator - C++ Robot Task and State Simulator\n\n"
        << "Usage:\n"
        << "  RobotSimulator <scenario-file>\n"
        << "  RobotSimulator --help\n\n"
        << "Example:\n"
        << "  RobotSimulator scenarios/normal_mission.json\n";
}

int runSimulation(const std::string& scenarioPath,
                   const std::filesystem::path& logsDir,
                   const std::filesystem::path& reportsDir,
                   std::ostream& out,
                   std::ostream& err)
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
    Simulator simulator(*source, machine, &logger);

    const SimulationResult result = simulator.run();
    const SimulationReport report = makeSimulationReport(result);

    StreamReportWriter fileWriter(reportFile);
    fileWriter.write(report);

    out << "Scenario: " << scenarioPath << "\n\n";
    StreamReportWriter consoleWriter(out);
    consoleWriter.write(report);

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

        case ArgumentAction::Run:
            return runSimulation(parsed.scenarioPath, logsDir, reportsDir, out, err);
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
