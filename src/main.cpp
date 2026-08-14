#include <iostream>
#include <string>

#include "robot/Event.hpp"
#include "robot/JsonScenarioSource.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"
#include "robot/Simulator.hpp"

namespace
{

void apply(robot::RobotStateMachine& machine, robot::EventType eventType, std::uint64_t timestampMs)
{
    const robot::Event event{eventType, timestampMs, std::nullopt};
    const robot::TransitionResult result = machine.processEvent(event);

    std::cout << "  event=" << robot::toString(eventType)
              << " -> result=" << (result == robot::TransitionResult::Success ? "Success" : "InvalidTransition")
              << " state=" << robot::toString(machine.currentState())
              << std::endl;
}

} // namespace

int main()
{
    std::cout << "RobotSimulator skeleton OK" << std::endl;

    std::cout << "\nNormal mission: Idle -> Ready -> Moving -> Completed" << std::endl;
    {
        robot::RobotStateMachine machine;
        apply(machine, robot::EventType::ScenarioLoaded, 0);
        apply(machine, robot::EventType::StartMission, 10);
        apply(machine, robot::EventType::MissionCompleted, 20);
    }

    std::cout << "\nObstacle: Moving -> WaitingForObstacleClear -> Moving" << std::endl;
    {
        robot::RobotStateMachine machine;
        apply(machine, robot::EventType::ScenarioLoaded, 0);
        apply(machine, robot::EventType::StartMission, 10);
        apply(machine, robot::EventType::ObstacleDetected, 20);
        apply(machine, robot::EventType::ObstacleCleared, 30);
    }

    std::cout << "\nLow battery: Moving -> ReturningHome -> Aborted" << std::endl;
    {
        robot::RobotStateMachine machine;
        apply(machine, robot::EventType::ScenarioLoaded, 0);
        apply(machine, robot::EventType::StartMission, 10);
        apply(machine, robot::EventType::BatteryCritical, 20);
        apply(machine, robot::EventType::HomeReached, 30);
    }

    std::cout << "\nInvalid transition: Idle + MissionCompleted is rejected" << std::endl;
    {
        robot::RobotStateMachine machine;
        apply(machine, robot::EventType::MissionCompleted, 0);
    }

    std::cout << "\nRunning JSON scenario through Simulator:" << std::endl;
    try
    {
        robot::JsonScenarioSource source(std::string(SCENARIOS_DIR) + "obstacle_test.json");
        robot::RobotStateMachine machine;
        robot::Simulator simulator(source, machine);

        const robot::SimulationResult result = simulator.run();

        std::cout << "Scenario finished" << std::endl;
        std::cout << "Final state: " << robot::toString(result.finalState) << std::endl;
        std::cout << "Events processed: " << result.eventsProcessed << std::endl;
        std::cout << "Successful transitions: " << result.successfulTransitions << std::endl;
        std::cout << "Rejected transitions: " << result.rejectedTransitions << std::endl;
    }
    catch (const robot::ScenarioParseError& e)
    {
        std::cout << "Failed to read scenario: " << e.what() << std::endl;
    }

    return 0;
}
