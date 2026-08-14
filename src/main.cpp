#include <iostream>
#include <string>

#include "robot/Event.hpp"
#include "robot/RobotState.hpp"
#include "robot/RobotStateMachine.hpp"

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

    return 0;
}
