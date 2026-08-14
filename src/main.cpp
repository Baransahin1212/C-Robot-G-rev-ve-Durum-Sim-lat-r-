#include <iostream>
#include <string>

#include "robot/Event.hpp"
#include "robot/RobotState.hpp"

int main()
{
    std::cout << "RobotSimulator skeleton OK" << std::endl;

    const robot::RobotState state = robot::RobotState::Moving;
    std::cout << "RobotState: " << robot::toString(state) << std::endl;

    const robot::Event event{robot::EventType::BatteryCritical, 1234, 17.5};
    std::cout << "EventType: " << robot::toString(event.type)
              << " timestampMs: " << event.timestampMs
              << " value: " << (event.value.has_value() ? std::to_string(*event.value) : "none")
              << std::endl;

    const robot::Event eventNoValue{robot::EventType::StartMission, 5678, std::nullopt};
    std::cout << "EventType: " << robot::toString(eventNoValue.type)
              << " timestampMs: " << eventNoValue.timestampMs
              << " value: " << (eventNoValue.value.has_value() ? std::to_string(*eventNoValue.value) : "none")
              << std::endl;

    return 0;
}
