#include "robot/RobotController.hpp"

namespace robot
{

RobotController::RobotController(IRobotHardware& hardware)
    : hardware_(hardware)
{
}

void RobotController::applyState(RobotState state)
{
    switch (state)
    {
        case RobotState::Idle:
        case RobotState::Ready:
        case RobotState::WaitingForObstacleClear:
        case RobotState::Completed:
        case RobotState::Aborted:
        case RobotState::EmergencyStopped:
        case RobotState::Error:
            hardware_.stop();
            break;

        case RobotState::Moving:
            hardware_.moveForward();
            break;

        case RobotState::ReturningHome:
            hardware_.returnToBase();
            break;
    }
}

} // namespace robot
