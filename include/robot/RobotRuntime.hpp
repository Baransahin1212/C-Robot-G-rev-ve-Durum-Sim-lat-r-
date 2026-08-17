#pragma once

#include "robot/IPollingEventSource.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotStateMachine.hpp"

namespace robot
{

// Outcome of one RobotRuntime::step() call.
enum class RuntimeStepResult
{
    NoEvent,
    TransitionAccepted,
    TransitionRejected
};

// Live, one-cycle-at-a-time counterpart to Simulator, built for
// IPollingEventSource instead of the finite IEventSource. Simulator::run()
// loops internally until nextEvent() reports exhaustion; RobotRuntime never
// loops on its own at all - step() polls at most one Event and returns
// immediately, because for a polling source std::nullopt means "nothing
// this cycle," not "done forever." Scheduling how often step() runs (a
// loop, a timer, a callback) is deliberately left to the caller; this
// class only provides the deterministic single-cycle primitive - it must
// never sleep, retry, block, or process more than one Event per call.
//
// RobotRuntime owns neither the source, the state machine, nor the
// controller - all are supplied by the caller and must outlive it, the
// same non-owning-reference pattern Simulator uses for its dependencies.
// It knows only IPollingEventSource, RobotStateMachine, and
// RobotController - it has no knowledge of IRobotHardware or
// SimulatedRobotHardware, never reads a sensor directly, and never calls
// an actuator method directly. Hardware synchronization and commands both
// flow exclusively through RobotController::applyState().
class RobotRuntime
{
public:
    RobotRuntime(IPollingEventSource& source, RobotStateMachine& stateMachine, RobotController& controller);

    // Processes at most one Event:
    //   - On the very first call only, synchronizes the controller with
    //     the state machine's current state before polling, so hardware
    //     never starts out of sync with the FSM. Later calls skip this -
    //     synchronization happens once, not every cycle.
    //   - Polls exactly one Event via IPollingEventSource::pollEvent(). If
    //     none is available, returns NoEvent without touching the FSM or
    //     the controller.
    //   - Otherwise hands the event to RobotStateMachine::processEvent().
    //     On an accepted transition, applies the resulting state through
    //     the controller and returns TransitionAccepted. On a rejected
    //     transition, the controller is not called and TransitionRejected
    //     is returned - actuator state never changes because of a
    //     transition the FSM refused.
    RuntimeStepResult step();

private:
    IPollingEventSource& source_;
    RobotStateMachine& stateMachine_;
    RobotController& controller_;
    bool initialized_ = false;
};

} // namespace robot
