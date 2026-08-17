#include "robot/LiveRuntimeRunner.hpp"

namespace robot
{

LiveRuntimeRunner::LiveRuntimeRunner(RobotRuntime& runtime)
    : runtime_(runtime)
{
}

RuntimeRunSummary LiveRuntimeRunner::runCycles(std::size_t cycleCount)
{
    RuntimeRunSummary summary;

    for (std::size_t i = 0; i < cycleCount; ++i)
    {
        ++summary.cyclesExecuted;

        switch (runtime_.step())
        {
            case RuntimeStepResult::NoEvent:
                ++summary.noEventCycles;
                break;
            case RuntimeStepResult::TransitionAccepted:
                ++summary.acceptedTransitions;
                break;
            case RuntimeStepResult::TransitionRejected:
                ++summary.rejectedTransitions;
                break;
        }
    }

    return summary;
}

} // namespace robot
