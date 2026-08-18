#include "robot/ScriptedLiveRuntimeRunner.hpp"

namespace robot
{

ScriptedLiveRuntimeRunner::ScriptedLiveRuntimeRunner(RobotRuntime& runtime,
                                                       SimulatedRobotHardware& hardware,
                                                       const SensorScript& sensorScript)
    : runtime_(runtime)
    , hardware_(hardware)
    , sensorScript_(&sensorScript)
    , commandSource_(nullptr)
{
}

ScriptedLiveRuntimeRunner::ScriptedLiveRuntimeRunner(RobotRuntime& runtime,
                                                       SimulatedRobotHardware& hardware,
                                                       const SensorScript& sensorScript,
                                                       ScriptedCommandEventSource& commandSource)
    : runtime_(runtime)
    , hardware_(hardware)
    , sensorScript_(&sensorScript)
    , commandSource_(&commandSource)
{
}

ScriptedLiveRuntimeRunner::ScriptedLiveRuntimeRunner(RobotRuntime& runtime,
                                                       SimulatedRobotHardware& hardware,
                                                       ScriptedCommandEventSource& commandSource)
    : runtime_(runtime)
    , hardware_(hardware)
    , sensorScript_(nullptr)
    , commandSource_(&commandSource)
{
}

void ScriptedLiveRuntimeRunner::applyEntriesForCycle(std::size_t cycle)
{
    const std::vector<SensorScriptEntry>& entries = sensorScript_->entries();

    while (nextEntryIndex_ < entries.size() && entries[nextEntryIndex_].cycle == cycle)
    {
        const SensorScriptEntry& entry = entries[nextEntryIndex_];
        switch (entry.sensor)
        {
            case SensorKind::Obstacle:
                hardware_.setObstacleDetected(entry.value != 0);
                break;
            case SensorKind::Battery:
                hardware_.setBatteryLevelPercent(entry.value);
                break;
            case SensorKind::EmergencyStop:
                hardware_.setEmergencyStopPressed(entry.value != 0);
                break;
        }
        ++nextEntryIndex_;
    }
}

RuntimeRunSummary ScriptedLiveRuntimeRunner::runCycles(std::size_t cycleCount)
{
    RuntimeRunSummary summary;

    for (std::size_t cycle = 0; cycle < cycleCount; ++cycle)
    {
        if (sensorScript_ != nullptr)
        {
            applyEntriesForCycle(cycle);
        }
        if (commandSource_ != nullptr)
        {
            commandSource_->setCurrentCycle(cycle);
        }

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
