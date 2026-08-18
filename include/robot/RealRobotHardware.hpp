#pragma once

#include <stdexcept>
#include <string>

#include "robot/IRobotHardware.hpp"
#include "robot/IRobotTransport.hpp"

namespace robot
{

// Thrown when a transport response does not conform to the wire protocol
// in RobotProtocol.hpp: wrong prefix, missing or extra tokens, a
// non-numeric or out-of-range value, or an empty response. Communication
// data that cannot be trusted must fail loudly - RealRobotHardware never
// substitutes a fabricated sensor value for one it could not parse.
class RobotTransportError : public std::runtime_error
{
public:
    explicit RobotTransportError(const std::string& message)
        : std::runtime_error(message)
    {
    }
};

// IRobotHardware implementation that drives a robot controller through
// IRobotTransport's tiny text request/response protocol, instead of the
// in-memory state SimulatedRobotHardware uses. See
// docs/technical-decisions.md (Phase 13K) for the transport-boundary
// rationale; no concrete transport (serial or otherwise) exists yet.
//
// RealRobotHardware knows only IRobotTransport and the wire-protocol
// strings in RobotProtocol.hpp - it has no knowledge of RobotState, Event,
// RobotStateMachine, RobotController, RobotRuntime, CommandScript, or
// SensorScript. It implements IRobotHardware only, so it is a drop-in
// replacement for SimulatedRobotHardware anywhere an IRobotHardware& is
// accepted (RobotController, HardwareEventSource, ...), and neither of
// those needs to know, or is able to tell, which implementation it was
// given.
class RealRobotHardware : public IRobotHardware
{
public:
    // transport must outlive this object - the same non-owning-reference
    // pattern used throughout this codebase (RobotController,
    // HardwareEventSource, ...). No heap allocation.
    explicit RealRobotHardware(IRobotTransport& transport);

    // Throws RobotTransportError if the transport's response does not
    // conform to the expected "<PREFIX> <value>" wire format for this
    // query, or the value is out of range.
    int batteryLevelPercent() const override;
    bool obstacleDetected() const override;
    bool emergencyStopPressed() const override;

    void moveForward() override;
    void stop() override;
    void returnToBase() override;

private:
    IRobotTransport& transport_;
};

} // namespace robot
