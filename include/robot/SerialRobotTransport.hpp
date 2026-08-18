#pragma once

#include <string>

#include "robot/IRobotTransport.hpp"
#include "robot/ISerialPort.hpp"

namespace robot
{

// IRobotTransport implementation over a line-oriented ISerialPort: adds
// exactly one responsibility - '\n' framing - on top of whatever message
// it is given. It has no knowledge of RobotProtocol's specific command/
// query/response strings, RobotState, Event, or the FSM; every
// command/request string is treated identically, and it only ever writes
// "<message>\n" and, for queries, returns ISerialPort::readLine()'s result
// unchanged. Semantic interpretation (BATTERY/OBSTACLE/ESTOP parsing,
// valid ranges, ...) remains RealRobotHardware's responsibility -
// SerialRobotTransport never looks at message content beyond appending
// the frame terminator.
//
// No OS-level ISerialPort implementation exists yet - see
// docs/technical-decisions.md (Phase 13L). This class is tested only
// against a test-only ISerialPort fake in
// tests/SerialRobotTransportTests.cpp.
class SerialRobotTransport : public IRobotTransport
{
public:
    // serialPort must outlive this object - the same non-owning-reference
    // pattern used throughout this codebase. No heap allocation.
    explicit SerialRobotTransport(ISerialPort& serialPort);

    // Writes command + '\n' to the serial port. No response is read.
    void sendCommand(const std::string& command) override;

    // Writes request + '\n' to the serial port, then returns
    // serialPort_.readLine()'s result unchanged - not trimmed, not
    // validated, not substituted on failure. An exception thrown by the
    // underlying ISerialPort propagates unchanged; an empty string
    // returned by readLine() is likewise returned unchanged -
    // RealRobotHardware's existing RobotTransportError handling is what
    // ultimately rejects malformed or empty responses, not this class.
    std::string query(const std::string& request) override;

private:
    ISerialPort& serialPort_;
};

} // namespace robot
