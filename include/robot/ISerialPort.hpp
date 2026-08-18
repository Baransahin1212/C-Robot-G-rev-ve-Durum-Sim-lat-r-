#pragma once

#include <string>

namespace robot
{

// Low-level, platform-independent line-oriented serial transport
// abstraction - the layer SerialRobotTransport sits on top of.
// Deliberately knows nothing about the robot wire protocol
// (RobotProtocol.hpp), IRobotHardware, RobotState, Event,
// RobotController, or RobotRuntime; it only ever writes text and reads a
// line of text back. Configuration a real serial port needs (device path,
// baud rate, parity, data/stop bits, flow control, timeouts) deliberately
// does not appear here - that belongs to a future concrete platform
// implementation's construction, not this interface.
//
// No concrete implementation exists yet beyond a test-only fake living in
// tests/SerialRobotTransportTests.cpp - a real Windows COM-port or Linux
// /dev/tty* implementation is deliberately out of scope for this phase;
// see docs/technical-decisions.md (Phase 13L).
class ISerialPort
{
public:
    virtual ~ISerialPort() = default;

    // Writes `data` verbatim - no framing (e.g. a trailing newline) is
    // added by this interface; that is SerialRobotTransport's
    // responsibility, not the serial port's.
    virtual void write(const std::string& data) = 0;

    // Reads and returns the next available line, with any line terminator
    // ('\n' or '\r\n') already removed - callers never see it. What
    // happens when no line is available yet is left to the implementation
    // (a future real port would presumably block or time out); this
    // interface defines no such policy.
    virtual std::string readLine() = 0;
};

} // namespace robot
