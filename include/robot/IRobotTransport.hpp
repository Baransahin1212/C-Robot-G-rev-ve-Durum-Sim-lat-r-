#pragma once

#include <string>

namespace robot
{

// Abstraction over communication with a lower-level robot controller (a
// future ESP32/Pico firmware, for example) - the transport boundary
// RealRobotHardware speaks the tiny text protocol in RobotProtocol.hpp
// over. Deliberately platform-independent and deliberately ignorant of
// this codebase's own vocabulary: IRobotTransport knows only request/
// response text messages. It has no notion of RobotState, Event,
// EventType, RobotStateMachine, RobotController, or RobotRuntime - moving
// robot-domain meaning into transport would make every future concrete
// implementation (serial, network, mock) reimplement that meaning, instead
// of just moving bytes.
//
// No concrete implementation is added in this phase (Phase 13K) beyond a
// test-only fake living in tests/RealRobotHardwareTests.cpp - a real
// serial-port transport is deliberately out of scope; see
// docs/technical-decisions.md.
class IRobotTransport
{
public:
    virtual ~IRobotTransport() = default;

    // Sends a one-way command message (e.g. "MOVE_FORWARD") with no
    // expected response.
    virtual void sendCommand(const std::string& command) = 0;

    // Sends a query message (e.g. "GET_BATTERY") and returns the single
    // response line the far end sends back for it. Implementations decide
    // how a communication failure (timeout, disconnect) is surfaced; this
    // interface itself defines no error contract beyond "the returned
    // string is whatever text came back" - RealRobotHardware is
    // responsible for validating that text against the wire protocol.
    virtual std::string query(const std::string& request) = 0;
};

} // namespace robot
