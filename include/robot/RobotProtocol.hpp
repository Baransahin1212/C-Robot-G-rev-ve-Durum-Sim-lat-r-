#pragma once

#include <string_view>

namespace robot::protocol
{

// Centralizes the tiny text wire protocol RealRobotHardware speaks to
// IRobotTransport, so no literal command/query/response-prefix string is
// duplicated across RealRobotHardware.cpp or its tests. This is a logical
// protocol contract only - no framing, checksum, retry, or serialization
// library is involved; a message is exactly one line of text.
//
// Actuator commands: one-way, no response expected.
inline constexpr std::string_view kMoveForward = "MOVE_FORWARD";
inline constexpr std::string_view kStop = "STOP";
inline constexpr std::string_view kReturnToBase = "RETURN_TO_BASE";

// Sensor queries: each expects exactly one response line back.
inline constexpr std::string_view kGetBattery = "GET_BATTERY";
inline constexpr std::string_view kGetObstacle = "GET_OBSTACLE";
inline constexpr std::string_view kGetEStop = "GET_ESTOP";

// Expected response prefix for each query above - the response is this
// prefix, one space, and a value token (e.g. "BATTERY 78", "OBSTACLE 1"),
// nothing more and nothing less. See RealRobotHardware.cpp for the strict
// parsing rules these prefixes are checked against.
inline constexpr std::string_view kBatteryResponsePrefix = "BATTERY";
inline constexpr std::string_view kObstacleResponsePrefix = "OBSTACLE";
inline constexpr std::string_view kEStopResponsePrefix = "ESTOP";

} // namespace robot::protocol
