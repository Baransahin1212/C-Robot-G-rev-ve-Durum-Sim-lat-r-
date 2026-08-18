#include "robot/RealRobotHardware.hpp"

#include <sstream>
#include <vector>

#include "robot/RobotProtocol.hpp"

namespace robot
{

namespace
{

std::vector<std::string> Tokenize(const std::string& text)
{
    std::vector<std::string> tokens;
    std::istringstream stream(text);
    std::string token;
    while (stream >> token)
    {
        tokens.push_back(token);
    }
    return tokens;
}

// Splits `response` into exactly two whitespace-separated tokens and
// verifies the first equals `expectedPrefix`, returning the second
// (the value token). Covers empty responses, wrong prefixes, missing
// values, and trailing garbage in one place, shared by all three sensor
// parsers below - strict, no partial parsing.
std::string ExtractValueToken(const std::string& response,
                               std::string_view expectedPrefix,
                               std::string_view queryName)
{
    const std::vector<std::string> tokens = Tokenize(response);

    if (tokens.empty())
    {
        throw RobotTransportError(std::string(queryName) + ": empty response");
    }
    if (tokens[0] != expectedPrefix)
    {
        throw RobotTransportError(std::string(queryName) + ": expected prefix \"" + std::string(expectedPrefix) +
                                   "\", got \"" + tokens[0] + "\"");
    }
    if (tokens.size() < 2)
    {
        throw RobotTransportError(std::string(queryName) + ": missing value after \"" +
                                   std::string(expectedPrefix) + "\"");
    }
    if (tokens.size() > 2)
    {
        throw RobotTransportError(std::string(queryName) + ": unexpected trailing tokens in response \"" +
                                   response + "\"");
    }

    return tokens[1];
}

int ParseBatteryValue(const std::string& token)
{
    std::size_t consumed = 0;
    int value = 0;
    try
    {
        value = std::stoi(token, &consumed);
    }
    catch (const std::exception&)
    {
        throw RobotTransportError("GET_BATTERY: invalid battery value \"" + token + "\"");
    }

    if (consumed != token.size())
    {
        throw RobotTransportError("GET_BATTERY: invalid battery value \"" + token + "\"");
    }
    if (value < 0 || value > 100)
    {
        throw RobotTransportError("GET_BATTERY: battery must be between 0 and 100, got \"" + token + "\"");
    }

    return value;
}

bool ParseBooleanValue(const std::string& token, std::string_view queryName)
{
    if (token == "0")
    {
        return false;
    }
    if (token == "1")
    {
        return true;
    }
    throw RobotTransportError(std::string(queryName) + ": expected \"0\" or \"1\", got \"" + token + "\"");
}

} // namespace

RealRobotHardware::RealRobotHardware(IRobotTransport& transport)
    : transport_(transport)
{
}

int RealRobotHardware::batteryLevelPercent() const
{
    const std::string response = transport_.query(std::string(protocol::kGetBattery));
    const std::string valueToken =
        ExtractValueToken(response, protocol::kBatteryResponsePrefix, protocol::kGetBattery);
    return ParseBatteryValue(valueToken);
}

bool RealRobotHardware::obstacleDetected() const
{
    const std::string response = transport_.query(std::string(protocol::kGetObstacle));
    const std::string valueToken =
        ExtractValueToken(response, protocol::kObstacleResponsePrefix, protocol::kGetObstacle);
    return ParseBooleanValue(valueToken, protocol::kGetObstacle);
}

bool RealRobotHardware::emergencyStopPressed() const
{
    const std::string response = transport_.query(std::string(protocol::kGetEStop));
    const std::string valueToken = ExtractValueToken(response, protocol::kEStopResponsePrefix, protocol::kGetEStop);
    return ParseBooleanValue(valueToken, protocol::kGetEStop);
}

void RealRobotHardware::moveForward()
{
    transport_.sendCommand(std::string(protocol::kMoveForward));
}

void RealRobotHardware::stop()
{
    transport_.sendCommand(std::string(protocol::kStop));
}

void RealRobotHardware::returnToBase()
{
    transport_.sendCommand(std::string(protocol::kReturnToBase));
}

} // namespace robot
