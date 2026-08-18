#include "robot/SerialRobotTransport.hpp"

namespace robot
{

SerialRobotTransport::SerialRobotTransport(ISerialPort& serialPort)
    : serialPort_(serialPort)
{
}

void SerialRobotTransport::sendCommand(const std::string& command)
{
    serialPort_.write(command + "\n");
}

std::string SerialRobotTransport::query(const std::string& request)
{
    serialPort_.write(request + "\n");
    return serialPort_.readLine();
}

} // namespace robot
