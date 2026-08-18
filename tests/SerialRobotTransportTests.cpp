#include <gtest/gtest.h>

#include <deque>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot/Event.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/ISerialPort.hpp"
#include "robot/RealRobotHardware.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotState.hpp"
#include "robot/SerialRobotTransport.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::HardwareEventSource;
using robot::ISerialPort;
using robot::RealRobotHardware;
using robot::RobotController;
using robot::RobotState;
using robot::SerialRobotTransport;

// Test-only ISerialPort - deliberately not a production fake (see
// docs/technical-decisions.md, Phase 13L): records every write() call
// verbatim and returns a caller-queued line per readLine() call, in FIFO
// order regardless of what was written before it. callOrder additionally
// interleaves both kinds of calls, for tests that care about write-before-
// read ordering.
class RecordingSerialPort : public ISerialPort
{
public:
    void write(const std::string& data) override
    {
        writes.push_back(data);
        callOrder.push_back("write:" + data);
    }

    std::string readLine() override
    {
        ++readLineCallCount;
        callOrder.push_back("readLine");
        if (linesToRead.empty())
        {
            return {};
        }
        const std::string line = linesToRead.front();
        linesToRead.pop_front();
        return line;
    }

    std::vector<std::string> writes;
    std::deque<std::string> linesToRead;
    std::size_t readLineCallCount = 0;
    std::vector<std::string> callOrder;
};

// Test-only ISerialPort that always throws, for the error-propagation
// tests - SerialRobotTransport must not catch and convert these.
class ThrowingSerialPort : public ISerialPort
{
public:
    void write(const std::string&) override
    {
        throw std::runtime_error("write failed");
    }

    std::string readLine() override
    {
        throw std::runtime_error("readLine failed");
    }
};

} // namespace

// --- SerialRobotTransport unit tests (section 8) ---

// 1: SendCommandAppendsNewline
TEST(SerialRobotTransportTest, SendCommandAppendsNewline)
{
    // Arrange
    RecordingSerialPort serial;
    SerialRobotTransport transport(serial);

    // Act
    transport.sendCommand("MOVE_FORWARD");

    // Assert
    ASSERT_EQ(serial.writes.size(), 1u);
    EXPECT_EQ(serial.writes[0], "MOVE_FORWARD\n");
}

// 2: StopCommandFraming
TEST(SerialRobotTransportTest, StopCommandFraming)
{
    // Arrange
    RecordingSerialPort serial;
    SerialRobotTransport transport(serial);

    // Act
    transport.sendCommand("STOP");

    // Assert
    ASSERT_EQ(serial.writes.size(), 1u);
    EXPECT_EQ(serial.writes[0], "STOP\n");
}

// 3: ReturnToBaseCommandFraming
TEST(SerialRobotTransportTest, ReturnToBaseCommandFraming)
{
    // Arrange
    RecordingSerialPort serial;
    SerialRobotTransport transport(serial);

    // Act
    transport.sendCommand("RETURN_TO_BASE");

    // Assert
    ASSERT_EQ(serial.writes.size(), 1u);
    EXPECT_EQ(serial.writes[0], "RETURN_TO_BASE\n");
}

// 4: QueryWritesRequestWithNewline
TEST(SerialRobotTransportTest, QueryWritesRequestWithNewline)
{
    // Arrange
    RecordingSerialPort serial;
    serial.linesToRead.push_back("BATTERY 78");
    SerialRobotTransport transport(serial);

    // Act
    transport.query("GET_BATTERY");

    // Assert
    ASSERT_EQ(serial.writes.size(), 1u);
    EXPECT_EQ(serial.writes[0], "GET_BATTERY\n");
}

// 5: QueryReturnsReadLineResult
TEST(SerialRobotTransportTest, QueryReturnsReadLineResult)
{
    // Arrange
    RecordingSerialPort serial;
    serial.linesToRead.push_back("BATTERY 78");
    SerialRobotTransport transport(serial);

    // Act
    const std::string result = transport.query("GET_BATTERY");

    // Assert
    EXPECT_EQ(result, "BATTERY 78");
}

// 6: QueryPerformsWriteBeforeRead
TEST(SerialRobotTransportTest, QueryPerformsWriteBeforeRead)
{
    // Arrange
    RecordingSerialPort serial;
    serial.linesToRead.push_back("BATTERY 78");
    SerialRobotTransport transport(serial);

    // Act
    transport.query("GET_BATTERY");

    // Assert
    ASSERT_EQ(serial.callOrder.size(), 2u);
    EXPECT_EQ(serial.callOrder[0], "write:GET_BATTERY\n");
    EXPECT_EQ(serial.callOrder[1], "readLine");
}

// 7: MultipleCommandsPreserveOrder
TEST(SerialRobotTransportTest, MultipleCommandsPreserveOrder)
{
    // Arrange
    RecordingSerialPort serial;
    SerialRobotTransport transport(serial);

    // Act
    transport.sendCommand("MOVE_FORWARD");
    transport.sendCommand("STOP");
    transport.sendCommand("RETURN_TO_BASE");

    // Assert
    ASSERT_EQ(serial.writes.size(), 3u);
    EXPECT_EQ(serial.writes[0], "MOVE_FORWARD\n");
    EXPECT_EQ(serial.writes[1], "STOP\n");
    EXPECT_EQ(serial.writes[2], "RETURN_TO_BASE\n");
}

// 8: MultipleQueriesPreserveOrder
TEST(SerialRobotTransportTest, MultipleQueriesPreserveOrder)
{
    // Arrange
    RecordingSerialPort serial;
    serial.linesToRead.push_back("BATTERY 100");
    serial.linesToRead.push_back("OBSTACLE 0");
    serial.linesToRead.push_back("ESTOP 0");
    SerialRobotTransport transport(serial);

    // Act
    const std::string first = transport.query("GET_BATTERY");
    const std::string second = transport.query("GET_OBSTACLE");
    const std::string third = transport.query("GET_ESTOP");

    // Assert
    EXPECT_EQ(first, "BATTERY 100");
    EXPECT_EQ(second, "OBSTACLE 0");
    EXPECT_EQ(third, "ESTOP 0");
    ASSERT_EQ(serial.writes.size(), 3u);
    EXPECT_EQ(serial.writes[0], "GET_BATTERY\n");
    EXPECT_EQ(serial.writes[1], "GET_OBSTACLE\n");
    EXPECT_EQ(serial.writes[2], "GET_ESTOP\n");
}

// 9: EmptyResponseIsReturnedUnchanged
TEST(SerialRobotTransportTest, EmptyResponseIsReturnedUnchanged)
{
    // Arrange: no line configured - readLine() returns "".
    RecordingSerialPort serial;
    SerialRobotTransport transport(serial);

    // Act
    const std::string result = transport.query("GET_BATTERY");

    // Assert: SerialRobotTransport invents no error of its own here -
    // RealRobotHardware's RobotTransportError handles this later.
    EXPECT_EQ(result, "");
}

// 10: ResponseWhitespaceIsNotNormalized
TEST(SerialRobotTransportTest, ResponseWhitespaceIsNotNormalized)
{
    // Arrange
    RecordingSerialPort serial;
    serial.linesToRead.push_back(" BATTERY 78 ");
    SerialRobotTransport transport(serial);

    // Act
    const std::string result = transport.query("GET_BATTERY");

    // Assert: returned exactly as readLine() produced it, not trimmed.
    EXPECT_EQ(result, " BATTERY 78 ");
}

// Error propagation (section 6) - not caught/converted, propagated as-is.
TEST(SerialRobotTransportTest, WriteExceptionPropagatesUnchanged)
{
    // Arrange
    ThrowingSerialPort serial;
    SerialRobotTransport transport(serial);

    // Act / Assert
    EXPECT_THROW(transport.sendCommand("MOVE_FORWARD"), std::runtime_error);
}

TEST(SerialRobotTransportTest, ReadLineExceptionPropagatesUnchanged)
{
    // Arrange
    ThrowingSerialPort serial;
    SerialRobotTransport transport(serial);

    // Act / Assert
    EXPECT_THROW(transport.query("GET_BATTERY"), std::runtime_error);
}

// --- RealRobotHardware + serial integration (section 9) ---

// A: BatteryQueryWorksThroughSerialTransport
TEST(SerialRobotTransportTest, BatteryQueryWorksThroughSerialTransport)
{
    // Arrange
    RecordingSerialPort serial;
    serial.linesToRead.push_back("BATTERY 78");
    SerialRobotTransport transport(serial);
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_EQ(hardware.batteryLevelPercent(), 78);
    ASSERT_EQ(serial.writes.size(), 1u);
    EXPECT_EQ(serial.writes[0], "GET_BATTERY\n");
}

// B: ObstacleQueryWorksThroughSerialTransport
TEST(SerialRobotTransportTest, ObstacleQueryWorksThroughSerialTransport)
{
    // Arrange
    RecordingSerialPort serial;
    serial.linesToRead.push_back("OBSTACLE 1");
    SerialRobotTransport transport(serial);
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_TRUE(hardware.obstacleDetected());
    ASSERT_EQ(serial.writes.size(), 1u);
    EXPECT_EQ(serial.writes[0], "GET_OBSTACLE\n");
}

// C: EmergencyStopQueryWorksThroughSerialTransport
TEST(SerialRobotTransportTest, EmergencyStopQueryWorksThroughSerialTransport)
{
    // Arrange
    RecordingSerialPort serial;
    serial.linesToRead.push_back("ESTOP 1");
    SerialRobotTransport transport(serial);
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_TRUE(hardware.emergencyStopPressed());
    ASSERT_EQ(serial.writes.size(), 1u);
    EXPECT_EQ(serial.writes[0], "GET_ESTOP\n");
}

// D: MoveForwardTravelsThroughSerialTransport
TEST(SerialRobotTransportTest, MoveForwardTravelsThroughSerialTransport)
{
    // Arrange
    RecordingSerialPort serial;
    SerialRobotTransport transport(serial);
    RealRobotHardware hardware(transport);

    // Act
    hardware.moveForward();

    // Assert
    ASSERT_EQ(serial.writes.size(), 1u);
    EXPECT_EQ(serial.writes[0], "MOVE_FORWARD\n");
}

// E: StopTravelsThroughSerialTransport
TEST(SerialRobotTransportTest, StopTravelsThroughSerialTransport)
{
    // Arrange
    RecordingSerialPort serial;
    SerialRobotTransport transport(serial);
    RealRobotHardware hardware(transport);

    // Act
    hardware.stop();

    // Assert
    ASSERT_EQ(serial.writes.size(), 1u);
    EXPECT_EQ(serial.writes[0], "STOP\n");
}

// F: ReturnToBaseTravelsThroughSerialTransport
TEST(SerialRobotTransportTest, ReturnToBaseTravelsThroughSerialTransport)
{
    // Arrange
    RecordingSerialPort serial;
    SerialRobotTransport transport(serial);
    RealRobotHardware hardware(transport);

    // Act
    hardware.returnToBase();

    // Assert
    ASSERT_EQ(serial.writes.size(), 1u);
    EXPECT_EQ(serial.writes[0], "RETURN_TO_BASE\n");
}

// --- End-to-end HardwareEventSource integration (section 10) ---
//
// RecordingSerialPort -> SerialRobotTransport -> RealRobotHardware ->
// HardwareEventSource -> Event, with HardwareEventSource itself
// unmodified. HardwareEventSource::sampleAndEnqueueEdges() reads sensors
// in this exact order - emergencyStopPressed(), then
// batteryLevelPercent(), then obstacleDetected() (see
// HardwareEventSource.cpp) - so each "fresh sample" corresponds to three
// serial reads in ESTOP/BATTERY/OBSTACLE order; the queued lines below
// must match that order or this test would read the wrong value for the
// wrong sensor.
TEST(SerialRobotTransportTest, HardwareEventSourceDetectsObstacleThroughSerialTransport)
{
    // Arrange: sample 1 (baseline, all safe).
    RecordingSerialPort serial;
    serial.linesToRead.push_back("ESTOP 0");
    serial.linesToRead.push_back("BATTERY 100");
    serial.linesToRead.push_back("OBSTACLE 0");
    // Sample 2: obstacle has become true.
    serial.linesToRead.push_back("ESTOP 0");
    serial.linesToRead.push_back("BATTERY 100");
    serial.linesToRead.push_back("OBSTACLE 1");

    SerialRobotTransport transport(serial);
    RealRobotHardware hardware(transport);
    HardwareEventSource source(hardware);

    // Act / Assert: baseline - no event.
    EXPECT_FALSE(source.nextEvent().has_value());

    // Act
    const std::optional<Event> event = source.nextEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::ObstacleDetected);
}

// --- RobotController serial integration (section 11) ---

TEST(SerialRobotTransportTest, RobotControllerAppliesStateThroughSerialTransport)
{
    // Arrange
    RecordingSerialPort serial;
    SerialRobotTransport transport(serial);
    RealRobotHardware hardware(transport);
    RobotController controller(hardware);

    // Act
    controller.applyState(RobotState::Moving);

    // Assert
    ASSERT_FALSE(serial.writes.empty());
    EXPECT_EQ(serial.writes.back(), "MOVE_FORWARD\n");
}
