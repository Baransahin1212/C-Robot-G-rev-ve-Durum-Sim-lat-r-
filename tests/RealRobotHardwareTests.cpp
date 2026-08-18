#include <gtest/gtest.h>

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "robot/Event.hpp"
#include "robot/HardwareEventSource.hpp"
#include "robot/IRobotTransport.hpp"
#include "robot/RealRobotHardware.hpp"
#include "robot/RobotController.hpp"
#include "robot/RobotState.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::HardwareEventSource;
using robot::IRobotTransport;
using robot::RealRobotHardware;
using robot::RobotController;
using robot::RobotState;
using robot::RobotTransportError;

// Test-only IRobotTransport - deliberately not a production fake (see
// docs/technical-decisions.md, Phase 13K): records every command sent and
// every request queried, and returns a caller-configured response per
// request string (empty string if none was configured, which exercises
// RealRobotHardware's "empty response" rejection path for free).
class RecordingRobotTransport : public IRobotTransport
{
public:
    void sendCommand(const std::string& command) override
    {
        sentCommands.push_back(command);
    }

    std::string query(const std::string& request) override
    {
        queries.push_back(request);
        const auto it = responses.find(request);
        if (it == responses.end())
        {
            return {};
        }
        return it->second;
    }

    std::vector<std::string> sentCommands;
    std::vector<std::string> queries;
    std::map<std::string, std::string> responses;
};

} // namespace

// --- Actuator commands (section 8) ---

TEST(RealRobotHardwareTest, MoveForwardSendsExpectedTransportCommand)
{
    // Arrange
    RecordingRobotTransport transport;
    RealRobotHardware hardware(transport);

    // Act
    hardware.moveForward();

    // Assert
    ASSERT_EQ(transport.sentCommands.size(), 1u);
    EXPECT_EQ(transport.sentCommands[0], "MOVE_FORWARD");
}

TEST(RealRobotHardwareTest, StopSendsExpectedTransportCommand)
{
    // Arrange
    RecordingRobotTransport transport;
    RealRobotHardware hardware(transport);

    // Act
    hardware.stop();

    // Assert
    ASSERT_EQ(transport.sentCommands.size(), 1u);
    EXPECT_EQ(transport.sentCommands[0], "STOP");
}

TEST(RealRobotHardwareTest, ReturnToBaseSendsExpectedTransportCommand)
{
    // Arrange
    RecordingRobotTransport transport;
    RealRobotHardware hardware(transport);

    // Act
    hardware.returnToBase();

    // Assert
    ASSERT_EQ(transport.sentCommands.size(), 1u);
    EXPECT_EQ(transport.sentCommands[0], "RETURN_TO_BASE");
}

// --- Sensor reads (section 9) ---

TEST(RealRobotHardwareTest, BatteryQueryReturnsParsedPercentage)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_BATTERY"] = "BATTERY 78";
    RealRobotHardware hardware(transport);

    // Act
    const int battery = hardware.batteryLevelPercent();

    // Assert
    EXPECT_EQ(battery, 78);
    ASSERT_EQ(transport.queries.size(), 1u);
    EXPECT_EQ(transport.queries[0], "GET_BATTERY");
}

TEST(RealRobotHardwareTest, ObstacleQueryReturnsFalse)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_OBSTACLE"] = "OBSTACLE 0";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_FALSE(hardware.obstacleDetected());
    ASSERT_EQ(transport.queries.size(), 1u);
    EXPECT_EQ(transport.queries[0], "GET_OBSTACLE");
}

TEST(RealRobotHardwareTest, ObstacleQueryReturnsTrue)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_OBSTACLE"] = "OBSTACLE 1";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_TRUE(hardware.obstacleDetected());
}

TEST(RealRobotHardwareTest, EmergencyStopQueryReturnsFalse)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_ESTOP"] = "ESTOP 0";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_FALSE(hardware.emergencyStopPressed());
    ASSERT_EQ(transport.queries.size(), 1u);
    EXPECT_EQ(transport.queries[0], "GET_ESTOP");
}

TEST(RealRobotHardwareTest, EmergencyStopQueryReturnsTrue)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_ESTOP"] = "ESTOP 1";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_TRUE(hardware.emergencyStopPressed());
}

// --- Invalid responses (section 10) ---

TEST(RealRobotHardwareTest, BatteryBelowZeroRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_BATTERY"] = "BATTERY -1";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.batteryLevelPercent(), RobotTransportError);
}

TEST(RealRobotHardwareTest, BatteryAboveHundredRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_BATTERY"] = "BATTERY 101";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.batteryLevelPercent(), RobotTransportError);
}

TEST(RealRobotHardwareTest, BatteryNonNumericRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_BATTERY"] = "BATTERY abc";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.batteryLevelPercent(), RobotTransportError);
}

TEST(RealRobotHardwareTest, BatteryWrongPrefixRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_BATTERY"] = "WRONG 50";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.batteryLevelPercent(), RobotTransportError);
}

TEST(RealRobotHardwareTest, BatteryTrailingGarbageRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_BATTERY"] = "BATTERY 50 extra";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.batteryLevelPercent(), RobotTransportError);
}

TEST(RealRobotHardwareTest, BatteryMissingValueRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_BATTERY"] = "BATTERY";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.batteryLevelPercent(), RobotTransportError);
}

TEST(RealRobotHardwareTest, ObstacleInvalidValueRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_OBSTACLE"] = "OBSTACLE 2";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.obstacleDetected(), RobotTransportError);
}

TEST(RealRobotHardwareTest, ObstacleWrongPrefixRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_OBSTACLE"] = "WRONG 1";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.obstacleDetected(), RobotTransportError);
}

TEST(RealRobotHardwareTest, ObstacleTrailingGarbageRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_OBSTACLE"] = "OBSTACLE 1 extra";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.obstacleDetected(), RobotTransportError);
}

TEST(RealRobotHardwareTest, EmergencyInvalidValueRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_ESTOP"] = "ESTOP yes";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.emergencyStopPressed(), RobotTransportError);
}

TEST(RealRobotHardwareTest, EmergencyWrongPrefixRejected)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_ESTOP"] = "WRONG 0";
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.emergencyStopPressed(), RobotTransportError);
}

TEST(RealRobotHardwareTest, EmptyResponseRejected)
{
    // Arrange: no response configured for GET_BATTERY - the transport
    // fake returns "" for any unconfigured request.
    RecordingRobotTransport transport;
    RealRobotHardware hardware(transport);

    // Act / Assert
    EXPECT_THROW(hardware.batteryLevelPercent(), RobotTransportError);
}

// --- RobotController integration (section 11) ---
//
// Proves the existing, unmodified RobotController drives RealRobotHardware
// correctly through IRobotHardware alone - RobotController never knows
// which concrete implementation it was given.
TEST(RealRobotHardwareTest, RobotControllerDrivesRealRobotHardwareThroughIRobotHardware)
{
    // Arrange
    RecordingRobotTransport transport;
    RealRobotHardware hardware(transport);
    RobotController controller(hardware);

    // Act / Assert: Moving -> MOVE_FORWARD
    controller.applyState(RobotState::Moving);
    ASSERT_FALSE(transport.sentCommands.empty());
    EXPECT_EQ(transport.sentCommands.back(), "MOVE_FORWARD");

    // Act / Assert: WaitingForObstacleClear -> STOP
    controller.applyState(RobotState::WaitingForObstacleClear);
    EXPECT_EQ(transport.sentCommands.back(), "STOP");

    // Act / Assert: ReturningHome -> RETURN_TO_BASE
    controller.applyState(RobotState::ReturningHome);
    EXPECT_EQ(transport.sentCommands.back(), "RETURN_TO_BASE");
}

// --- HardwareEventSource integration (section 12) ---
//
// Proves the existing, unmodified HardwareEventSource can read
// RealRobotHardware through IRobotHardware exactly as it reads
// SimulatedRobotHardware: Transport -> RealRobotHardware ->
// HardwareEventSource -> Event, with no HardwareEventSource change at all.
TEST(RealRobotHardwareTest, HardwareEventSourceDetectsObstacleThroughRealRobotHardware)
{
    // Arrange
    RecordingRobotTransport transport;
    transport.responses["GET_BATTERY"] = "BATTERY 100";
    transport.responses["GET_OBSTACLE"] = "OBSTACLE 0";
    transport.responses["GET_ESTOP"] = "ESTOP 0";
    RealRobotHardware hardware(transport);
    HardwareEventSource source(hardware);

    // Act / Assert: baseline - no event.
    EXPECT_FALSE(source.nextEvent().has_value());

    // Act: obstacle appears on the far end of the transport.
    transport.responses["GET_OBSTACLE"] = "OBSTACLE 1";
    const std::optional<Event> event = source.nextEvent();

    // Assert
    ASSERT_TRUE(event.has_value());
    EXPECT_EQ(event->type, EventType::ObstacleDetected);
}
