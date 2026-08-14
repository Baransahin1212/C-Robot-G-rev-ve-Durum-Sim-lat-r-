#include <gtest/gtest.h>

#include <sstream>
#include <string>

#include "robot/Event.hpp"
#include "robot/RobotState.hpp"
#include "robot/StreamSimulationLogger.hpp"

namespace
{

using robot::Event;
using robot::EventType;
using robot::RobotState;
using robot::StreamSimulationLogger;

} // namespace

TEST(StreamSimulationLoggerTest, EventLoggingIncludesTypeAndTimestamp)
{
    // Arrange
    std::ostringstream out;
    StreamSimulationLogger logger(out);
    const Event event{EventType::ObstacleDetected, 2000, std::nullopt};

    // Act
    logger.logEventReceived(event);

    // Assert
    const std::string logText = out.str();
    EXPECT_NE(logText.find("2000"), std::string::npos);
    EXPECT_NE(logText.find("ObstacleDetected"), std::string::npos);
    EXPECT_NE(logText.find("INFO"), std::string::npos);
}

TEST(StreamSimulationLoggerTest, EventValueLoggingIncludesNumericPayload)
{
    // Arrange
    std::ostringstream out;
    StreamSimulationLogger logger(out);
    const Event event{EventType::BatteryCritical, 7000, 8.0};

    // Act
    logger.logEventReceived(event);

    // Assert
    const std::string logText = out.str();
    EXPECT_NE(logText.find("BatteryCritical"), std::string::npos);
    EXPECT_NE(logText.find("value=8"), std::string::npos);
}

TEST(StreamSimulationLoggerTest, SuccessfulTransitionLoggingShowsFromAndToStates)
{
    // Arrange
    std::ostringstream out;
    StreamSimulationLogger logger(out);

    // Act
    logger.logTransitionSucceeded(2000, RobotState::Moving, RobotState::WaitingForObstacleClear);

    // Assert
    const std::string logText = out.str();
    EXPECT_NE(logText.find("Moving"), std::string::npos);
    EXPECT_NE(logText.find("WaitingForObstacleClear"), std::string::npos);
    EXPECT_NE(logText.find("->"), std::string::npos);
    EXPECT_NE(logText.find("2000"), std::string::npos);
}

TEST(StreamSimulationLoggerTest, RejectedTransitionLoggingIsWarningAndIncludesStateAndEvent)
{
    // Arrange
    std::ostringstream out;
    StreamSimulationLogger logger(out);

    // Act
    logger.logTransitionRejected(3000, RobotState::Idle, EventType::MissionCompleted);

    // Assert
    const std::string logText = out.str();
    EXPECT_NE(logText.find("WARNING"), std::string::npos);
    EXPECT_NE(logText.find("Idle"), std::string::npos);
    EXPECT_NE(logText.find("MissionCompleted"), std::string::npos);
    EXPECT_NE(logText.find("3000"), std::string::npos);
}

TEST(StreamSimulationLoggerTest, MultipleEventsAreLoggedInOrder)
{
    // Arrange
    std::ostringstream out;
    StreamSimulationLogger logger(out);
    const Event first{EventType::ScenarioLoaded, 0, std::nullopt};
    const Event second{EventType::StartMission, 100, std::nullopt};

    // Act
    logger.logEventReceived(first);
    logger.logEventReceived(second);

    // Assert
    const std::string logText = out.str();
    const std::size_t firstPos = logText.find("ScenarioLoaded");
    const std::size_t secondPos = logText.find("StartMission");
    ASSERT_NE(firstPos, std::string::npos);
    ASSERT_NE(secondPos, std::string::npos);
    EXPECT_LT(firstPos, secondPos);
}
