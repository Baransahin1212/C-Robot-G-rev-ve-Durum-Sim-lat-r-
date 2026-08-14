cmake_minimum_required(VERSION 3.20)

project(RobotSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

# --- robot_domain: shared domain headers (Event, RobotState, IEventSource) ---
# Header-only, so it is an INTERFACE target with no implementation of its
# own. Both robot_core and robot_scenario depend on it for the shared
# vocabulary types, without depending on each other's implementation.
add_library(robot_domain INTERFACE)

target_include_directories(robot_domain INTERFACE
    ${CMAKE_SOURCE_DIR}/include
)

# --- robot_core: production FSM + orchestration, shared by the app and tests ---
# Simulator lives here (not in robot_scenario) because it only depends on
# domain types and RobotStateMachine - it has no knowledge of JSON at all.
# Deliberately has no JSON dependency.
add_library(robot_core STATIC
    src/RobotStateMachine.cpp
    src/Simulator.cpp
)

target_link_libraries(robot_core PUBLIC robot_domain)

# --- nlohmann/json, pinned to a stable tagged release (not main/master) ---
set(JSON_BuildTests OFF CACHE INTERNAL "")

FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.11.3
)

FetchContent_MakeAvailable(nlohmann_json)

# --- robot_scenario: JSON scenario file -> Event objects ---
# Depends only on robot_domain for the shared Event/IEventSource types, not
# on robot_core's FSM implementation. nlohmann/json is an implementation
# detail (only used inside the .cpp, never in the public header), so it is
# linked PRIVATE and does not leak into anything that links robot_scenario.
add_library(robot_scenario STATIC
    src/JsonScenarioSource.cpp
)

target_link_libraries(robot_scenario
    PUBLIC robot_domain
    PRIVATE nlohmann_json::nlohmann_json
)

# --- robot_logging: concrete ISimulationLogger implementation(s) ---
# Depends only on robot_domain (for ISimulationLogger/Event/RobotState), not
# on robot_core or robot_scenario, and has no JSON dependency.
add_library(robot_logging STATIC
    src/StreamSimulationLogger.cpp
)

target_link_libraries(robot_logging PUBLIC robot_domain)

# --- robot_reporting: MissionOutcome mapping + SimulationReport writer ---
# Depends on robot_core because SimulationReport is derived from
# SimulationResult (defined alongside Simulator). RobotStateMachine and
# JsonScenarioSource never depend on this target - the dependency only
# flows one way, from reporting down to the data it summarizes.
add_library(robot_reporting STATIC
    src/StreamReportWriter.cpp
)

target_link_libraries(robot_reporting PUBLIC robot_core)

# --- RobotSimulator: the command-line application ---
add_executable(RobotSimulator
    src/main.cpp
)

target_link_libraries(RobotSimulator PRIVATE robot_core robot_scenario robot_logging robot_reporting)

target_compile_definitions(RobotSimulator PRIVATE
    SCENARIOS_DIR="${CMAKE_SOURCE_DIR}/scenarios/"
    LOGS_DIR="${CMAKE_SOURCE_DIR}/logs/"
    REPORTS_DIR="${CMAKE_SOURCE_DIR}/reports/"
)

# --- Tests (GoogleTest via FetchContent, pinned to a stable tagged release) ---
enable_testing()

# MSVC default runtime is /MD; force gtest to match so link flags agree.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)

FetchContent_MakeAvailable(googletest)

include(GoogleTest)

add_executable(RobotSimulatorTests
    tests/RobotStateMachineTests.cpp
    tests/SimulatorTests.cpp
    tests/StreamSimulationLoggerTests.cpp
    tests/SimulationReportTests.cpp
)

target_link_libraries(RobotSimulatorTests PRIVATE
    robot_core
    robot_logging
    robot_reporting
    GTest::gtest_main
)

gtest_discover_tests(RobotSimulatorTests)

add_executable(ScenarioSourceTests
    tests/JsonScenarioSourceTests.cpp
)

target_link_libraries(ScenarioSourceTests PRIVATE
    robot_scenario
    GTest::gtest_main
)

target_compile_definitions(ScenarioSourceTests PRIVATE
    TEST_FIXTURES_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures/"
)

gtest_discover_tests(ScenarioSourceTests)

# --- ScenarioIntegrationTests: exercises the real end-to-end pipeline
# (JsonScenarioSource -> Simulator -> RobotStateMachine -> SimulationResult
# -> SimulationReport) against the final scenario files under scenarios/,
# so it needs every production library.
add_executable(ScenarioIntegrationTests
    tests/ScenarioIntegrationTests.cpp
)

target_link_libraries(ScenarioIntegrationTests PRIVATE
    robot_core
    robot_scenario
    robot_logging
    robot_reporting
    GTest::gtest_main
)

target_compile_definitions(ScenarioIntegrationTests PRIVATE
    SCENARIOS_DIR="${CMAKE_SOURCE_DIR}/scenarios/"
)

gtest_discover_tests(ScenarioIntegrationTests)
