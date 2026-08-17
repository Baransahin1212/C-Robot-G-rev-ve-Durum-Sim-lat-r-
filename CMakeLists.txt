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

# --- robot_hardware: IRobotHardware abstraction + SimulatedRobotHardware ---
# Depends only on robot_domain, not on robot_core or robot_scenario.
# RobotStateMachine deliberately does not depend on robot_hardware (and
# vice versa) - connecting FSM states/events to hardware commands is a
# future orchestration layer, not part of this target.
add_library(robot_hardware STATIC
    src/SimulatedRobotHardware.cpp
)

target_link_libraries(robot_hardware PUBLIC robot_domain)

# --- robot_controller: RobotState -> IRobotHardware command translation ---
# Depends on robot_domain (RobotState) and robot_hardware (IRobotHardware).
# RobotStateMachine has no dependency on this target, direct or transitive -
# the controller only observes a resulting RobotState, it is never consulted
# by, or wired into, the FSM itself.
add_library(robot_controller STATIC
    src/RobotController.cpp
)

target_link_libraries(robot_controller PUBLIC
    robot_domain
    robot_hardware
)

# --- robot_hardware_events: HardwareEventSource, sensor state -> Event ---
# Depends on robot_domain (Event/IEventSource) and robot_hardware
# (IRobotHardware). Deliberately independent of robot_core/robot_controller
# - it only ever produces Events; Simulator/RobotStateMachine consume it
# through the existing IEventSource interface exactly like
# JsonScenarioSource, with no FSM-specific knowledge of its own, and it
# never calls an IRobotHardware actuator method.
add_library(robot_hardware_events STATIC
    src/HardwareEventSource.cpp
)

target_link_libraries(robot_hardware_events PUBLIC
    robot_domain
    robot_hardware
)

# Simulator.hpp includes RobotController.hpp so it can accept an optional
# RobotController*, so robot_core needs robot_controller PUBLIC (its
# headers, and anything that includes Simulator.hpp, need to resolve
# RobotController). RobotStateMachine itself still has no such include -
# only Simulator.hpp/.cpp changed.
target_link_libraries(robot_core PUBLIC robot_controller)

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

# --- robot_app: CLI argument parsing + application-run composition ---
# Exists so the command-line logic (parseArguments/runSimulation/
# runApplication) is directly unit-testable - callers pass in vectors of
# strings and output streams instead of going through argv or a subprocess.
# It has no compile-time path configuration of its own; the caller (main.cpp
# for the real CLI, tests for temp directories) supplies logs/reports paths.
add_library(robot_app STATIC
    src/Application.cpp
)

target_link_libraries(robot_app PUBLIC
    robot_core
    robot_scenario
    robot_logging
    robot_reporting
)

# --- RobotSimulator: the command-line application ---
add_executable(RobotSimulator
    src/main.cpp
)

target_link_libraries(RobotSimulator PRIVATE robot_app)

target_compile_definitions(RobotSimulator PRIVATE
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

# --- RobotHardwareTests: SimulatedRobotHardware in isolation ---
add_executable(RobotHardwareTests
    tests/SimulatedRobotHardwareTests.cpp
)

target_link_libraries(RobotHardwareTests PRIVATE
    robot_hardware
    GTest::gtest_main
)

gtest_discover_tests(RobotHardwareTests)

# --- RobotControllerTests: RobotState -> IRobotHardware command mapping,
# exercised against SimulatedRobotHardware as the deterministic test double.
add_executable(RobotControllerTests
    tests/RobotControllerTests.cpp
)

target_link_libraries(RobotControllerTests PRIVATE
    robot_controller
    GTest::gtest_main
)

gtest_discover_tests(RobotControllerTests)

# --- HardwareEventSourceTests: sensor-state -> Event edge/priority/queue
# behavior, plus one Simulator integration test proving HardwareEventSource
# satisfies IEventSource end-to-end. Links robot_core (RobotStateMachine,
# Simulator) in addition to robot_hardware_events for that integration test.
add_executable(HardwareEventSourceTests
    tests/HardwareEventSourceTests.cpp
)

target_link_libraries(HardwareEventSourceTests PRIVATE
    robot_core
    robot_hardware_events
    GTest::gtest_main
)

gtest_discover_tests(HardwareEventSourceTests)

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

# --- ApplicationTests: CLI argument parsing and application-run behavior ---
# Drives robot::app::runApplication() directly with in-memory streams and a
# build-tree-local output directory, so tests never touch the real
# logs/ or reports/ directories and never spawn a subprocess.
add_executable(ApplicationTests
    tests/ApplicationTests.cpp
)

target_link_libraries(ApplicationTests PRIVATE
    robot_app
    GTest::gtest_main
)

target_compile_definitions(ApplicationTests PRIVATE
    SCENARIOS_DIR="${CMAKE_SOURCE_DIR}/scenarios/"
    TEST_FIXTURES_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures/"
    TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output/"
)

gtest_discover_tests(ApplicationTests)

# --- Warnings: scoped to our own targets only, never to FetchContent
# dependencies (GoogleTest, nlohmann/json), so third-party warning noise
# is never our concern to fix or silence.
if (MSVC)
    foreach(robot_target
        robot_core robot_scenario robot_logging robot_reporting robot_app
        robot_hardware robot_controller robot_hardware_events
        RobotSimulator RobotSimulatorTests ScenarioSourceTests
        RobotHardwareTests RobotControllerTests HardwareEventSourceTests
        ScenarioIntegrationTests ApplicationTests
    )
        target_compile_options(${robot_target} PRIVATE /W4)
    endforeach()
endif()
