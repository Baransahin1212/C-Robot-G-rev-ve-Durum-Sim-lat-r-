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

# --- robot_transport: IRobotTransport abstraction + RobotProtocol wire-
# protocol constants (Phase 13K) ---
# Header-only (IRobotTransport is pure virtual; RobotProtocol.hpp is only
# constexpr string_view constants - neither has a .cpp), so this is an
# INTERFACE target, matching robot_domain's own pattern. Depends on
# robot_domain purely for the shared include directory - neither header
# uses any robot_domain type (Event/RobotState/IEventSource).
# IRobotTransport knows only std::string request/response messages, never
# RobotState, Event, RobotController, or RobotRuntime.
add_library(robot_transport INTERFACE)

target_link_libraries(robot_transport INTERFACE
    robot_domain
)

# --- robot_real_hardware: RealRobotHardware, an IRobotHardware
# implementation that drives a future physical robot through
# IRobotTransport instead of SimulatedRobotHardware's in-memory state
# (Phase 13K) ---
# Depends on robot_hardware (IRobotHardware) and robot_transport
# (IRobotTransport/RobotProtocol). Deliberately independent of robot_core/
# robot_controller/robot_runtime/robot_app - it implements IRobotHardware
# only, with no knowledge of RobotState, Event, RobotStateMachine,
# RobotController, RobotRuntime, CommandScript, or SensorScript. No serial-
# port or other platform-specific transport is implemented yet, and
# RobotSimulator/robot_app do not link this target - the CLI continues to
# use SimulatedRobotHardware exclusively; see docs/technical-decisions.md.
add_library(robot_real_hardware STATIC
    src/RealRobotHardware.cpp
)

target_link_libraries(robot_real_hardware PUBLIC
    robot_hardware
    robot_transport
)

# --- robot_serial_transport: SerialRobotTransport + ISerialPort, the
# line-oriented serial framing layer under IRobotTransport (Phase 13L) ---
# Depends only on robot_transport (IRobotTransport). ISerialPort lives in
# this target's public include surface (header-only, no .cpp of its own -
# it is pure virtual) rather than a separate target, since it has exactly
# one consumer (SerialRobotTransport) and splitting it out further would be
# fragmentation without a decoupling benefit. Deliberately independent of
# robot_hardware/robot_real_hardware/robot_app - SerialRobotTransport only
# ever frames whatever command/query string IRobotTransport's contract
# hands it (appends '\n'), with no knowledge of RobotProtocol's specific
# strings, IRobotHardware, RobotState, Event, RobotController, or
# RobotRuntime. No OS-level ISerialPort implementation exists yet - only a
# test-only fake in tests/SerialRobotTransportTests.cpp.
add_library(robot_serial_transport STATIC
    src/SerialRobotTransport.cpp
)

target_link_libraries(robot_serial_transport PUBLIC
    robot_transport
)

# Simulator.hpp includes RobotController.hpp so it can accept an optional
# RobotController*, so robot_core needs robot_controller PUBLIC (its
# headers, and anything that includes Simulator.hpp, need to resolve
# RobotController). RobotStateMachine itself still has no such include -
# only Simulator.hpp/.cpp changed.
target_link_libraries(robot_core PUBLIC robot_controller)

# --- robot_runtime: RobotRuntime, one live-polling cycle per step() ---
# Depends on robot_domain (IPollingEventSource), robot_core
# (RobotStateMachine), and robot_controller (RobotController). Deliberately
# depends on the IPollingEventSource interface only, not on any concrete
# polling source - robot_hardware_events (HardwareEventSource) is not a
# dependency of this target, only of its tests.
add_library(robot_runtime STATIC
    src/RobotRuntime.cpp
)

target_link_libraries(robot_runtime PUBLIC
    robot_domain
    robot_core
    robot_controller
)

# --- robot_runtime_runner: LiveRuntimeRunner, deterministic finite cycle
# scheduler on top of RobotRuntime::step() ---
# Depends only on robot_runtime (RobotRuntime, RuntimeStepResult).
# Deliberately does not depend on robot_hardware, robot_hardware_events, or
# robot_controller directly - LiveRuntimeRunner only ever calls
# RobotRuntime::step() and classifies the result, with no knowledge of
# sensors, actuators, or FSM internals.
add_library(robot_runtime_runner STATIC
    src/LiveRuntimeRunner.cpp
)

target_link_libraries(robot_runtime_runner PUBLIC
    robot_runtime
)

# --- robot_sensor_script: SensorScript, deterministic sensor-injection
# script parsing (Phase 13I) ---
# Depends only on robot_domain (for the include directory - SensorScript
# itself uses no domain types, only the standard library) and is
# deliberately independent of robot_hardware/robot_runtime/robot_app: it
# only ever parses a script file into SensorScriptEntry values, with no
# knowledge of SimulatedRobotHardware or cycle execution.
add_library(robot_sensor_script STATIC
    src/SensorScript.cpp
)

target_link_libraries(robot_sensor_script PUBLIC
    robot_domain
)

# --- robot_scripted_runtime: ScriptedLiveRuntimeRunner, applies a
# SensorScript's mutations to SimulatedRobotHardware immediately before
# each RobotRuntime::step() (Phase 13I) ---
# Depends on robot_runtime (RobotRuntime/RuntimeStepResult),
# robot_runtime_runner (reuses RuntimeRunSummary), robot_hardware
# (SimulatedRobotHardware), and robot_sensor_script (SensorScript). All
# PUBLIC because every one of those types appears in
# ScriptedLiveRuntimeRunner.hpp's public API. Deliberately does not modify
# or depend on anything inside robot_runtime/robot_runtime_runner - this is
# a simulator-only adapter layered on top of them.
add_library(robot_scripted_runtime STATIC
    src/ScriptedLiveRuntimeRunner.cpp
)

target_link_libraries(robot_scripted_runtime PUBLIC
    robot_runtime
    robot_runtime_runner
    robot_hardware
    robot_sensor_script
)

# --- robot_command_script: CommandScript, deterministic mission-command
# script parsing (Phase 13J) ---
# Depends only on robot_domain (for the include directory and EventType,
# from Event.hpp). Deliberately independent of robot_sensor_script/
# robot_hardware/robot_app: it only ever parses a script file into
# CommandScriptEntry values (cycle + one of the existing EventType values
# that represent mission-lifecycle/operator commands), with no knowledge of
# sensors or cycle execution.
add_library(robot_command_script STATIC
    src/CommandScript.cpp
)

target_link_libraries(robot_command_script PUBLIC
    robot_domain
)

# --- robot_command_events: ScriptedCommandEventSource, turns a
# CommandScript's entries into an IPollingEventSource (Phase 13J) ---
# Depends on robot_domain (IPollingEventSource/Event) and
# robot_command_script (CommandScript). Deliberately independent of
# robot_hardware/robot_runtime: it never reads IRobotHardware and knows
# nothing about RobotRuntime - it only turns eligible CommandScriptEntry
# values into Event values on the cycle schedule it is externally driven
# with (setCurrentCycle()).
add_library(robot_command_events STATIC
    src/ScriptedCommandEventSource.cpp
)

target_link_libraries(robot_command_events PUBLIC
    robot_domain
    robot_command_script
)

# --- robot_polling_composite: CompositePollingEventSource, combines two
# IPollingEventSource instances (command-priority-first) into the one
# RobotRuntime accepts (Phase 13J) ---
# Depends only on robot_domain (IPollingEventSource/Event). Deliberately
# independent of robot_command_events/robot_hardware_events: it only ever
# calls pollEvent() on the two IPollingEventSource references it was given,
# with no knowledge of CommandScript, SensorScript, or IRobotHardware.
add_library(robot_polling_composite STATIC
    src/CompositePollingEventSource.cpp
)

target_link_libraries(robot_polling_composite PUBLIC
    robot_domain
)

# Phase 13J: ScriptedLiveRuntimeRunner gained constructors that accept a
# ScriptedCommandEventSource&, so robot_scripted_runtime now additionally
# needs robot_command_events PUBLIC (the type appears in
# ScriptedLiveRuntimeRunner.hpp's public API).
target_link_libraries(robot_scripted_runtime PUBLIC
    robot_command_events
)

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

# Live-mode composition (runLiveSimulation) constructs HardwareEventSource,
# RobotRuntime, and LiveRuntimeRunner internally. None of those types appear
# in Application.hpp, but - matching robot_core/robot_scenario/robot_logging/
# robot_reporting above - they must still be PUBLIC: robot_app is a STATIC
# library, and a STATIC library's PRIVATE link dependencies are not added to
# the final executable's link line, so a PRIVATE dependency here would leave
# RobotSimulator/ApplicationTests unable to resolve these symbols.
target_link_libraries(robot_app PUBLIC
    robot_hardware_events
    robot_runtime
    robot_runtime_runner
)

# Phase 13I: --sensor-script support. Same PUBLIC/STATIC-library
# propagation reasoning as immediately above.
target_link_libraries(robot_app PUBLIC
    robot_sensor_script
    robot_scripted_runtime
)

# Phase 13J: --command-script support. Same PUBLIC/STATIC-library
# propagation reasoning as immediately above.
target_link_libraries(robot_app PUBLIC
    robot_command_script
    robot_command_events
    robot_polling_composite
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

# --- raylib, pinned to the 6.0 tagged release (not master) - dependency
# of the visual simulator only (Phase 13M) ---
# No other target links raylib, and it must never leak into
# robot_core/robot_domain/robot_hardware/robot_app - the dependency points
# inward, robot_visual -> raylib, never the reverse. See
# docs/technical-decisions.md.
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 6.0
)

FetchContent_MakeAvailable(raylib)

# --- robot_visual_world: VirtualWorld, plain world/scene data (Phase 13M) ---
# Deliberately raylib-free: VirtualWorld/RobotPose/BoxObstacle/
# BasePlatform use only this project's own Vec3, never raylib's Vector3,
# so this target - and anything testing it - never needs to link raylib.
# Depends only on robot_domain for the shared include directory; it uses
# no domain type (Event/RobotState/...).
add_library(robot_visual_world STATIC
    src/visual/VirtualWorld.cpp
)

target_link_libraries(robot_visual_world PUBLIC
    robot_domain
)

# --- robot_visual: VisualRobot + Renderer3D, the raylib-based 3D drawing
# layer (Phase 13M) ---
# Depends on robot_visual_world (VirtualWorld/RobotPose/BoxObstacle/
# BasePlatform) and raylib. This is the only point where this project
# depends on raylib. Has no knowledge of RobotStateMachine, RobotRuntime,
# or RobotController - it only ever reads VirtualWorld's plain data.
add_library(robot_visual STATIC
    src/visual/VisualRobot.cpp
    src/visual/Renderer3D.cpp
)

target_link_libraries(robot_visual PUBLIC
    robot_visual_world
    raylib
)

# --- robot_visual_simulation: VirtualRobotHardware + VirtualDistanceSensor,
# the FSM-driven actuator/sensor boundary for the visual simulator (Phase
# 13N/13O) ---
# Depends on robot_visual_world (VirtualWorld), robot_hardware
# (IRobotHardware), robot_controller (linked transitively via
# robot_runtime, but named explicitly for clarity), and robot_runtime
# (RobotRuntime/IPollingEventSource) - the same real FSM/runtime/
# controller stack the CLI uses, never a raylib-aware substitute.
# VirtualDistanceSensor (Phase 13O) is a pure VirtualWorld-geometry query
# with no FSM/Event knowledge of its own - it lives here (not
# robot_visual_world) because it is conceptually part of the virtual
# hardware's sensor boundary, exactly like VirtualRobotHardware itself.
# Deliberately independent of robot_visual: neither has a raylib dependency
# of its own, matching SimulatedRobotHardware/RealRobotHardware's own
# boundaries. robot_visual and robot_visual_simulation are sibling targets
# under RobotSimulator3D, neither depending on the other - only
# raylib-drawing code needs raylib, and only FSM-driving code needs the
# robot core/runtime stack. DifferentialDrive (Phase 13P) is the raylib-free
# kinematic model VirtualRobotHardware now delegates movement to - it lives
# here rather than robot_visual_world since it has no VirtualWorld
# mutation/query behavior of its own, only RobotPose in/out. RobotCollision
# (Phase 13P) is the raylib-free circle-vs-AABB obstacle collision query
# VirtualRobotHardware::update() validates a proposed pose against before
# committing it - a read-only query over VirtualWorld's existing obstacle
# data, exactly like VirtualDistanceSensor. ManualDriveInput (Phase 13P) is
# the pure X/UP/DOWN/LEFT/RIGHT -> WheelSpeeds decision function behind
# RobotSimulator3D's manual drive mode, kept here so it is unit-testable
# without raylib even though its only caller is main3d.cpp.
# ReactiveObstacleAvoidance (Phase 13Q; stateful latch as of Phase 13R) is
# the raylib-free "what wheel speeds does an avoidance turn use, and is
# the turn still active" policy behind RobotSimulator3D's autonomous
# obstacle avoidance - it has no FSM/sensor/clearance knowledge of its
# own; the decision of *when* to trigger/release it lives in main3d.cpp,
# exactly like ManualDriveInput's decision function. ForwardClearanceProbe
# (Phase 13R) is the raylib-free swept-body forward-corridor clearance
# query - expanded-AABB-vs-segment, using RobotCollision's own collision
# radius as its source of truth - that drives avoidance's release
# condition; it answers a different question than VirtualDistanceSensor's
# single-point ray (perception) and is never a substitute for
# RobotCollision's own unconditional final penetration guard.
add_library(robot_visual_simulation STATIC
    src/visual/VirtualRobotHardware.cpp
    src/visual/VirtualDistanceSensor.cpp
    src/visual/DifferentialDrive.cpp
    src/visual/RobotCollision.cpp
    src/visual/ManualDriveInput.cpp
    src/visual/ReactiveObstacleAvoidance.cpp
    src/visual/ForwardClearanceProbe.cpp
)

target_link_libraries(robot_visual_simulation PUBLIC
    robot_visual_world
    robot_hardware
    robot_controller
    robot_runtime
    robot_domain
)

# --- RobotSimulator3D: the interactive 3D visual simulator executable ---
# Deliberately separate from RobotSimulator (the CLI) - RobotSimulator
# never links robot_visual/robot_visual_simulation or raylib. As of Phase
# 13N, main3d.cpp constructs the real RobotStateMachine/RobotController/
# RobotRuntime (via robot_visual_simulation) to drive the on-screen robot
# through VirtualRobotHardware; robot_visual (raylib) remains purely the
# drawing layer, reading only VirtualWorld's plain data. As of Phase 13O,
# main3d.cpp also constructs the real HardwareEventSource and
# CompositePollingEventSource - the same live-event composition
# Application::runLiveSimulation() uses for the CLI's --live mode - so
# obstacle detection flows through the unmodified production event chain;
# hence the direct robot_hardware_events/robot_polling_composite link below
# (robot_visual_simulation itself does not need them - only main3d.cpp's
# translation unit does).
add_executable(RobotSimulator3D
    src/visual/main3d.cpp
)

target_link_libraries(RobotSimulator3D PRIVATE
    robot_visual
    robot_visual_simulation
    robot_hardware_events
    robot_polling_composite
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

# --- RobotRuntimeTests: one-cycle live polling orchestration (step()),
# using both a small test-only IPollingEventSource fake and the real
# HardwareEventSource for genuine live-integration coverage.
add_executable(RobotRuntimeTests
    tests/RobotRuntimeTests.cpp
)

target_link_libraries(RobotRuntimeTests PRIVATE
    robot_runtime
    robot_hardware_events
    robot_hardware
    GTest::gtest_main
)

gtest_discover_tests(RobotRuntimeTests)

# --- LiveRuntimeRunnerTests: deterministic finite-cycle scheduling over
# RobotRuntime::step(), including one genuine live integration test using
# the real HardwareEventSource.
add_executable(LiveRuntimeRunnerTests
    tests/LiveRuntimeRunnerTests.cpp
)

target_link_libraries(LiveRuntimeRunnerTests PRIVATE
    robot_runtime_runner
    robot_hardware_events
    robot_hardware
    GTest::gtest_main
)

gtest_discover_tests(LiveRuntimeRunnerTests)

# --- RealRobotHardwareTests: RealRobotHardware's actuator command mapping
# and strict wire-protocol response parsing (Phase 13K), plus focused
# RobotController/HardwareEventSource integration tests proving both
# consume RealRobotHardware through IRobotHardware exactly like
# SimulatedRobotHardware, unmodified. Uses a test-only
# RecordingRobotTransport defined inside the test file, not a production
# fake.
add_executable(RealRobotHardwareTests
    tests/RealRobotHardwareTests.cpp
)

target_link_libraries(RealRobotHardwareTests PRIVATE
    robot_real_hardware
    robot_controller
    robot_hardware_events
    GTest::gtest_main
)

gtest_discover_tests(RealRobotHardwareTests)

# --- SerialRobotTransportTests: '\n' framing, readLine passthrough
# (including no-trim behavior), and error-propagation behavior (Phase
# 13L), plus focused RealRobotHardware/HardwareEventSource/RobotController
# integration tests proving the full RecordingSerialPort ->
# SerialRobotTransport -> RealRobotHardware chain works exactly like the
# existing RecordingRobotTransport-based RealRobotHardwareTests, with
# HardwareEventSource/RobotController unmodified. Uses a test-only
# RecordingSerialPort defined inside the test file, not a production fake.
add_executable(SerialRobotTransportTests
    tests/SerialRobotTransportTests.cpp
)

target_link_libraries(SerialRobotTransportTests PRIVATE
    robot_serial_transport
    robot_real_hardware
    robot_hardware_events
    robot_controller
    GTest::gtest_main
)

gtest_discover_tests(SerialRobotTransportTests)

# --- SensorScriptTests: sensor script text-format parsing/validation
# (Phase 13I) - format, ordering, and error-message behavior in isolation
# from any runtime execution.
add_executable(SensorScriptTests
    tests/SensorScriptTests.cpp
)

target_link_libraries(SensorScriptTests PRIVATE
    robot_sensor_script
    GTest::gtest_main
)

target_compile_definitions(SensorScriptTests PRIVATE
    TEST_FIXTURES_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures/"
)

gtest_discover_tests(SensorScriptTests)

# --- CommandScriptTests: command script text-format parsing/validation
# (Phase 13J) - format, ordering, and error-message behavior in isolation
# from any runtime execution.
add_executable(CommandScriptTests
    tests/CommandScriptTests.cpp
)

target_link_libraries(CommandScriptTests PRIVATE
    robot_command_script
    GTest::gtest_main
)

target_compile_definitions(CommandScriptTests PRIVATE
    TEST_FIXTURES_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures/"
)

gtest_discover_tests(CommandScriptTests)

# --- ScriptedCommandEventSourceTests: cycle-eligibility/pending/consumption
# semantics for turning a CommandScript into an IPollingEventSource
# (Phase 13J).
add_executable(ScriptedCommandEventSourceTests
    tests/ScriptedCommandEventSourceTests.cpp
)

target_link_libraries(ScriptedCommandEventSourceTests PRIVATE
    robot_command_events
    GTest::gtest_main
)

target_compile_definitions(ScriptedCommandEventSourceTests PRIVATE
    TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output/"
)

gtest_discover_tests(ScriptedCommandEventSourceTests)

# --- CompositePollingEventSourceTests: command-before-sensor priority and
# at-most-one-event-per-poll behavior (Phase 13J), using small test-only
# IPollingEventSource fakes.
add_executable(CompositePollingEventSourceTests
    tests/CompositePollingEventSourceTests.cpp
)

target_link_libraries(CompositePollingEventSourceTests PRIVATE
    robot_polling_composite
    GTest::gtest_main
)

gtest_discover_tests(CompositePollingEventSourceTests)

# --- ScriptedLiveRuntimeRunnerTests: cycle-based sensor-script injection
# (Phase 13I), plus (Phase 13J) command-script cycle-notification and
# full live-mission integration tests combining CommandScript/SensorScript/
# ScriptedCommandEventSource/HardwareEventSource/CompositePollingEventSource
# with the real RobotRuntime/RobotStateMachine/RobotController/
# SimulatedRobotHardware pipeline.
add_executable(ScriptedLiveRuntimeRunnerTests
    tests/ScriptedLiveRuntimeRunnerTests.cpp
)

target_link_libraries(ScriptedLiveRuntimeRunnerTests PRIVATE
    robot_scripted_runtime
    robot_hardware_events
    robot_polling_composite
    GTest::gtest_main
)

target_compile_definitions(ScriptedLiveRuntimeRunnerTests PRIVATE
    TEST_OUTPUT_DIR="${CMAKE_BINARY_DIR}/test_output/"
)

gtest_discover_tests(ScriptedLiveRuntimeRunnerTests)

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

# --- VirtualWorldTests: deterministic initial demo-scene data (Phase
# 13M) - robot pose, base platform, and obstacle list. Links only
# robot_visual_world, never robot_visual/raylib, so this target never
# creates a window - genuinely headless, per the project's testing
# convention of never requiring a graphical display.
add_executable(VirtualWorldTests
    tests/visual/VirtualWorldTests.cpp
)

target_link_libraries(VirtualWorldTests PRIVATE
    robot_visual_world
    GTest::gtest_main
)

gtest_discover_tests(VirtualWorldTests)

# --- VirtualRobotHardwareTests: FSM-driven virtual actuator/movement
# behavior (Phase 13N) - command tracking, deterministic forward movement,
# the heading/front-marker direction convention, world-bounds clamping,
# and one focused RobotStateMachine/RobotController/VirtualRobotHardware/
# VirtualWorld integration test. As of Phase 13O, also covers
# geometry-backed obstacleDetected()/obstacleDistance() and full
# closed-loop ObstacleDetected/ObstacleCleared tests through the real
# HardwareEventSource/CompositePollingEventSource - hence the added
# robot_hardware_events/robot_polling_composite links below. Links
# robot_visual_simulation (which transitively provides
# robot_controller/robot_core/robot_runtime) plus robot_controller/
# robot_core explicitly, since the test file includes
# RobotController.hpp/RobotStateMachine.hpp directly. Never links
# robot_visual/raylib - genuinely headless, no window is ever created by
# this target.
add_executable(VirtualRobotHardwareTests
    tests/visual/VirtualRobotHardwareTests.cpp
)

target_link_libraries(VirtualRobotHardwareTests PRIVATE
    robot_visual_simulation
    robot_controller
    robot_core
    robot_hardware_events
    robot_polling_composite
    GTest::gtest_main
)

gtest_discover_tests(VirtualRobotHardwareTests)

# --- VirtualDistanceSensorTests: deterministic ray/AABB forward-sensor
# geometry (Phase 13O) - heading convention, sensor origin at the robot's
# front, range/threshold behavior, disabled-obstacle handling, and
# parallel/tangent/boundary ray edge cases. Uses VirtualWorld's
# setObstaclePosition()/setObstacleEnabled() test-support mutators to build
# controlled obstacle placements, never a second/disconnected obstacle
# representation. Links robot_visual_simulation only - never
# robot_visual/raylib - genuinely headless, no window is ever created by
# this target.
add_executable(VirtualDistanceSensorTests
    tests/visual/VirtualDistanceSensorTests.cpp
)

target_link_libraries(VirtualDistanceSensorTests PRIVATE
    robot_visual_simulation
    GTest::gtest_main
)

gtest_discover_tests(VirtualDistanceSensorTests)

# --- DifferentialDriveTests: differential-drive kinematics (Phase 13P) -
# wheel-speed commands, straight/backward/in-place/arc motion, the heading
# convention, delta-time scaling, and heading wraparound. Links only
# robot_visual_simulation - never robot_visual/raylib - genuinely headless,
# no window is ever created by this target.
add_executable(DifferentialDriveTests
    tests/visual/DifferentialDriveTests.cpp
)

target_link_libraries(DifferentialDriveTests PRIVATE
    robot_visual_simulation
    GTest::gtest_main
)

gtest_discover_tests(DifferentialDriveTests)

# --- RobotCollisionTests: circle-vs-AABB obstacle collision query (Phase
# 13P) - enabled/disabled obstacles, side/far obstacles, and toggling.
# Links only robot_visual_simulation - never robot_visual/raylib -
# genuinely headless, no window is ever created by this target.
add_executable(RobotCollisionTests
    tests/visual/RobotCollisionTests.cpp
)

target_link_libraries(RobotCollisionTests PRIVATE
    robot_visual_simulation
    GTest::gtest_main
)

gtest_discover_tests(RobotCollisionTests)

# --- ManualDriveInputTests: pure X/UP/DOWN/LEFT/RIGHT wheel-speed decision
# logic behind RobotSimulator3D's manual drive mode (Phase 13P) - X's
# highest priority, additive directional keys, opposite-key cancellation.
# No raylib keyboard state is faked; this tests the plain-boolean decision
# function main3d.cpp calls with real IsKeyDown() results. Links only
# robot_visual_simulation - never robot_visual/raylib.
add_executable(ManualDriveInputTests
    tests/visual/ManualDriveInputTests.cpp
)

target_link_libraries(ManualDriveInputTests PRIVATE
    robot_visual_simulation
    GTest::gtest_main
)

gtest_discover_tests(ManualDriveInputTests)

# --- ReactiveObstacleAvoidanceTests: deterministic turn wheel speeds
# behind RobotSimulator3D's reactive obstacle-avoidance policy (Phase
# 13Q) - opposite-sign wheels, zero linear velocity, the real
# DifferentialDrive turning convention, and turn-speed magnitude.  Links
# only robot_visual_simulation - never robot_visual/raylib - genuinely
# headless, no window is ever created by this target.
add_executable(ReactiveObstacleAvoidanceTests
    tests/visual/ReactiveObstacleAvoidanceTests.cpp
)

target_link_libraries(ReactiveObstacleAvoidanceTests PRIVATE
    robot_visual_simulation
    GTest::gtest_main
)

gtest_discover_tests(ReactiveObstacleAvoidanceTests)

# --- ForwardClearanceProbeTests: swept-body forward corridor clearance
# geometry (Phase 13R) - the expanded-AABB-vs-segment test behind
# avoidance's release condition, distinct from VirtualDistanceSensor's
# single-point ray. Links only robot_visual_simulation - never
# robot_visual/raylib - genuinely headless, no window is ever created by
# this target.
add_executable(ForwardClearanceProbeTests
    tests/visual/ForwardClearanceProbeTests.cpp
)

target_link_libraries(ForwardClearanceProbeTests PRIVATE
    robot_visual_simulation
    GTest::gtest_main
)

gtest_discover_tests(ForwardClearanceProbeTests)

# --- Warnings: scoped to our own targets only, never to FetchContent
# dependencies (GoogleTest, nlohmann/json), so third-party warning noise
# is never our concern to fix or silence.
if (MSVC)
    foreach(robot_target
        robot_core robot_scenario robot_logging robot_reporting robot_app
        robot_hardware robot_controller robot_hardware_events robot_runtime
        robot_runtime_runner robot_sensor_script robot_scripted_runtime
        robot_command_script robot_command_events robot_polling_composite
        robot_real_hardware robot_serial_transport
        robot_visual_world robot_visual robot_visual_simulation
        RobotSimulator RobotSimulator3D RobotSimulatorTests ScenarioSourceTests
        RobotHardwareTests RobotControllerTests HardwareEventSourceTests
        RobotRuntimeTests LiveRuntimeRunnerTests ScenarioIntegrationTests
        ApplicationTests SensorScriptTests ScriptedLiveRuntimeRunnerTests
        CommandScriptTests ScriptedCommandEventSourceTests
        CompositePollingEventSourceTests RealRobotHardwareTests
        SerialRobotTransportTests VirtualWorldTests VirtualRobotHardwareTests
        VirtualDistanceSensorTests DifferentialDriveTests
        RobotCollisionTests ManualDriveInputTests ReactiveObstacleAvoidanceTests
        ForwardClearanceProbeTests
    )
        target_compile_options(${robot_target} PRIVATE /W4)
    endforeach()
endif()
