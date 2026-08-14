cmake_minimum_required(VERSION 3.20)

project(RobotSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# --- robot_core: production FSM library, shared by the app and the tests ---
add_library(robot_core STATIC
    src/RobotStateMachine.cpp
)

target_include_directories(robot_core PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# --- RobotSimulator: the command-line application ---
add_executable(RobotSimulator
    src/main.cpp
)

target_link_libraries(RobotSimulator PRIVATE robot_core)

# --- Tests (GoogleTest via FetchContent, pinned to a stable tagged release) ---
enable_testing()

include(FetchContent)

# MSVC default runtime is /MD; force gtest to match so link flags agree.
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.15.2
)

FetchContent_MakeAvailable(googletest)

add_executable(RobotSimulatorTests
    tests/RobotStateMachineTests.cpp
)

target_link_libraries(RobotSimulatorTests PRIVATE
    robot_core
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(RobotSimulatorTests)
