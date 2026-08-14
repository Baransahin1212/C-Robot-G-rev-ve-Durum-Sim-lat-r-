cmake_minimum_required(VERSION 3.20)

project(RobotSimulator LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(RobotSimulator
    src/main.cpp
)

target_include_directories(RobotSimulator PRIVATE
    ${CMAKE_SOURCE_DIR}/include
)
