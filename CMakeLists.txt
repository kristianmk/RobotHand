# Written by K. M. Knausgård 2023-10. All rights reserved.

cmake_minimum_required(VERSION 3.5.1)
project(RobotHandTest VERSION 0.1.0 LANGUAGES CXX)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include FetchContent module
include(FetchContent)

# Define the content to fetch (qbdevice-api-7.x.x)
# Following https://cmake.org/cmake/help/latest/module/FetchContent.html
FetchContent_Declare(
  qbrobotics-device
  GIT_REPOSITORY https://bitbucket.org/qbrobotics/qbdevice-api-7.x.x.git
  GIT_TAG 1a744b318d73f32553e578ff9b7ca240b2d45c07  # specify the commit hash or tag
)

# Fetch the content
FetchContent_MakeAvailable(qbrobotics-device)


# List of source files
set(SOURCES
    main.cpp
    qroboticsrobothand.cpp
    serialportqroboticsrobothanddetector.cpp
)

# List of header files
set(HEADERS
    robothand.h
    qroboticsrobothand.h
    qroboticsrobothanddetector.h
    serialportqroboticsrobothanddetector.h
)

# Add executable
add_executable(RobotHandTest ${SOURCES})

# Specify the headers for the target
target_sources(RobotHandTest PRIVATE ${HEADERS})

# Make sure handTest depends on qbrobotics_driver and Serial
# Serial is not built automatically without this..
add_dependencies(RobotHandTest qbrobotics_driver Serial)

# Link with the driver library
target_link_libraries(RobotHandTest PRIVATE qbrobotics_driver::qbrobotics_driver)


