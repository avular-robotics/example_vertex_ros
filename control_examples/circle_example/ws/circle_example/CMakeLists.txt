# Copyright (C) 2024 Avular B.V. - All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file. If not, please write to: legal@avular.com

cmake_minimum_required(VERSION 3.16)

# find_package(cmake_avular REQUIRED)
# parse_manifest("${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
project(drone_circle_example
    VERSION 0.1.0
    DESCRIPTION "Drone example where it flies in a circle from it's starting position."
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS NO)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -Werror")

include(FetchContent)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(avular_mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


add_library(circle-example-lib
    src/main.cpp
    src/circle_example_node.cpp
)


target_include_directories(circle-example-lib
PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
)

add_executable(circle_example
    src/main.cpp
)

ament_target_dependencies(circle-example-lib rclcpp std_msgs avular_mavros_msgs nav_msgs sensor_msgs tf2_ros)

target_link_libraries(circle_example
    circle-example-lib
)

install(TARGETS
    circle_example
    DESTINATION lib/${PROJECT_NAME})

ament_package()

message("")
message("Target")
message("--------------------------------")
message("Architecture: ${CMAKE_SYSTEM_PROCESSOR}")
message("System:       ${CMAKE_SYSTEM_NAME}")
message("--------------------------------")

message("")

