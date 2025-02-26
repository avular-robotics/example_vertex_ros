# Copyright (C) 2024 Avular B.V. - All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular 
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

cmake_minimum_required(VERSION 3.20)

project(system_monitor_example VERSION 1.0.0 LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(creos_sdk_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp creos_sdk_msgs)

install(TARGETS
${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

ament_package()
