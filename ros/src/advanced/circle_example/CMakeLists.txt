# Copyright (C) 2024 Avular B.V. - All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular 
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

cmake_minimum_required(VERSION 3.20)

project(circle_example VERSION 1.0.0 LANGUAGES CXX)

option(BUILD_TESTS "Build the tests" OFF)

find_package(rclcpp REQUIRED)
find_package(creos_sdk_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(advanced_common REQUIRED)
find_package(tf2_ros REQUIRED)

# Library
add_library(${PROJECT_NAME}_lib STATIC src/circle_references.cpp)
ament_target_dependencies(${PROJECT_NAME}_lib rclcpp creos_sdk_msgs sensor_msgs advanced_common tf2_ros)

install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Executable
add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib)
ament_target_dependencies(${PROJECT_NAME} rclcpp advanced_common)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

install(TARGETS
${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

ament_package()
