# Copyright (C) 2024 Avular B.V. - All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular 
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

cmake_minimum_required(VERSION 3.20)

project(advanced_common VERSION 1.0.0 LANGUAGES CXX)

option(BUILD_TESTS "Build the tests" OFF)

find_package(rclcpp REQUIRED)
find_package(creos_sdk_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

file(GLOB SOURCES "src/*.cpp")

add_library(${PROJECT_NAME} STATIC ${SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

ament_target_dependencies(${PROJECT_NAME} rclcpp creos_sdk_msgs sensor_msgs nav_msgs geometry_msgs tf2_ros)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
