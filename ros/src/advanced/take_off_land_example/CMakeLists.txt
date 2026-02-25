cmake_minimum_required(VERSION 3.20)

project(take_off_land_example VERSION 1.0.0 LANGUAGES CXX)

find_package(rclcpp REQUIRED)
find_package(creos_sdk_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(advanced_common REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME} rclcpp creos_sdk_msgs sensor_msgs advanced_common)

install(TARGETS
${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

ament_package()
