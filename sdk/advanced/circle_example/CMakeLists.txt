# Copyright (C) 2024 Avular B.V. - All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular 
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

set(EXAMPLE_NAME circle_example)

add_executable(${EXAMPLE_NAME} src/main.cpp)

find_package(creos_client REQUIRED)
find_package(creos_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

add_library(${EXAMPLE_NAME}_lib STATIC)
target_link_libraries(${EXAMPLE_NAME}_lib PUBLIC
    advanced_common
    spdlog::spdlog)

target_link_libraries(${EXAMPLE_NAME} PRIVATE creos::client ${EXAMPLE_NAME}_lib spdlog::spdlog CLI11::CLI11)

target_sources(
    ${EXAMPLE_NAME}_lib
    PRIVATE
        src/circle_references.cpp
)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
