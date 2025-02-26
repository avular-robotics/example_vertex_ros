# Copyright (C) 2024 Avular B.V. - All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular 
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

set(EXAMPLE_NAME system_monitor_example)

add_executable(${EXAMPLE_NAME} src/main.cpp)

find_package(creos_client REQUIRED)
find_package(creos_msgs REQUIRED)

target_link_libraries(${EXAMPLE_NAME} PRIVATE creos::client spdlog::spdlog)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()
