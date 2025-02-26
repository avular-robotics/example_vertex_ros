# Copyright (C) 2024 Avular B.V. - All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular 
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

file(GLOB SOURCES "src/*.cpp")

find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)

add_library(advanced_common STATIC ${SOURCES})

target_include_directories(advanced_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(advanced_common PUBLIC
    Eigen3::Eigen
    spdlog::spdlog
)

if(BUILD_TESTS)
    add_subdirectory(tests)
endif()

add_subdirectory(cli11)
