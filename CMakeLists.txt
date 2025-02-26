# Copyright (C) 2024 Avular B.V. - All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular 
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

cmake_minimum_required(VERSION 3.20)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

project(vertex_creos_sdk_examples)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Place all binaries in the same directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

option(BUILD_EXAMPLES "Build the examples" ON)
option(BUILD_TESTS "Build the tests" OFF)
option(CHECK_COVERAGE "Check the code coverage." OFF)
option(ENABLE_SANITIZERS "Enable sanitizers" OFF)
option(WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

# Enable extra warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
if(WARNINGS_AS_ERRORS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
endif()

if(BUILD_TESTS)
    include(CTest)
    enable_testing()
endif()

if(CHECK_COVERAGE)
    include(CodeCoverage)
    append_coverage_compiler_flags()
endif()

include(GetPackageDependencies)

add_subdirectory(sdk)
