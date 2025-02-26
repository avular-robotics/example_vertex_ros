# Copyright 2024 Avular B.V.
# All rights reserved.
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

function(GetPackageDebianDependencies)
    set(options)
    set(oneValueArgs PACKAGE_XML PREFIX)
    set(multiValueArgs)
    cmake_parse_arguments(ARG "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN})
    
    # Check if the required arguments are provided
    if (NOT ARG_PREFIX)
        message(FATAL_ERROR "No PREFIX provided")
    endif()

    # Check if package.xml exists
    if (NOT EXISTS "${ARG_PACKAGE_XML}")
        set(ARG_PACKAGE_XML "${CMAKE_CURRENT_SOURCE_DIR}/package.xml")
    endif()

    # Get xmllint executable
    if (NOT EXISTS "${ARG_PACKAGE_XML}")
        message(FATAL_ERROR "No package.xml file found in the root of the project")
    endif()

    find_program(XMLLINT_EXECUTABLE xmllint REQUIRED)

    # Get the dependencies from the package.xml file
    execute_process(
      COMMAND ${XMLLINT_EXECUTABLE} --xpath "package/export/debian_depend/text()" "${ARG_PACKAGE_XML}"
      OUTPUT_VARIABLE PACKAGE_DEPENDENCIES
      ERROR_QUIET
    )

    if (PACKAGE_DEPENDENCIES)
      # strip leading and trailing whitespace
      string(STRIP ${PACKAGE_DEPENDENCIES} PACKAGE_DEPENDENCIES)
      # dependencies are separated by newlines, but Debian requires commas
      string(REPLACE "\n" ", " PACKAGE_DEPENDENCIES ${PACKAGE_DEPENDENCIES})
    else()
      # if no dependencies are found, set an empty string
      set(PACKAGE_DEPENDENCIES "")
    endif()

    set("${ARG_PREFIX}_DEPS" ${PACKAGE_DEPENDENCIES} PARENT_SCOPE)
endfunction()
