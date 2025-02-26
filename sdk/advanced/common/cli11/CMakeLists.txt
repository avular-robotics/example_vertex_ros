# Copyright (C) 2024 Avular B.V. - All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular 
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

add_library(CLI11::CLI11 INTERFACE IMPORTED GLOBAL)
set_target_properties(CLI11::CLI11 PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}"
)
