// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include <creos/messages/system_info.hpp>
#include <creos/system_info_interface.hpp>
#include <spdlog/spdlog.h>

/**
 * @brief Gets the system info and prints it to the console
 * @param system_info_interface The system info interface
 * @details This function gets the system info and prints it to the console.
 */
void get_system_info(creos::ISystemInfoInterface *system_info_interface)
{
    auto system_info = system_info_interface->getSystemInfo();
    spdlog::info("System info:");
    spdlog::info("  - Name: {}", system_info.name);
    spdlog::info("  - Hostname: {}", system_info.hostname);
    spdlog::info("  - Serial: {}", system_info.serial);
    spdlog::info("  - Platform: {}", system_info.platform);
    spdlog::info("  - Components:");
    for(const auto &component : system_info.component_versions)
    {
        std::string type = "";
        switch(component.type)
        {
        case creos_messages::ComponentVersion::Type::kUnknown:
            type = "Unknown";
            break;
        case creos_messages::ComponentVersion::Type::kOs:
            type = "Operating system";
            break;
        case creos_messages::ComponentVersion::Type::kContainer:
            type = "Container";
            break;
        default:
            throw std::runtime_error("Unknown component type");
        }
        spdlog::info("    - {} ({}): {}", component.name, type, component.version);
    }
}
