// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include <creos_sdk_msgs/msg/system_info.hpp>
#include <creos_sdk_msgs/msg/component_version.hpp>

/**
 * @brief Handles the system info message
 * @param system_info The system info message
 * @details This function logs the system information to the console
 */
void handle_system_info(const creos_sdk_msgs::msg::SystemInfo::SharedPtr system_info,
                        rclcpp::Node::SharedPtr                          node)
{
    RCLCPP_INFO(node->get_logger(), "System info:");
    RCLCPP_INFO(node->get_logger(), "  - Name: %s", system_info->name.c_str());
    RCLCPP_INFO(node->get_logger(), "  - Hostname: %s", system_info->hostname.c_str());
    RCLCPP_INFO(node->get_logger(), "  - Serial: %s", system_info->serial.c_str());
    RCLCPP_INFO(node->get_logger(), "  - Platform: %s", system_info->platform.c_str());
    RCLCPP_INFO(node->get_logger(), "  - Components:");
    for(const auto &component : system_info->component_versions)
    {
        std::string type = "";
        switch(component.type)
        {
        case creos_sdk_msgs::msg::ComponentVersion::UNKNOWN_TYPE:
            type = "Unknown";
            break;
        case creos_sdk_msgs::msg::ComponentVersion::OS:
            type = "Operating system";
            break;
        case creos_sdk_msgs::msg::ComponentVersion::CONTAINER:
            type = "Container";
            break;
        default:
            throw std::runtime_error("Unknown component type");
        }
        RCLCPP_INFO(node->get_logger(), "    - %s (%s): %s", component.name.c_str(), type.c_str(),
                    component.version.c_str());
    }
}
