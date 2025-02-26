// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include <creos_sdk_msgs/msg/control_source.hpp>

/**
 * @brief Notifies the user of control source changes
 * @param control_source The control source message
 * @param node The node
 * @details This function logs the control source to the console whenever it changes
 */
void control_source_notifier(const creos_sdk_msgs::msg::ControlSource::SharedPtr control_source,
                             rclcpp::Node::SharedPtr                             node)
{
    static int8_t previous_control_source = -1;

    if(control_source->source != previous_control_source)
    {
        std::string control_source_str = "";
        switch(control_source->source)
        {
        case creos_sdk_msgs::msg::ControlSource::DISABLED:
            control_source_str = "Disabled";
            break;
        case creos_sdk_msgs::msg::ControlSource::MANUAL:
            control_source_str = "Manual";
            break;
        case creos_sdk_msgs::msg::ControlSource::AUTONOMOUS:
            control_source_str = "Autonomous";
            break;
        case creos_sdk_msgs::msg::ControlSource::USER:
            control_source_str = "User";
            break;
        default:
            throw std::runtime_error("Unknown control source");
        }

        RCLCPP_INFO(node->get_logger(), "Control source: %s", control_source_str.c_str());
    }
    previous_control_source = control_source->source;
}
