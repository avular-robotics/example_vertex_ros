// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include <creos_sdk_msgs/msg/state.hpp>

/**
 * @brief Notifies the user of robot state changes
 * @param state The state message
 * @param node The node
 * @details This function logs the robot state to the console whenever it changes
 */
void state_notifier(const creos_sdk_msgs::msg::State::SharedPtr state, rclcpp::Node::SharedPtr node)
{
    static std::shared_ptr<creos_sdk_msgs::msg::State> previous_state =
        std::make_shared<creos_sdk_msgs::msg::State>();

    if(state->current_action != previous_state->current_action ||
       state->ready_state != previous_state->ready_state)
    {
        std::string ready_state = "";
        switch(state->ready_state)
        {
        case creos_sdk_msgs::msg::State::UNKNOWN:
            ready_state = "Unknown";
            break;
        case creos_sdk_msgs::msg::State::NOT_READY:
            ready_state = "NotReady";
            break;
        case creos_sdk_msgs::msg::State::PRE_ARM:
            ready_state = "PreArm";
            break;
        case creos_sdk_msgs::msg::State::ACTIVE:
            ready_state = "Active";
            break;
        case creos_sdk_msgs::msg::State::WAITING:
            ready_state = "Waiting";
            break;
        case creos_sdk_msgs::msg::State::FAILSAFE:
            ready_state = "Failsafe";
            break;
        case creos_sdk_msgs::msg::State::ERROR:
            ready_state = "Error";
            break;
        default:
            throw std::runtime_error("Unknown ready state");
        }

        RCLCPP_INFO(node->get_logger(), "State: %s - %s", ready_state.c_str(),
                    state->current_action.c_str());
    }
    previous_state = state;
}
