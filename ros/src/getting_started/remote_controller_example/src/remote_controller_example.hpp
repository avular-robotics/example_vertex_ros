// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include "rc_utils.hpp"

inline void handleRemoteControllerInput(const sensor_msgs::msg::Joy &state,
                                        RcUtils                     &rc_utils,
                                        rclcpp::Logger               logger)
{
    RCLCPP_INFO(logger, "%s", rc_utils.CreateControllerInputDisplay(state).c_str());
}
