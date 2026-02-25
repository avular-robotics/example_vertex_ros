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
#include <creos_sdk_msgs/msg/state_reference.hpp>

#include <common/drone_state_interface.hpp>
#include "remote_controller_references.hpp"

constexpr uint kRollIndex     = 0;
constexpr uint kPitchIndex    = 1;
constexpr uint kThrottleIndex = 2;
constexpr uint kYawIndex      = 3;

void handleRemoteControllerInput(
    const sensor_msgs::msg::Joy                                      &state,
    RemoteControllerReferences                                       &remote_controller_references,
    std::shared_ptr<IDroneState>                                      drone_state,
    rclcpp::Publisher<creos_sdk_msgs::msg::StateReference>::SharedPtr state_reference_pub,
    rclcpp::Logger                                                    logger)
{
    static creos_sdk_msgs::msg::ControlSource previous_control_source;

    auto current_control_source = drone_state->GetControlSource();

    // Notify the user when the control source changes
    if(current_control_source.source != previous_control_source.source)
    {
        if(current_control_source.source == creos_sdk_msgs::msg::ControlSource::USER)
        {
            RCLCPP_INFO(logger,
                        "User control enabled: Controlling the drone using StateReference from "
                        "the remote controller input");
        }
        else
        {
            RCLCPP_INFO(logger, "User control disabled");
        }
        previous_control_source = current_control_source;
    }

    // Only send StateReference when the control source is set to User and the drone is flying
    if(current_control_source.source == creos_sdk_msgs::msg::ControlSource::USER &&
       drone_state->IsInFlight())
    {
        const std::array<float, 2> left_stick = {state.axes[kYawIndex], state.axes[kThrottleIndex]};
        const std::array<float, 2> right_stick = {state.axes[kRollIndex], state.axes[kPitchIndex]};

        // Use the timestamp from the Joy message
        creos_sdk_msgs::msg::StateReference reference =
            remote_controller_references.CreateReference(
                left_stick, right_stick, drone_state->GetOrientation(), state.header.stamp);
        state_reference_pub->publish(reference);
    }
}
