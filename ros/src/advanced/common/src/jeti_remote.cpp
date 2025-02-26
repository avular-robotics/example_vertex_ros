// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include "jeti_remote.hpp"

void JetiRemote::RegisterActivationButtonCallback(std::function<void()> callback)
{
    activation_button_pressed_ = callback;
    RCLCPP_INFO(logger_, "JetiRemote: Use the SF switch to activate the execution of the example");
}

std::function<void(const sensor_msgs::msg::Joy &)> JetiRemote::GetControllerStateCallback()
{
    return std::bind(&JetiRemote::ControllerStateReceivedCallback, this, std::placeholders::_1);
}

void JetiRemote::ControllerStateReceivedCallback(const sensor_msgs::msg::Joy &controller_state)
{
    // Initial value is -100 to ensure the initial state is handled correctly
    static int32_t last_switch_state      = -100;
    static bool    switch_started_up_lock = false;

    // Detect the initial state of the switch
    if(last_switch_state == -100)
    {
        last_switch_state = controller_state.buttons[kSwitchSF];
        if(controller_state.buttons[kSwitchSF] == 1)
        {
            RCLCPP_INFO(logger_,
                        "JetiRemote: Initial state of the SF switch is up which is not allowed. "
                        "Put the switch in the down position before continuing...");
            switch_started_up_lock = true;
            return;
        }
    }

    // Detect the switch state change
    if(controller_state.buttons[kSwitchSF] != last_switch_state)
    {
        last_switch_state = controller_state.buttons[kSwitchSF];

        if(switch_started_up_lock)
        {
            switch_started_up_lock = false;
            RCLCPP_INFO(logger_, "JetiRemote: Switch is now in the down position. Switch up to "
                                 "activate the example");
            return;
        }

        activation_button_pressed_();
    }
}
