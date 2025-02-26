// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include "herelink_remote.hpp"
#include <spdlog/spdlog.h>

void HereLinkRemote::RegisterActivationButtonCallback(std::function<void()> callback)
{
    activation_button_pressed_ = callback;
    spdlog::info("HereLinkRemote: Press the C button to toggle execution of the example");
}

std::function<void(const creos_messages::ControllerState &)>
    HereLinkRemote::GetControllerStateCallback()
{
    return std::bind(&HereLinkRemote::ControllerStateReceivedCallback, this, std::placeholders::_1);
}

void HereLinkRemote::ControllerStateReceivedCallback(
    const creos_messages::ControllerState &controller_state)
{
    // Toggle execution when the button is pressed
    if(controller_state.buttons[ButtonIndex::kButtonC] ==
       creos_messages::ControllerState::ButtonState::kPressed)
    {
        activation_button_pressed_();
    }
}
