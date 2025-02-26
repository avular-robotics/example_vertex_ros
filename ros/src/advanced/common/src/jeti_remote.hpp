// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#pragma once

#include <functional>
#include <sensor_msgs/msg/joy.hpp>
#include <common/remote_controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

class JetiRemote : public IRemoteController
{
public:
    enum ButtonIndex
    {
        kSwitchSB = 0,
        kSwitchSA = 1,
        kSwitchSF = 2,
        kSwitchSE = 3,
    };

    enum AxisIndex
    {
        kXAxis = 0,
        kYAxis = 1,
        kZAxis = 2
    };

    JetiRemote(rclcpp::Logger logger) : logger_(logger) {}

    void RegisterActivationButtonCallback(std::function<void()> callback) override;
    std::function<void(const sensor_msgs::msg::Joy &)> GetControllerStateCallback() override;

    void ControllerStateReceivedCallback(const sensor_msgs::msg::Joy &controller_state) override;

private:
    std::function<void()> activation_button_pressed_;
    rclcpp::Logger        logger_;
};
