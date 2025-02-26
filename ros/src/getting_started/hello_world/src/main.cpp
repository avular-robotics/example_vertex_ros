// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Simple hello world example for the CreOS client library.
 *
 * This example demonstrates how to connect to the CreOS server and subscribe to
 * battery status messages.
 ****************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <creos_sdk_msgs/msg/battery_status.hpp>
#include <iostream>

class HelloWorldNode : public rclcpp::Node
{
public:
    HelloWorldNode() : Node("hello_world_node")
    {
        // Create a subscription to the battery status topic
        battery_status_sub_ = this->create_subscription<creos_sdk_msgs::msg::BatteryStatus>(
            "/robot/battery", rclcpp::SensorDataQoS(),
            [this](const creos_sdk_msgs::msg::BatteryStatus::SharedPtr msg)
            {
                RCLCPP_INFO(this->get_logger(), "Battery state of charge: %.2f%%",
                            msg->state_of_charge * 100);
            });
    }

private:
    rclcpp::Subscription<creos_sdk_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
};

int main()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<HelloWorldNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
