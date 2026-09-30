// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Example of how to print remote controller input from ROS 2
 ****************************************************************************/
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>

#include "remote_controller_example.hpp"

class RemoteControllerNode : public rclcpp::Node
{
public:
    RemoteControllerNode() : Node("remote_controller_node")
    {
        // Setup Controller input
        controller_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/robot/joy", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Joy::SharedPtr state)
            { handleRemoteControllerInput(*state, rc_utils_, this->get_logger()); });
    }

private:
    RcUtils                                                rc_utils_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr controller_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteControllerNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
