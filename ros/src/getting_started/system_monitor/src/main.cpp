// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * This example demonstrates how to retrieve system information and monitor
 * the state of the robot.
 *
 * Upon running this example, the system information and battery status will
 * be logged once. The state of the robot will be printed initially and whenever it changes.
 ****************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <creos_sdk_msgs/msg/system_info.hpp>
#include <creos_sdk_msgs/msg/battery_status.hpp>
#include <creos_sdk_msgs/msg/state.hpp>
#include <creos_sdk_msgs/msg/control_source.hpp>

#include <iostream>
#include <mutex>

#include "system_info.hpp"
#include "battery_notifier.hpp"
#include "state_notifier.hpp"
#include "control_source_notifier.hpp"

class SystemMonitorNode : public rclcpp::Node
{
public:
    SystemMonitorNode() : Node("system_monitor_node")
    {
        RCLCPP_INFO(this->get_logger(), "System monitor example:");

        system_info_sub_ = this->create_subscription<creos_sdk_msgs::msg::SystemInfo>(
            "/robot/system_info", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            [this](const creos_sdk_msgs::msg::SystemInfo::SharedPtr msg)
            { handle_system_info(msg, this->shared_from_this()); });
        state_sub_ = this->create_subscription<creos_sdk_msgs::msg::State>(
            "/robot/state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
            [this](const creos_sdk_msgs::msg::State::SharedPtr msg)
            { state_notifier(msg, this->shared_from_this()); });
        battery_status_sub_ = this->create_subscription<creos_sdk_msgs::msg::BatteryStatus>(
            "/robot/battery", rclcpp::SensorDataQoS(),
            [this](const creos_sdk_msgs::msg::BatteryStatus::SharedPtr msg)
            { battery_notifier(msg, this->shared_from_this()); });
        control_source_sub_ = this->create_subscription<creos_sdk_msgs::msg::ControlSource>(
            "/robot/current_control_source", rclcpp::SensorDataQoS(),
            [this](const creos_sdk_msgs::msg::ControlSource::SharedPtr msg)
            { control_source_notifier(msg, this->shared_from_this()); });
    }

private:
    rclcpp::Subscription<creos_sdk_msgs::msg::SystemInfo>::SharedPtr    system_info_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::State>::SharedPtr         state_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::BatteryStatus>::SharedPtr battery_status_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::ControlSource>::SharedPtr control_source_sub_;
};

int main()
{
    rclcpp::init(0, nullptr);
    auto node = std::make_shared<SystemMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
