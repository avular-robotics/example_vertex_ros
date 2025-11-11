// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Example of how to use the remote controller with the drone via the Jetson
 ****************************************************************************/
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <common/logging.hpp>
#include <common/drone_state.hpp>

#include "remote_controller_example.hpp"
#include "remote_controller_references.hpp"

class RemoteControllerNode : public rclcpp::Node
{
public:
    RemoteControllerNode() : Node("remote_controller_node")
    {
        auto max_velocity_horizontal_param = rcl_interfaces::msg::ParameterDescriptor{};
        max_velocity_horizontal_param.description =
            "Maximum velocity in the horizontal plane in m/s. Default is 5 m/s.";
        this->declare_parameter("max_velocity_horizontal", 5.0, max_velocity_horizontal_param);

        auto max_velocity_vertical_param = rcl_interfaces::msg::ParameterDescriptor{};
        max_velocity_vertical_param.description =
            "Maximum velocity in the vertical plane in m/s. Default is 1 m/s.";
        this->declare_parameter("max_velocity_vertical", 1.0, max_velocity_vertical_param);

        auto max_yaw_rate_param = rcl_interfaces::msg::ParameterDescriptor{};
        max_yaw_rate_param.description =
            "Maximum yaw rate in degrees per second. Default is 60 deg/s.";
        this->declare_parameter("max_yaw_rate", 60.0, max_yaw_rate_param);

        auto yaw_rate_mode_param        = rcl_interfaces::msg::ParameterDescriptor{};
        yaw_rate_mode_param.description = "Enable yaw rate mode. Default is false.";
        this->declare_parameter("yaw_rate_mode", false, yaw_rate_mode_param);

        max_velocity_horizontal_m_ = this->get_parameter("max_velocity_horizontal").as_double();
        max_velocity_vertical_m_   = this->get_parameter("max_velocity_vertical").as_double();
        max_yaw_rate_deg_          = this->get_parameter("max_yaw_rate").as_double();
        yaw_rate_mode_             = this->get_parameter("yaw_rate_mode").as_bool();

        // Setup DroneState
        drone_state_  = std::make_shared<DroneState>(this->get_logger());
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot/odometry", rclcpp::SensorDataQoS(), drone_state_->GetOdometryCallback());
        state_sub_ = this->create_subscription<creos_sdk_msgs::msg::State>(
            "robot/state", rclcpp::SensorDataQoS(), drone_state_->GetStateCallback());
        control_source_sub_ = this->create_subscription<creos_sdk_msgs::msg::ControlSource>(
            "/robot/current_control_source", rclcpp::SensorDataQoS(),
            drone_state_->GetControlSourceCallback());

        // Setup StateReference Publisher
        state_reference_pub_ = this->create_publisher<creos_sdk_msgs::msg::StateReference>(
            "/robot/cmd_state_ref", rclcpp::SensorDataQoS());

        // Setup Controller input
        controller_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/robot/joy", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::Joy::SharedPtr state)
            {
                handleRemoteControllerInput(*state, *remote_controller_references_, drone_state_,
                                            state_reference_pub_, this->get_logger());
            });

        // Setup Remote Controller References
        remote_controller_references_ = std::make_shared<RemoteControllerReferences>(
            max_velocity_horizontal_m_, max_velocity_vertical_m_, max_yaw_rate_deg_,
            yaw_rate_mode_);
    }

private:
    double max_velocity_horizontal_m_;
    double max_velocity_vertical_m_;
    double max_yaw_rate_deg_;
    bool   yaw_rate_mode_;

    std::shared_ptr<DroneState> drone_state_;

    std::shared_ptr<RemoteControllerReferences> remote_controller_references_;

    // ROS Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr              controller_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odometry_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::State>::SharedPtr         state_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::ControlSource>::SharedPtr control_source_sub_;

    // ROS Publishers
    rclcpp::Publisher<creos_sdk_msgs::msg::StateReference>::SharedPtr state_reference_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RemoteControllerNode>();

    setup_logging("remote_controller_example", node->get_logger());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
