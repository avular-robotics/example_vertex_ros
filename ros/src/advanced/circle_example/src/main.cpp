// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Example of how to fly a circle with the drone.
 ****************************************************************************/
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <creos_sdk_msgs/msg/state.hpp>
#include <creos_sdk_msgs/msg/state_reference.hpp>
#include <creos_sdk_msgs/msg/control_source.hpp>
#include <creos_sdk_msgs/srv/send_command.hpp>

#include <common/logging.hpp>
#include <common/drone_state.hpp>
#include <common/remote_controller_interface.hpp>
#include <common/flight_controller.hpp>

#include "circle_references.hpp"

class CircleNode : public rclcpp::Node
{
public:
    CircleNode() : Node("circle_node")
    {
        auto circle_radius_param        = rcl_interfaces::msg::ParameterDescriptor{};
        circle_radius_param.description = "Radius of the circle in meters. Default is 2 meters.";
        this->declare_parameter("circle_radius", 2.0, circle_radius_param);

        auto speed_param        = rcl_interfaces::msg::ParameterDescriptor{};
        speed_param.description = "Speed of the drone in meters per second. Default is 0.5 m/s.";
        this->declare_parameter("speed", 0.5, speed_param);

        auto frequency_param        = rcl_interfaces::msg::ParameterDescriptor{};
        frequency_param.description = "Update frequency in Hz. Default is 100 Hz.";
        this->declare_parameter("frequency", 100, frequency_param);

        auto delay_param        = rcl_interfaces::msg::ParameterDescriptor{};
        delay_param.description = "Delay in seconds before the drone takes off when all take off "
                                  "conditions are met. Default is 2 seconds.";
        this->declare_parameter("take_off_delay", 2, delay_param);

        auto controller_param = rcl_interfaces::msg::ParameterDescriptor{};
        controller_param.description =
            "Controller type to use: 'herelink' or 'jeti'. Default is 'herelink'.";
        this->declare_parameter("controller", "herelink");

        circle_radius_m_            = this->get_parameter("circle_radius").as_double();
        speed_mps_                  = this->get_parameter("speed").as_double();
        update_frequency_hz_        = this->get_parameter("frequency").as_int();
        take_off_delay_s_           = this->get_parameter("take_off_delay").as_int();
        std::string controller_type = this->get_parameter("controller").as_string();

        timer_ = create_wall_timer(std::chrono::milliseconds(1000 / update_frequency_hz_),
                                   std::bind(&CircleNode::Run, this));

        // Setup DroneState
        drone_state_  = std::make_shared<DroneState>();
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot/odometry", rclcpp::SensorDataQoS(), drone_state_->GetOdometryCallback());
        state_sub_ = this->create_subscription<creos_sdk_msgs::msg::State>(
            "robot/state", rclcpp::SensorDataQoS(), drone_state_->GetStateCallback());
        control_source_sub_ = this->create_subscription<creos_sdk_msgs::msg::ControlSource>(
            "/robot/current_control_source", rclcpp::SensorDataQoS(),
            drone_state_->GetControlSourceCallback());

        // Setup Controller input
        if(controller_type == "herelink")
        {
            controller_ = CreateRemoteController(ControllerType::kHerelink, this->get_logger());
        }
        else if(controller_type == "jeti")
        {
            controller_ = CreateRemoteController(ControllerType::kJeti, this->get_logger());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Unknown controller type: %s",
                         controller_type.c_str());
            throw std::runtime_error("Unknown controller type");
        }
        controller_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/robot/joy", rclcpp::SensorDataQoS(), controller_->GetControllerStateCallback());
        controller_->RegisterActivationButtonCallback(
            [this]()
            {
                execution_active_ = !execution_active_;
                RCLCPP_INFO(this->get_logger(), "Execution active: %s",
                            execution_active_ ? "true" : "false");
            });

        // Setup FlightController
        flight_controller_ = std::make_shared<FlightController>(*drone_state_, take_off_delay_s_,
                                                                this->get_logger());
        send_command_client_ =
            this->create_client<creos_sdk_msgs::srv::SendCommand>("/robot/send_command");
        flight_controller_->RegisterTakeOffTrigger(
            [this]()
            {
                try
                {
                    auto request    = std::make_shared<creos_sdk_msgs::srv::SendCommand::Request>();
                    request->action = creos_sdk_msgs::srv::SendCommand::Request::TAKE_OFF;
                    send_command_client_->async_send_request(request);
                }
                catch(const std::exception &e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to send takeoff command: %s",
                                 e.what());
                }
            });

        // Setup CircleReferences
        circle_references_ = std::make_shared<CircleReferences>(
            this->get_logger(), update_frequency_hz_, circle_radius_m_, speed_mps_);

        state_reference_pub_ = this->create_publisher<creos_sdk_msgs::msg::StateReference>(
            "/robot/cmd_state_ref", rclcpp::SensorDataQoS());
    }

    void Run()
    {
        if(execution_active_)
        {
            auto state = flight_controller_->Run();
            if(state == FlightState::kFlying)
            {
                // Publish circle references when the drone is flying
                creos_sdk_msgs::msg::StateReference state_reference =
                    circle_references_->GetNewStateReference(this->now());
                state_reference_pub_->publish(state_reference);
            }
            else
            {
                // Reset the circle references when the drone is not flying
                circle_references_->Reset(drone_state_->GetPosition(), drone_state_->GetYaw());
            }
        }
        // Reset the circle references when the exacution is not active since the drone could be
        // moving using the remote controller therefore the middle of circle should be updated
        else
        {
            circle_references_->Reset(drone_state_->GetPosition(), drone_state_->GetYaw());
        }
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    double   circle_radius_m_;
    double   speed_mps_;
    unsigned update_frequency_hz_;
    unsigned take_off_delay_s_;

    bool execution_active_ = false;

    std::shared_ptr<DroneState>        drone_state_;
    std::shared_ptr<IRemoteController> controller_;
    std::shared_ptr<FlightController>  flight_controller_;

    std::shared_ptr<CircleReferences> circle_references_;

    // ROS Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr              controller_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr            odometry_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::State>::SharedPtr         state_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::ControlSource>::SharedPtr control_source_sub_;

    // ROS Publishers
    rclcpp::Publisher<creos_sdk_msgs::msg::StateReference>::SharedPtr state_reference_pub_;

    // ROS Clients
    rclcpp::Client<creos_sdk_msgs::srv::SendCommand>::SharedPtr send_command_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CircleNode>();

    setup_logging("circle_example", node->get_logger());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
