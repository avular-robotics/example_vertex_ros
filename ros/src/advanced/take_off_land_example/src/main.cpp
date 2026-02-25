// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Example of how to use the CreOS SDK to automatically take off and land.
 ****************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <chrono>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <creos_sdk_msgs/msg/state.hpp>
#include <creos_sdk_msgs/msg/control_source.hpp>
#include <creos_sdk_msgs/srv/send_command.hpp>

#include <common/logging.hpp>
#include <common/drone_state.hpp>
#include <common/remote_controller_interface.hpp>
#include <common/flight_controller.hpp>

class TakeOffLandNode : public rclcpp::Node
{
public:
    TakeOffLandNode() : Node("take_off_land_node")
    {
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

        update_frequency_hz_        = this->get_parameter("frequency").as_int();
        take_off_delay_s            = this->get_parameter("take_off_delay").as_int();
        std::string controller_type = this->get_parameter("controller").as_string();

        timer_ = create_wall_timer(std::chrono::milliseconds(1000 / update_frequency_hz_),
                                   std::bind(&TakeOffLandNode::Run, this));

        // Setup DroneState
        drone_state_     = std::make_shared<DroneState>();
        global_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/robot/pose", rclcpp::SensorDataQoS(), drone_state_->GetGlobalPoseCallback());
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
            throw std::runtime_error("Invalid controller type: " + controller_type);
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
        flight_controller_ =
            std::make_shared<FlightController>(*drone_state_, take_off_delay_s, this->get_logger());
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
                    std::cerr << "Failed to send takeoff command: " << e.what() << std::endl;
                }
            });
    }

    void Run()
    {
        static bool land_command_sent = false;

        if(execution_active_)
        {
            FlightState state = flight_controller_->Run();
            if(land_command_sent && state == FlightState::kFlying)
            {
            }
            else if(state == FlightState::kFlying)
            {
                RCLCPP_INFO(this->get_logger(), "Sending Land command");
                auto request    = std::make_shared<creos_sdk_msgs::srv::SendCommand::Request>();
                request->action = creos_sdk_msgs::srv::SendCommand::Request::LAND;
                send_command_client_->async_send_request(request);
                land_command_sent = true;
            }
            else if(state == FlightState::kLanding)
            {
                land_command_sent = false;
            }
        }
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    unsigned update_frequency_hz_;
    unsigned take_off_delay_s;

    bool execution_active_ = false;

    std::shared_ptr<DroneState>        drone_state_;
    std::shared_ptr<IRemoteController> controller_;
    std::shared_ptr<FlightController>  flight_controller_;

    // ROS Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr                         controller_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global_pose_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::State>::SharedPtr                    state_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::ControlSource>::SharedPtr control_source_sub_;

    // ROS Clients
    rclcpp::Client<creos_sdk_msgs::srv::SendCommand>::SharedPtr send_command_client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TakeOffLandNode>();

    setup_logging("take_off_land_example", node->get_logger());

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
