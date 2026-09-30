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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <creos_sdk_msgs/msg/state.hpp>
#include <creos_sdk_msgs/msg/state_reference.hpp>
#include <creos_sdk_msgs/msg/control_source.hpp>

#include <common/logging.hpp>
#include <common/drone_state.hpp>
#include <common/remote_controller_interface.hpp>

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

        auto accel_param        = rcl_interfaces::msg::ParameterDescriptor{};
        accel_param.description = "Acceleration of the drone in meters per second squared. "
                                  "Default is 1.0 m/s^2.";
        this->declare_parameter("accel", 1.0, accel_param);

        auto frequency_param        = rcl_interfaces::msg::ParameterDescriptor{};
        frequency_param.description = "Update frequency in Hz. Default is 100 Hz.";
        this->declare_parameter("frequency", 100, frequency_param);

        auto controller_param = rcl_interfaces::msg::ParameterDescriptor{};
        controller_param.description =
            "Controller type to use: 'herelink' or 'jeti'. Default is 'herelink'.";
        this->declare_parameter("controller", "herelink");

        circle_radius_m_            = this->get_parameter("circle_radius").as_double();
        speed_mps_                  = this->get_parameter("speed").as_double();
        accel_mps2_                 = this->get_parameter("accel").as_double();
        update_frequency_hz_        = this->get_parameter("frequency").as_int();
        std::string controller_type = this->get_parameter("controller").as_string();

        // Check that the speed does not violate the acceleration limits.
        if(std::pow(speed_mps_, 2) > accel_mps2_ * circle_radius_m_)
        {
            const double new_speed = std::sqrt(accel_mps2_ * circle_radius_m_);
            RCLCPP_WARN(this->get_logger(),
                        "The selected speed (%f m/s) and circle radius (%f m) violate the "
                        "acceleration limit (%f m/s^2). Adjusting speed to (%f m/s) to fit "
                        "within the acceleration limits.",
                        speed_mps_, circle_radius_m_, accel_mps2_, new_speed);
            speed_mps_ = new_speed;
        }

        timer_ = create_wall_timer(std::chrono::milliseconds(1000 / update_frequency_hz_),
                                   std::bind(&CircleNode::Run, this));

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

        // Setup CircleReferences
        circle_references_ = std::make_shared<CircleReferences>(
            this->get_logger(), update_frequency_hz_, circle_radius_m_, speed_mps_, accel_mps2_);

        state_reference_pub_ = this->create_publisher<creos_sdk_msgs::msg::StateReference>(
            "/robot/cmd_state_ref", rclcpp::SensorDataQoS());
    }

    void Run()
    {
        if(execution_active_)
        {
            if(!drone_state_->IsInFlight())
            {
                // Stop execution immediately if the drone is not airborne when activated
                RCLCPP_WARN(this->get_logger(),
                            "Stopping execution: drone is not flying. "
                            "Take off manually before activating the circle example.");
                execution_active_ = false;
                circle_references_->Reset(drone_state_->GetPosition(), drone_state_->GetYaw());
                return;
            }
            if(!drone_state_->IsInUserControlMode())
            {
                // Stop execution when the drone leaves SDK mode (e.g. switch to position mode).
                // Reset so the next activation restarts from the current position.
                RCLCPP_INFO(
                    this->get_logger(),
                    "Stopping execution: control mode changed away from SDK mode. "
                    "Press activation button to restart the circle from the current position.");
                execution_active_ = false;
                circle_references_->Reset(drone_state_->GetPosition(), drone_state_->GetYaw());
                return;
            }
            creos_sdk_msgs::msg::StateReference state_reference =
                circle_references_->GetNewStateReference(this->now());
            state_reference_pub_->publish(state_reference);
        }
        // Reset the circle references when the execution is not active so that when the execution
        // is activated again, the drone will start from its current position and heading.
        // This is important because the drone might have been moved manually or drifted while the
        // execution was inactive, and we want to ensure that the circle is generated from the
        // current position and heading of the drone.
        else
        {
            circle_references_->Reset(drone_state_->GetPosition(), drone_state_->GetYaw());
        }
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    double   circle_radius_m_;
    double   speed_mps_;
    double   accel_mps2_;
    unsigned update_frequency_hz_;

    bool execution_active_ = false;

    std::shared_ptr<DroneState>        drone_state_;
    std::shared_ptr<IRemoteController> controller_;

    std::shared_ptr<CircleReferences> circle_references_;

    // ROS Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr                         controller_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr global_pose_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::State>::SharedPtr                    state_sub_;
    rclcpp::Subscription<creos_sdk_msgs::msg::ControlSource>::SharedPtr control_source_sub_;

    // ROS Publishers
    rclcpp::Publisher<creos_sdk_msgs::msg::StateReference>::SharedPtr state_reference_pub_;
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
