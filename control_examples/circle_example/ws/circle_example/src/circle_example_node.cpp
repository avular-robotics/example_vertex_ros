// Copyright 2024 Avular B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#include <chrono>

#include <tf2/LinearMath/Transform.h>
#include <rclcpp/logger.hpp>
#include "circle_example_node.hpp"


CircleExampleNode::CircleExampleNode(rclcpp::NodeOptions options) : Node("circle_example", options)
{
    // Only support update frequencies above 10Hz.
    assert(kUpdateFrequency_Hz >= 10);


    _callback_remote_controller =
        std::bind(&CircleExampleNode::DroneRemoteControllerCallback, this, std::placeholders::_1);
    _callback_state_estimate =
        std::bind(&CircleExampleNode::DroneStateEstimateCallback, this, std::placeholders::_1);
    _callback_supervisor_mode =
        std::bind(&CircleExampleNode::DroneSupervisorModeCallback, this, std::placeholders::_1);


    _state_reference_publisher = create_publisher<avular_mavros_msgs::msg::StateReference>(
        "/robot/send/state_reference", 10);
    _supervisor_command_publisher = create_publisher<avular_mavros_msgs::msg::SupervisorCommand>(
        "/robot/send/supervisor_command", 10);


    _state_estimate_subscriber = create_subscription<nav_msgs::msg::Odometry>(
        "/robot/estimate/state", rclcpp::SensorDataQoS(), _callback_state_estimate);

    _supervisor_mode_subscriber = create_subscription<avular_mavros_msgs::msg::SupervisorMode>(
        "/robot/supervisor_mode", rclcpp::SensorDataQoS(), _callback_supervisor_mode);

    _remote_controller_subscriber = create_subscription<sensor_msgs::msg::Joy>(
        "/robot/sensor/remote_controller/data", rclcpp::SensorDataQoS(),
        _callback_remote_controller);

    _set_control_mode_service =
        create_client<avular_mavros_msgs::srv::SetControlMode>("/robot/set_control_mode");
}

const std::array<float, 3> CircleExampleNode::ComputeCircleMiddle(
    const std::array<float, 3> &begin_position,
    const double                initial_heading) const
{
    // Compute the middle point of the circle based on the heading of the drone.
    std::array<float, 3> middle_point;
    middle_point[0] = begin_position[0] + cos(initial_heading) * kCircleRadius_m; // x
    middle_point[1] = begin_position[1] + sin(initial_heading) * kCircleRadius_m; // y
    middle_point[2] = begin_position[2];                                          // z

    return middle_point;
}

const CircleExampleNode::StateReference CircleExampleNode::ComputeNewPosition(
    const std::array<float, 3> &circle_middle,
    const double                initial_heading,
    const double                circle_angle) const
{
    // Compute the next position.
    CircleExampleNode::StateReference reference;

    reference.position_ref[0] =
        circle_middle[0] + (-kCircleRadius_m * cos(circle_angle + initial_heading));
    reference.position_ref[1] =
        circle_middle[1] + (-kCircleRadius_m * sin(circle_angle + initial_heading));

    // compute feedforward velocity
    reference.velocity_ref[0] =
        kCircleRadius_m * sin(circle_angle + initial_heading) * (2 * M_PI / kCircleExecutionTime_s);
    reference.velocity_ref[1] = -kCircleRadius_m * cos(circle_angle + initial_heading) *
                                (2 * M_PI / kCircleExecutionTime_s);

    // Compute feedforward acceleration
    reference.acceleration_ref[0] = kCircleRadius_m * cos(circle_angle + initial_heading) *
                                    pow(2 * M_PI / kCircleExecutionTime_s, 2);
    reference.acceleration_ref[1] = kCircleRadius_m * sin(circle_angle + initial_heading) *
                                    pow(2 * M_PI / kCircleExecutionTime_s, 2);

    // Keep height constant at the start height
    reference.position_ref[2]     = circle_middle[2];
    reference.velocity_ref[2]     = 0;
    reference.acceleration_ref[2] = 0;

    // Keep heading constant
    reference.heading_ref = initial_heading;

    return reference;
}


void CircleExampleNode::FlyCircle(bool first_step)
{
    static constexpr unsigned kNumberOfSteps = kUpdateFrequency_Hz * kCircleExecutionTime_s;
    static constexpr double   kAngleSteps    = (2.0 * M_PI) / kNumberOfSteps;

    static unsigned sine_counter = 0;

    // First time the execution continuous, save the current heading and position to
    // define the starting point.
    if(first_step)
    {
        // First get the current position and heading from the flight controller
        // estimator
        _drone_initial_heading = GetLatestHeading().heading_estimate;
        auto begin_position    = GetLatestPosition().position_estimate;

        _middle_point_circle = ComputeCircleMiddle(begin_position, _drone_initial_heading);

        RCLCPP_INFO(get_logger(), "Middle point: %f, %f\tHeading: %f", _middle_point_circle[0],
                    _middle_point_circle[1], _drone_initial_heading);

        // Restart counter so you will begin from the start of the circle
        sine_counter = 0;
    }

    // Compute the current angle in the circle.
    double circle_angle = sine_counter * kAngleSteps;
    auto reference = ComputeNewPosition(_middle_point_circle, _drone_initial_heading, circle_angle);

    // Get current time to timestamp the generated references
    struct timespec current_time;
    timespec_get(&current_time, TIME_UTC);

    reference.timestamp.tv_sec  = current_time.tv_sec;
    reference.timestamp.tv_nsec = current_time.tv_nsec;

    // Now publish the references
    PublishStateReference(reference);

    // only continue counting when active so it continues flying the circle of you pause
    // it.
    sine_counter++;
    if(sine_counter >= kNumberOfSteps)
    {
        sine_counter = 0;
    }
}

void CircleExampleNode::ExecuteExample()
{
    static unsigned take_off_count    = 0;
    static bool     first_flying_step = true;


    auto &clk = *this->get_clock();

    // if the execution is not activation or when it is not in auto mode, force deactivate the
    // execution and reset all counters + states.
    if(!_execution_activated || !IsInAutoMode())
    {
        first_flying_step    = true;
        _execution_activated = false;
        take_off_count       = 0;
        return;
    }

    if(IsModeDisarmed())
    {
        // Arm drone
        if(GetCurrentControlMode() !=
           avular_mavros_msgs::msg::SupervisorMode::EXTERNAL_MODE_POSITION)
        {
            // Set control mode to Position references
            RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000, "Set Control mode");
            SetControlMode(avular_mavros_msgs::msg::ControlMode::CONTROL_MODE_POSITION,
                           avular_mavros_msgs::msg::ControlMode::HEADING_MODE_ANGLE);
        }
        else
        {
            // Arming in this example, should be done for safety reason manually. This can be
            // automated but more protective measures should be implemented.
            RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000,
                                 "Ready for take-off. Arm the drone with your remote!");
        }
    }
    else if(IsModeArmed())
    {
        // Wait for one second until arm is completed.
        if(take_off_count++ < kUpdateFrequency_Hz)
        {
            return;
        }

        if(IsIdleMode())
        {
            RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000, "Requested Take-off");
            PublishSupervisorCommand(
                avular_mavros_msgs::msg::SupervisorCommand::VERTEX_COMMAND_TAKEOFF);
        }
        else if(IsInTakeOff())
        {
            RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000, "Take-off");
            // Wait until take-off is ready
        }
        else if(IsInFlight())
        {
            RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000, "Flying");
            FlyCircle(first_flying_step);
            first_flying_step = false;
        }
    }
}


void CircleExampleNode::PublishStateReference(const StateReference &reference)
{
    avular_mavros_msgs::msg::StateReference message;
    message.header.stamp.sec      = reference.timestamp.tv_sec;
    message.header.stamp.nanosec  = reference.timestamp.tv_nsec;
    message.header.frame_id       = "NED_odom";
    message.pose.position.x       = reference.position_ref[0];
    message.pose.position.y       = reference.position_ref[1];
    message.pose.position.z       = reference.position_ref[2];
    message.velocity.linear.x     = reference.velocity_ref[0];
    message.velocity.linear.y     = reference.velocity_ref[1];
    message.velocity.linear.z     = reference.velocity_ref[2];
    message.acceleration.linear.x = reference.acceleration_ref[0];
    message.acceleration.linear.y = reference.acceleration_ref[1];
    message.acceleration.linear.z = reference.acceleration_ref[2];

    tf2::Quaternion orientation;
    orientation.setEuler(0, 0, reference.heading_ref);

    message.pose.orientation.x = orientation.getX();
    message.pose.orientation.y = orientation.getY();
    message.pose.orientation.z = orientation.getZ();
    message.pose.orientation.w = orientation.getW();


    _state_reference_publisher->publish(message);
}

void CircleExampleNode::PublishSupervisorCommand(const uint8_t supervisor_mode)
{
    avular_mavros_msgs::msg::SupervisorCommand message;
    message.command = supervisor_mode;

    _supervisor_command_publisher->publish(message);
}

int CircleExampleNode::SetControlMode(const uint8_t control_mode, const uint8_t heading_mode)
{
    using namespace std::chrono_literals; // Required for '1s' literal

    // Get current time to timestamp the generated references
    struct timespec current_time;
    timespec_get(&current_time, TIME_UTC);

    auto request = std::make_shared<avular_mavros_msgs::srv::SetControlMode::Request>();
    request->control_mode.header.stamp.sec     = current_time.tv_sec;
    request->control_mode.header.stamp.nanosec = current_time.tv_nsec;
    request->control_mode.control_mode         = control_mode;
    request->control_mode.heading_mode         = heading_mode;

    // Check if the service is available
    _set_control_mode_service->wait_for_service(std::chrono::duration<int, std::milli>(200));
    if(!rclcpp::ok())
    {
        RCLCPP_INFO(get_logger(), "Set control mode not available");
        return -EBUSY;
    }


    // Send request and wait for response.
    auto result = _set_control_mode_service->async_send_request(request);

    auto &clk = *this->get_clock();

    auto status = rclcpp::spin_until_future_complete(this->get_node_base_interface(), result,
                                                     std::chrono::duration<int, std::milli>(200));
    switch(status)
    {
    case rclcpp::FutureReturnCode::SUCCESS:
    {
        auto response = result.get();
        RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000,
                             "Set control mode reponse: status=%d, message=%s", response->status,
                             response->message.c_str());
        return response->status;
    }
    case rclcpp::FutureReturnCode::TIMEOUT:
        RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000, "Set control mode request timed out!");
        return -ETIMEDOUT;
    case rclcpp::FutureReturnCode::INTERRUPTED:
        RCLCPP_INFO_THROTTLE(get_logger(), clk, 1000, "Set control mode request interrupted!");
        return -EINTR;
    }
    return -EIO;
}


CircleExampleNode::DronePosition CircleExampleNode::GetLatestPosition()
{
    std::lock_guard<std::mutex> lock(_state_estimate_mutex);

    return _latest_position;
}


CircleExampleNode::DroneHeading CircleExampleNode::GetLatestHeading()
{
    std::lock_guard<std::mutex> lock(_state_estimate_mutex);

    return _latest_heading;
}

bool CircleExampleNode::IsModeArmed()
{
    std::lock_guard<std::mutex> lock(_supervisor_mode_mutex);
    return _latest_supervisor_mode.mode_of_flight ==
           avular_mavros_msgs::msg::SupervisorMode::FLIGHT_MODE_ARMED;
};

bool CircleExampleNode::IsModeDisarmed()
{
    std::lock_guard<std::mutex> lock(_supervisor_mode_mutex);
    return _latest_supervisor_mode.mode_of_flight ==
           avular_mavros_msgs::msg::SupervisorMode::FLIGHT_MODE_DISARMED;
};

bool CircleExampleNode::IsIdleMode()
{
    std::lock_guard<std::mutex> lock(_supervisor_mode_mutex);
    return _latest_supervisor_mode.mode_armed ==
           avular_mavros_msgs::msg::SupervisorMode::ARMED_MODE_IDLE;
};

bool CircleExampleNode::IsInTakeOff()
{
    std::lock_guard<std::mutex> lock(_supervisor_mode_mutex);
    return _latest_supervisor_mode.mode_armed ==
           avular_mavros_msgs::msg::SupervisorMode::ARMED_MODE_TAKE_OFF;
};

bool CircleExampleNode::IsInFlight()
{
    std::lock_guard<std::mutex> lock(_supervisor_mode_mutex);
    return _latest_supervisor_mode.mode_armed ==
           avular_mavros_msgs::msg::SupervisorMode::ARMED_MODE_IN_FLIGHT;
};

bool CircleExampleNode::IsInAutoMode()
{
    std::lock_guard<std::mutex> lock(_supervisor_mode_mutex);
    return _latest_supervisor_mode.mode_of_operation ==
           avular_mavros_msgs::msg::SupervisorMode::OPERATION_MODE_POSITION_EXTERNAL;
};

uint8_t CircleExampleNode::GetCurrentControlMode()
{
    std::lock_guard<std::mutex> lock(_supervisor_mode_mutex);
    return _latest_supervisor_mode.mode_external_control;
};


void CircleExampleNode::DroneRemoteControllerCallback(
    const sensor_msgs::msg::Joy::SharedPtr message)
{
    // Activate the execution based on button press.
    if(message->buttons[ButtonIndex::kButtonC] == 1) // detect short press on button C
    {
        RCLCPP_INFO(get_logger(), "Button C pressed");

        _execution_activated = !_execution_activated;
        RCLCPP_INFO(get_logger(), "Example execution state changed to: %s",
                    _execution_activated ? "True" : "False");
    }
}

void CircleExampleNode::DroneStateEstimateCallback(const nav_msgs::msg::Odometry::SharedPtr message)
{
    std::lock_guard<std::mutex> lock(_state_estimate_mutex);

    _latest_position.position_estimate[0] = message->pose.pose.position.x;
    _latest_position.position_estimate[1] = message->pose.pose.position.y;
    _latest_position.position_estimate[2] = message->pose.pose.position.z;

    _latest_position.timestamp.tv_sec  = message->header.stamp.sec;
    _latest_position.timestamp.tv_nsec = message->header.stamp.nanosec;

    // Extract the quaternion from the pose
    const auto &q = message->pose.pose.orientation;

    // Convert quaternion to tf2::Quaternion
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);

    // Convert quaternion to Euler angles
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    _latest_heading.heading_estimate = yaw;

    _latest_heading.timestamp.tv_sec  = message->header.stamp.sec;
    _latest_heading.timestamp.tv_nsec = message->header.stamp.nanosec;
}

void CircleExampleNode::DroneSupervisorModeCallback(
    const avular_mavros_msgs::msg::SupervisorMode::SharedPtr message)
{
    std::lock_guard<std::mutex> lock(_supervisor_mode_mutex);

    _latest_supervisor_mode.mode_supervisor       = message->mode_supervisor;
    _latest_supervisor_mode.mode_armed            = message->mode_armed;
    _latest_supervisor_mode.mode_of_operation     = message->mode_of_operation;
    _latest_supervisor_mode.mode_of_flight        = message->mode_of_flight;
    _latest_supervisor_mode.mode_external_control = message->mode_external_control;
}
