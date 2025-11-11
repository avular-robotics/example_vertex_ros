// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include <common/drone_state.hpp>

#include <tf2/LinearMath/Transform.h>
#include <iostream>

DroneState::DroneState(std::optional<rclcpp::Logger> logger) : logger_(logger) {}

const std::array<float, 3> DroneState::GetPosition()
{
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    return {
        static_cast<float>(latest_odometry_.pose.pose.position.x),
        static_cast<float>(latest_odometry_.pose.pose.position.y),
        static_cast<float>(latest_odometry_.pose.pose.position.z),
    };
}

const geometry_msgs::msg::Quaternion &DroneState::GetOrientation()
{
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    return latest_odometry_.pose.pose.orientation;
}

/**
 * @brief Get the yaw angle of the drone in radians
 */
double DroneState::GetYaw()
{
    std::lock_guard<std::mutex> lock(odometry_mutex_);

    // Extract the quaternion from the pose
    tf2::Quaternion quat(
        latest_odometry_.pose.pose.orientation.x, latest_odometry_.pose.pose.orientation.y,
        latest_odometry_.pose.pose.orientation.z, latest_odometry_.pose.pose.orientation.w);

    // Convert quaternion to Euler angles
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

bool DroneState::IsArmed()
{
    if(GetState().ready_state == creos_sdk_msgs::msg::State::WAITING &&
       GetState().current_action == "Waiting for take-off command")
    {
        return true;
    }
    else if(GetState().ready_state == creos_sdk_msgs::msg::State::ACTIVE)
    {
        return true;
    }
    else if(GetState().ready_state == creos_sdk_msgs::msg::State::FAILSAFE &&
            GetState().current_action == "Performing failsafe action")
    {
        return true;
    }
    return false;
}

bool DroneState::IsDisarmed()
{
    return !IsArmed();
}

bool DroneState::IsPerformingPreFlightChecks()
{
    if(GetState().ready_state == creos_sdk_msgs::msg::State::PRE_ARM &&
       GetState().current_action == "Performing pre-flight checks")
    {
        return true;
    }
    return false;
}

bool DroneState::IsReadyForTakeOff()
{
    if(GetState().ready_state == creos_sdk_msgs::msg::State::WAITING &&
       GetState().current_action == "Waiting for take-off command")
    {
        return true;
    }
    return false;
}

bool DroneState::IsInTakeOff()
{
    if(GetState().ready_state == creos_sdk_msgs::msg::State::ACTIVE &&
       GetState().current_action == "Taking off")
    {
        return true;
    }
    return false;
}

bool DroneState::IsInFlight()
{
    if(GetState().ready_state == creos_sdk_msgs::msg::State::ACTIVE &&
       GetState().current_action == "In flight")
    {
        return true;
    }
    return false;
}

bool DroneState::IsLanding()
{
    if(GetState().ready_state == creos_sdk_msgs::msg::State::ACTIVE &&
       GetState().current_action == "Landing")
    {
        return true;
    }
    return false;
}

bool DroneState::IsInUserControlMode()
{
    std::lock_guard<std::mutex> lock(control_source_mutex_);
    return latest_control_source_.source == creos_sdk_msgs::msg::ControlSource::USER;
}

const creos_sdk_msgs::msg::ControlSource &DroneState::GetControlSource()
{
    std::lock_guard<std::mutex> lock(control_source_mutex_);
    return latest_control_source_;
}

std::function<void(const creos_sdk_msgs::msg::ControlSource &)>
    DroneState::GetControlSourceCallback()
{
    return std::bind(&DroneState::controlSourceCallback, this, std::placeholders::_1);
}

void DroneState::controlSourceCallback(const creos_sdk_msgs::msg::ControlSource &msg)
{
    std::lock_guard<std::mutex> lock(control_source_mutex_);
    latest_control_source_ = msg;
}

std::function<void(const nav_msgs::msg::Odometry &)> DroneState::GetOdometryCallback()
{
    return std::bind(&DroneState::odometryCallback, this, std::placeholders::_1);
}

void DroneState::odometryCallback(const nav_msgs::msg::Odometry &msg)
{
    std::lock_guard<std::mutex> lock(odometry_mutex_);
    latest_odometry_ = msg;
}

std::function<void(const creos_sdk_msgs::msg::State &)> DroneState::GetStateCallback()
{
    return std::bind(&DroneState::stateCallback, this, std::placeholders::_1);
}

void DroneState::stateCallback(const creos_sdk_msgs::msg::State &msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if(logger_.has_value() && (msg.current_action != latest_state_.current_action ||
                               msg.ready_state != latest_state_.ready_state))
    {
        std::string ready_state = "";
        switch(msg.ready_state)
        {
        case creos_sdk_msgs::msg::State::UNKNOWN:
            ready_state = "Unknown";
            break;
        case creos_sdk_msgs::msg::State::NOT_READY:
            ready_state = "NotReady";
            break;
        case creos_sdk_msgs::msg::State::PRE_ARM:
            ready_state = "PreArm";
            break;
        case creos_sdk_msgs::msg::State::ACTIVE:
            ready_state = "Active";
            break;
        case creos_sdk_msgs::msg::State::WAITING:
            ready_state = "Waiting";
            break;
        case creos_sdk_msgs::msg::State::FAILSAFE:
            ready_state = "Failsafe";
            break;
        case creos_sdk_msgs::msg::State::ERROR:
            ready_state = "Error";
            break;
        default:
            throw std::runtime_error("Unknown ready state");
        }

        RCLCPP_INFO(logger_.value(), "State: %s - %s", ready_state.c_str(),
                    msg.current_action.c_str());
    }
    latest_state_ = msg;
}

const creos_sdk_msgs::msg::State &DroneState::GetState()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}
