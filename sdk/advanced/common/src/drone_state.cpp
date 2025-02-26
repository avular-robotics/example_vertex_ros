// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include <common/drone_state.hpp>

#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <spdlog/spdlog.h>

DroneState::DroneState(bool log_state_change) : kLog_state_change_(log_state_change) {}

const std::array<float, 3> DroneState::GetPosition()
{
    std::lock_guard<std::mutex> lock(global_pose_mutex_);
    return {
        static_cast<float>(latest_global_pose_.position.x),
        static_cast<float>(latest_global_pose_.position.y),
        static_cast<float>(latest_global_pose_.position.z),
    };
}

const creos_messages::Quaterniond &DroneState::GetOrientation()
{
    std::lock_guard<std::mutex> lock(global_pose_mutex_);
    return latest_global_pose_.orientation;
}

/**
 * @brief Get the yaw angle of the drone in radians
 */
double DroneState::GetYaw()
{
    std::lock_guard<std::mutex> lock(global_pose_mutex_);

    // Extract the quaternion from the pose
    Eigen::Quaterniond q(latest_global_pose_.orientation.w, latest_global_pose_.orientation.x,
                         latest_global_pose_.orientation.y, latest_global_pose_.orientation.z);

    // Convert the quaternion to a 3x3 rotation matrix
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

    // Extract Euler angles from the rotation matrix
    double yaw = std::atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    return yaw;
}

bool DroneState::IsArmed()
{
    if(GetState().ready_state == creos_messages::State::Ready::kWaiting &&
       GetState().current_action == "Waiting for take-off command")
    {
        return true;
    }
    else if(GetState().ready_state == creos_messages::State::Ready::kActive)
    {
        return true;
    }
    else if(GetState().ready_state == creos_messages::State::Ready::kFailsafe &&
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
    if(GetState().ready_state == creos_messages::State::Ready::kPreArm &&
       GetState().current_action == "Performing pre-flight checks")
    {
        return true;
    }
    return false;
}

bool DroneState::IsReadyForTakeOff()
{
    if(GetState().ready_state == creos_messages::State::Ready::kWaiting &&
       GetState().current_action == "Waiting for take-off command")
    {
        return true;
    }
    return false;
}

bool DroneState::IsInTakeOff()
{
    if(GetState().ready_state == creos_messages::State::Ready::kActive &&
       GetState().current_action == "Taking off")
    {
        return true;
    }
    return false;
}

bool DroneState::IsInFlight()
{
    if(GetState().ready_state == creos_messages::State::Ready::kActive &&
       GetState().current_action == "In flight")
    {
        return true;
    }
    return false;
}

bool DroneState::IsLanding()
{
    if(GetState().ready_state == creos_messages::State::Ready::kActive &&
       GetState().current_action == "Landing")
    {
        return true;
    }
    return false;
}

bool DroneState::IsInUserControlMode()
{
    std::lock_guard<std::mutex> lock(control_source_mutex_);
    return latest_control_source_ == creos_messages::ControlSource::kUser;
}

const creos_messages::ControlSource &DroneState::GetControlSource()
{
    std::lock_guard<std::mutex> lock(control_source_mutex_);
    return latest_control_source_;
}

std::function<void(const creos_messages::ControlSource &)> DroneState::GetControlSourceCallback()
{
    return std::bind(&DroneState::controlSourceCallback, this, std::placeholders::_1);
}

void DroneState::controlSourceCallback(const creos_messages::ControlSource &msg)
{
    std::lock_guard<std::mutex> lock(control_source_mutex_);
    latest_control_source_ = msg;
}

std::function<void(const creos_messages::PoseWithCovarianceStamped &)>
    DroneState::GetGlobalPoseCallback()
{
    return std::bind(&DroneState::globalPoseCallback, this, std::placeholders::_1);
}

void DroneState::globalPoseCallback(const creos_messages::PoseWithCovarianceStamped &msg)
{
    std::lock_guard<std::mutex> lock(global_pose_mutex_);
    latest_global_pose_ = msg;
}

std::function<void(const creos_messages::State &)> DroneState::GetStateCallback()
{
    return std::bind(&DroneState::stateCallback, this, std::placeholders::_1);
}

void DroneState::stateCallback(const creos_messages::State &msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if(kLog_state_change_ && (msg.current_action != latest_state_.current_action ||
                              msg.ready_state != latest_state_.ready_state))
    {
        std::string ready_state = "";
        switch(msg.ready_state)
        {
        case creos_messages::State::Ready::kUnknown:
            ready_state = "Unknown";
            break;
        case creos_messages::State::Ready::kNotReady:
            ready_state = "NotReady";
            break;
        case creos_messages::State::Ready::kPreArm:
            ready_state = "PreArm";
            break;
        case creos_messages::State::Ready::kActive:
            ready_state = "Active";
            break;
        case creos_messages::State::Ready::kWaiting:
            ready_state = "Waiting";
            break;
        case creos_messages::State::Ready::kFailsafe:
            ready_state = "Failsafe";
            break;
        case creos_messages::State::Ready::kError:
            ready_state = "Error";
            break;
        default:
            throw std::runtime_error("Unknown ready state");
        }
        spdlog::info("State: {} - {}", ready_state, msg.current_action);
    }
    latest_state_ = msg;
}

const creos_messages::State &DroneState::GetState()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_state_;
}
