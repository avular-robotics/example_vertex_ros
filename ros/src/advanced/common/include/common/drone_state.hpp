// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include "drone_state_interface.hpp"

#include <creos_sdk_msgs/msg/control_source.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <creos_sdk_msgs/msg/state.hpp>

#include <mutex>
#include <rclcpp/rclcpp.hpp>

class DroneState : public IDroneState
{
public:
    DroneState(std::optional<rclcpp::Logger> logger = std::nullopt);
    ~DroneState() = default;

    const std::array<float, 3>                GetPosition();
    const geometry_msgs::msg::Quaternion     &GetOrientation();
    double                                    GetYaw() override;
    const creos_sdk_msgs::msg::State         &GetState();
    const creos_sdk_msgs::msg::ControlSource &GetControlSource() override;

    bool IsArmed() override;
    bool IsDisarmed() override;
    bool IsPerformingPreFlightChecks() override;
    bool IsReadyForTakeOff() override;
    bool IsInTakeOff() override;
    bool IsInFlight() override;
    bool IsLanding() override;
    bool IsInUserControlMode() override;

    std::function<void(const nav_msgs::msg::Odometry &)>            GetOdometryCallback();
    std::function<void(const creos_sdk_msgs::msg::ControlSource &)> GetControlSourceCallback();
    std::function<void(const creos_sdk_msgs::msg::State &)>         GetStateCallback();

private:
    const std::optional<rclcpp::Logger> logger_;

    std::mutex              odometry_mutex_;
    nav_msgs::msg::Odometry latest_odometry_;
    void                    odometryCallback(const nav_msgs::msg::Odometry &msg);

    std::mutex                         control_source_mutex_;
    creos_sdk_msgs::msg::ControlSource latest_control_source_;
    void controlSourceCallback(const creos_sdk_msgs::msg::ControlSource &msg);

    std::mutex                 state_mutex_;
    creos_sdk_msgs::msg::State latest_state_;
    void                       stateCallback(const creos_sdk_msgs::msg::State &msg);
};
