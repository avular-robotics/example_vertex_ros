// Copyright 2024 Avular B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <avular_mavros_msgs/msg/state_reference.hpp>
#include <avular_mavros_msgs/msg/supervisor_mode.hpp>
#include <avular_mavros_msgs/msg/supervisor_command.hpp>
#include <avular_mavros_msgs/srv/set_control_mode.hpp>

class CircleExampleNode : public rclcpp::Node
{
public:
    static constexpr unsigned kUpdateFrequency_Hz    = 100;
    static constexpr unsigned kCircleExecutionTime_s = 20;
    static constexpr double   kCircleRadius_m        = 2;
    static constexpr unsigned kMsInSecond            = 1000;

    enum ButtonIndex
    {
        kButtonA       = 0,
        kButtonB       = 1,
        kButtonC       = 2,
        kButtonD       = 3,
        kButtonTrigger = 4,
        kButtonHome    = 5,
        kWheel         = 6
    };

    enum AxisIndex
    {
        kXAxis = 0,
        kYAxis = 1,
        kZAxis = 2
    };

    struct StateReference
    {
        std::array<float, 3> position_ref;
        std::array<float, 3> velocity_ref;
        std::array<float, 3> acceleration_ref;
        float                heading_ref;
        struct timespec      timestamp;
    };

    struct DronePosition
    {
        std::array<float, 3> position_estimate;
        struct timespec      timestamp;
    };

    struct DroneHeading
    {
        double          heading_estimate;
        struct timespec timestamp;
    };

    struct SupervisorMode
    {
        uint8_t mode_supervisor;
        uint8_t mode_armed;
        uint8_t mode_of_operation;
        uint8_t mode_of_flight;
        uint8_t mode_external_control;
    };

    struct RemoteControllerInput
    {
        std::array<float, 2> left_stick;
        std::array<float, 2> right_stick;
        int                  button_a;
        int                  button_b;
        int                  button_c;
        int                  button_d;
        int                  button_home;
        int                  button_trigger;
    };

    CircleExampleNode(rclcpp::NodeOptions options);

    void ExecuteExample();
    void PublishStateReference(const StateReference &reference);
    void PublishSupervisorCommand(const uint8_t supervisor_mode);
    int  SetControlMode(const uint8_t control_mode, const uint8_t heading_mode);


    DronePosition GetLatestPosition();
    DroneHeading  GetLatestHeading();
    bool          IsModeArmed();
    bool          IsModeDisarmed();
    bool          IsIdleMode();
    bool          IsInTakeOff();
    bool          IsInFlight();
    bool          IsInAutoMode();
    uint8_t       GetCurrentControlMode();

    const StateReference       ComputeNewPosition(const std::array<float, 3> &circle_middle,
                                                  const double                initial_heading,
                                                  const double                circle_angle) const;
    const std::array<float, 3> ComputeCircleMiddle(const std::array<float, 3> &begin_position,
                                                   const double initial_heading) const;

private:
    DronePosition  _latest_position;
    DroneHeading   _latest_heading;
    SupervisorMode _latest_supervisor_mode;

    std::mutex _state_estimate_mutex;
    std::mutex _supervisor_mode_mutex;

    bool                 _execution_activated   = false;
    std::array<float, 3> _middle_point_circle   = {0, 0, 0};
    float                _drone_initial_heading = 0;

    rclcpp::TimerBase::SharedPtr _timer;


    rclcpp::Publisher<avular_mavros_msgs::msg::StateReference>::SharedPtr
        _state_reference_publisher;
    rclcpp::Publisher<avular_mavros_msgs::msg::SupervisorCommand>::SharedPtr
        _supervisor_command_publisher;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _state_estimate_subscriber;
    rclcpp::Subscription<avular_mavros_msgs::msg::SupervisorMode>::SharedPtr
                                                           _supervisor_mode_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _remote_controller_subscriber;

    rclcpp::Client<avular_mavros_msgs::srv::SetControlMode>::SharedPtr _set_control_mode_service;


    std::function<void(const nav_msgs::msg::Odometry::SharedPtr)> _callback_state_estimate;
    std::function<void(const avular_mavros_msgs::msg::SupervisorMode::SharedPtr)>
                                                                _callback_supervisor_mode;
    std::function<void(const sensor_msgs::msg::Joy::SharedPtr)> _callback_remote_controller;

    void FlyCircle(bool first_step);

    void DroneRemoteControllerCallback(const sensor_msgs::msg::Joy::SharedPtr message);
    void DroneStateEstimateCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void DroneSupervisorModeCallback(const avular_mavros_msgs::msg::SupervisorMode::SharedPtr msg);
};
