// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#pragma once

#include <creos_sdk_msgs/msg/state_reference.hpp>
#include <math.h>
#include <rclcpp/rclcpp.hpp>

class CircleReferences
{
public:
    enum AxisIndex
    {
        kXAxis = 0,
        kYAxis = 1,
        kZAxis = 2
    };

    CircleReferences(rclcpp::Logger logger,
                     double         update_frequency_hz,
                     double         circle_radius_m_ = 2,
                     double         speed_mps        = 0.5);
    ~CircleReferences() = default;

    void Reset(const std::array<float, 3> position, const double yaw_heading);
    creos_sdk_msgs::msg::StateReference GetNewStateReference(rclcpp::Time time);

private:
    rclcpp::Logger logger_;
    const double   update_frequency_hz_;
    const double   circle_radius_m_;
    const double   circle_execution_time_s_;

    unsigned             sine_counter_        = 0;
    float                initial_heading_     = 0.0f;
    std::array<float, 3> middle_point_circle_ = {0.0f, 0.0f, 0.0f};

    unsigned totalSteps() const { return update_frequency_hz_ * circle_execution_time_s_; }
    double   angleSteps() const { return (2.0 * M_PI) / totalSteps(); }

    // Helper functions
    const creos_sdk_msgs::msg::StateReference computeNewPosition(
        const std::array<float, 3> &circle_middle,
        const double                initial_heading,
        const double                circle_angle) const;
    const std::array<float, 3> computeCircleMiddle(const std::array<float, 3> &begin_position,
                                                   const double initial_heading) const;
};
