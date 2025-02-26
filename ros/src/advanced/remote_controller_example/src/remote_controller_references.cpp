// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include "remote_controller_references.hpp"
#include <cassert>
#include <math.h>
#include <tf2/LinearMath/Transform.h>

RemoteControllerReferences::RemoteControllerReferences(double max_velocity_horizontal_m,
                                                       double max_velocity_vertical_m,
                                                       double max_yaw_rate_deg,
                                                       bool   yaw_rate_mode)
    : kMax_velocity_horizontal_m_(max_velocity_horizontal_m),
      kMax_velocity_vertical_m_(max_velocity_vertical_m),
      kMax_yaw_rate_deg_(max_yaw_rate_deg),
      yaw_rate_mode_(yaw_rate_mode)
{
}

void RemoteControllerReferences::Reset()
{
    heading_ref_ = std::nullopt;
}

void RemoteControllerReferences::SetYawRateMode(bool yaw_rate_mode)
{
    yaw_rate_mode_ = yaw_rate_mode;
}

bool RemoteControllerReferences::GetYawRateMode() const
{
    return yaw_rate_mode_;
}

const creos_sdk_msgs::msg::StateReference RemoteControllerReferences::CreateReference(
    const std::array<float, 2>         &left_stick,
    const std::array<float, 2>         &right_stick,
    const float                         latest_heading,
    const float                         time_delta_s,
    const builtin_interfaces::msg::Time time,
    const std::string                   frame_id)
{
    // Use the latest heading as the initial heading reference
    if(!heading_ref_.has_value())
    {
        heading_ref_ = latest_heading;
    }

    creos_sdk_msgs::msg::StateReference reference;

    const float kCos_max_velocity_horizontal_m = cos(latest_heading) * kMax_velocity_horizontal_m_;
    const float kSin_max_velocity_horizontal_m = sin(latest_heading) * kMax_velocity_horizontal_m_;

    // The right-stick Y-axis moves the drone in the drone's X-axis (forward/backward)
    reference.twist.linear.x = kCos_max_velocity_horizontal_m * right_stick[AxisIndex::kYAxis] -
                               kSin_max_velocity_horizontal_m * right_stick[AxisIndex::kXAxis];
    // The right-stick X-axis moves the drone in the drone's Y-axis (left/right)
    reference.twist.linear.y = kSin_max_velocity_horizontal_m * right_stick[AxisIndex::kYAxis] +
                               kCos_max_velocity_horizontal_m * right_stick[AxisIndex::kXAxis];
    // The left-stick Y-axis moves the drone in the drone's Z-axis (up/down)
    reference.twist.linear.z = kMax_velocity_vertical_m_ * -left_stick[AxisIndex::kYAxis];

    // Update the heading reference
    const float kMax_yaw_rate_step = kMax_yaw_rate_deg_ / 180 * M_PI;

    reference.translation_mode = creos_sdk_msgs::msg::StateReference::TRANSLATION_MODE_VELOCITY;

    if(yaw_rate_mode_)
    {
        // The left_stick X-axis changes the yaw rate of the drone
        reference.twist.angular.z = left_stick[AxisIndex::kXAxis] * kMax_yaw_rate_step;
        reference.orientation_mode =
            creos_sdk_msgs::msg::StateReference::ORIENTATION_MODE_ANGULAR_VELOCITY;
    }
    else
    {
        // The left-stick X-axis changes the heading of the drone
        heading_ref_ = heading_ref_.value() +
                       (left_stick[AxisIndex::kXAxis] * kMax_yaw_rate_step * time_delta_s);

        tf2::Quaternion quat;
        quat.setRPY(0, 0, heading_ref_.value());
        reference.pose.orientation.x = quat.x();
        reference.pose.orientation.y = quat.y();
        reference.pose.orientation.z = quat.z();
        reference.pose.orientation.w = quat.w();
        reference.orientation_mode = creos_sdk_msgs::msg::StateReference::ORIENTATION_MODE_ATTITUDE;
    }

    reference.header.stamp    = time;
    reference.header.frame_id = frame_id;

    return reference;
}
