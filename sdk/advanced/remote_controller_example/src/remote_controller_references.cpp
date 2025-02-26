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
#include <spdlog/spdlog.h>
#include "eigen3/Eigen/Geometry"

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

const creos_messages::StateReference RemoteControllerReferences::CreateReference(
    const std::array<float, 2>         &left_stick,
    const std::array<float, 2>         &right_stick,
    const float                         latest_heading,
    const float                         time_delta_s,
    const creos::RobotClock::time_point timestamp,
    const std::string                   frame_id)
{
    // Use the latest heading as the initial heading reference
    if(!heading_ref_.has_value())
    {
        heading_ref_ = latest_heading;
    }

    creos_messages::StateReference reference = {};

    const float kCos_max_velocity_horizontal_m = cos(latest_heading) * kMax_velocity_horizontal_m_;
    const float kSin_max_velocity_horizontal_m = sin(latest_heading) * kMax_velocity_horizontal_m_;

    // The right-stick Y-axis moves the drone in the drone's X-axis (forward/backward)
    reference.velocity.linear.x = kCos_max_velocity_horizontal_m * right_stick[AxisIndex::kYAxis] -
                                  kSin_max_velocity_horizontal_m * right_stick[AxisIndex::kXAxis];
    // The right-stick X-axis moves the drone in the drone's Y-axis (left/right)
    reference.velocity.linear.y = kSin_max_velocity_horizontal_m * right_stick[AxisIndex::kYAxis] +
                                  kCos_max_velocity_horizontal_m * right_stick[AxisIndex::kXAxis];
    // The left-stick Y-axis moves the drone in the drone's Z-axis (up/down)
    reference.velocity.linear.z = kMax_velocity_vertical_m_ * -left_stick[AxisIndex::kYAxis];

    // Update the heading reference
    const float kMax_yaw_rate_step = kMax_yaw_rate_deg_ / 180 * M_PI;

    reference.translation_mode = creos_messages::StateReference::TranslationMode::kVelocity;

    if(yaw_rate_mode_)
    {
        // The left_stick X-axis changes the yaw rate of the drone
        reference.velocity.angular.z = left_stick[AxisIndex::kXAxis] * kMax_yaw_rate_step;
        reference.orientation_mode =
            creos_messages::StateReference::OrientationMode::kAngularVelocity;
    }
    else
    {
        // The left-stick X-axis changes the heading of the drone
        heading_ref_ = heading_ref_.value() +
                       (left_stick[AxisIndex::kXAxis] * kMax_yaw_rate_step * time_delta_s);

        Eigen::Quaterniond orientation(
            Eigen::AngleAxisd(heading_ref_.value(), Eigen::Vector3d::UnitZ()));
        reference.pose.orientation.x = orientation.x();
        reference.pose.orientation.y = orientation.y();
        reference.pose.orientation.z = orientation.z();
        reference.pose.orientation.w = orientation.w();

        reference.orientation_mode = creos_messages::StateReference::OrientationMode::kAttitude;
    }

    reference.timestamp = timestamp;
    reference.frame_id  = frame_id;

    return reference;
}
