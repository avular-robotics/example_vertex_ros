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
                                                       double max_yaw_rate_deg)
    : kMax_velocity_horizontal_m_(max_velocity_horizontal_m),
      kMax_velocity_vertical_m_(max_velocity_vertical_m),
      kMax_yaw_rate_deg_(max_yaw_rate_deg)
{
}

const creos_messages::StateReference RemoteControllerReferences::CreateReference(
    const std::array<float, 2>         &left_stick,
    const std::array<float, 2>         &right_stick,
    const creos_messages::Quaterniond  &latest_attitude,
    const creos::RobotClock::time_point timestamp,
    const std::string                   frame_id)
{
    creos_messages::StateReference reference = {};

    // Extract the heading (yaw) from the latest attitude quaternion.
    const Eigen::Quaterniond attitude(latest_attitude.w, latest_attitude.x, latest_attitude.y,
                                      latest_attitude.z);
    const Eigen::Vector3d    rotated_x_axis = attitude * Eigen::Vector3d::UnitX();
    const double             heading        = std::atan2(rotated_x_axis.y(), rotated_x_axis.x());

    const Eigen::Quaterniond heading_quaternion(
        Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));

    // Stick vector.
    // The right-stick Y-axis is forward/backward (X-axis)
    // The right-stick X-axis is left/right (Y-axis)
    // The left-stick Y-axis is up/down (Z-axis)
    Eigen::Vector3d stick_vector(right_stick[AxisIndex::kYAxis],  // Forward/Backward
                                 -right_stick[AxisIndex::kXAxis], // Left/Right
                                 left_stick[AxisIndex::kYAxis]    // Up/Down
    );

    // Normalize the stick vector to ensure its magnitude does not exceed 1.
    if(stick_vector.head<2>().norm() > 1.0)
    {
        stick_vector.head<2>().normalize();
    }
    if(stick_vector.tail<1>().norm() > 1.0)
    {
        stick_vector.tail<1>().normalize();
    }

    // Multiply the stick vector by the maximum velocities to get the desired velocity in level
    // frame.
    const Eigen::Vector3d level_velocity_vector(kMax_velocity_horizontal_m_ * stick_vector.x(),
                                                kMax_velocity_horizontal_m_ * stick_vector.y(),
                                                kMax_velocity_vertical_m_ * stick_vector.z());

    // Convert the level frame velocity vector to the body frame.
    const Eigen::Vector3d body_velocity_vector =
        attitude.inverse() * heading_quaternion * level_velocity_vector;

    // Set the velocity in the reference message.
    reference.velocity.linear.x = body_velocity_vector.x();
    reference.velocity.linear.y = body_velocity_vector.y();
    reference.velocity.linear.z = body_velocity_vector.z();
    reference.translation_mode  = creos_messages::StateReference::TranslationMode::kVelocity;

    // Update the yaw rate.
    // The left_stick X-axis changes the yaw rate of the drone
    reference.velocity.angular.z = -left_stick[AxisIndex::kXAxis] * kMax_yaw_rate_deg_ / 180 * M_PI;
    reference.orientation_mode = creos_messages::StateReference::OrientationMode::kAngularVelocity;

    // Set the timestamp and frame_id.
    reference.timestamp = timestamp;
    reference.frame_id  = frame_id;

    return reference;
}
