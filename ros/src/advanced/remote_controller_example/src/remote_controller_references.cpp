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
#include <eigen3/Eigen/Geometry>

RemoteControllerReferences::RemoteControllerReferences(double max_velocity_horizontal_m,
                                                       double max_velocity_vertical_m,
                                                       double max_yaw_rate_deg)
    : kMax_velocity_horizontal_m_(max_velocity_horizontal_m),
      kMax_velocity_vertical_m_(max_velocity_vertical_m),
      kMax_yaw_rate_deg_(max_yaw_rate_deg)
{
}

const creos_sdk_msgs::msg::StateReference RemoteControllerReferences::CreateReference(
    const std::array<float, 2>           &left_stick,
    const std::array<float, 2>           &right_stick,
    const geometry_msgs::msg::Quaternion &latest_attitude,
    const builtin_interfaces::msg::Time   time,
    const std::string                     frame_id)
{
    creos_sdk_msgs::msg::StateReference reference;

    // Extract the heading (yaw) from the latest attitude quaternion.
    const tf2::Quaternion attitude(latest_attitude.x, latest_attitude.y, latest_attitude.z,
                                   latest_attitude.w);
    const tf2::Vector3    rotated_x_axis = tf2::quatRotate(attitude, tf2::Vector3(1.0, 0.0, 0.0));
    const double          heading        = std::atan2(rotated_x_axis.y(), rotated_x_axis.x());
    const tf2::Quaternion heading_quaternion(tf2::Vector3(0.0, 0.0, 1.0), heading);

    // Stick vector.
    // The right-stick Y-axis is forward/backward (X-axis)
    // The right-stick X-axis is left/right (Y-axis)
    // The left-stick Y-axis is up/down (Z-axis)
    tf2::Vector3 stick_vector(right_stick[AxisIndex::kYAxis],  // Forward/Backward
                              -right_stick[AxisIndex::kXAxis], // Left/Right
                              left_stick[AxisIndex::kYAxis]    // Up/Down
    );

    // Normalize the stick vector to ensure its magnitude does not exceed 1.
    const double horizontal_magnitude = std::hypot(stick_vector.x(), stick_vector.y());
    if(horizontal_magnitude > 1.0)
    {
        stick_vector.setX(stick_vector.x() / horizontal_magnitude);
        stick_vector.setY(stick_vector.y() / horizontal_magnitude);
    }
    if(std::abs(stick_vector.z()) > 1.0)
    {
        stick_vector.setZ(stick_vector.z() / std::abs(stick_vector.z()));
    }

    // Multiply the stick vector by the maximum velocities to get the desired velocity in level
    // frame.
    const tf2::Vector3 velocity_level_frame =
        tf2::Vector3(kMax_velocity_horizontal_m_ * stick_vector.x(),
                     kMax_velocity_horizontal_m_ * stick_vector.y(),
                     kMax_velocity_vertical_m_ * stick_vector.z());

    // Convert the level frame velocity vector to the body frame.
    const tf2::Vector3 velocity_body_frame =
        tf2::quatRotate(attitude.inverse() * heading_quaternion, velocity_level_frame);

    // Set the linear velocity references in the reference message.
    reference.twist.linear.x   = velocity_body_frame.x();
    reference.twist.linear.y   = velocity_body_frame.y();
    reference.twist.linear.z   = velocity_body_frame.z();
    reference.translation_mode = creos_sdk_msgs::msg::StateReference::TRANSLATION_MODE_VELOCITY;

    // Update the yaw rate reference.
    // The left_stick X-axis changes the yaw rate of the drone
    reference.twist.angular.z =
        -left_stick[AxisIndex::kXAxis] * (kMax_yaw_rate_deg_ / 180.0 * M_PI);
    reference.orientation_mode =
        creos_sdk_msgs::msg::StateReference::ORIENTATION_MODE_ANGULAR_VELOCITY;

    // Set the header.
    reference.header.stamp    = time;
    reference.header.frame_id = frame_id;

    return reference;
}
