// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include "circle_references.hpp"
#include <tf2/LinearMath/Transform.h>

CircleReferences::CircleReferences(rclcpp::Logger logger,
                                   double         update_frequency_hz,
                                   double         circle_radius_m,
                                   double         speed_mps)
    : logger_(logger),
      update_frequency_hz_(update_frequency_hz),
      circle_radius_m_(circle_radius_m),
      circle_execution_time_s_((2 * M_PI * circle_radius_m) / speed_mps)
{
    // Only support update frequencies above 10Hz.
    assert(update_frequency_hz >= 10);
}

void CircleReferences::Reset(const std::array<float, 3> position, const double yaw_heading)
{
    initial_heading_     = yaw_heading;
    middle_point_circle_ = computeCircleMiddle(position, initial_heading_);

    RCLCPP_DEBUG(logger_, "CircleReferences: Middle point: %f, %f, Heading: %f",
                 middle_point_circle_[AxisIndex::kXAxis], middle_point_circle_[AxisIndex::kYAxis],
                 initial_heading_);

    // Restart counter so you will begin from the start of the circle
    sine_counter_ = 0;
}

creos_sdk_msgs::msg::StateReference CircleReferences::GetNewStateReference(const rclcpp::Time time)
{
    // Compute the current angle in the circle.
    double circle_angle = sine_counter_ * angleSteps();
    auto   reference    = computeNewPosition(middle_point_circle_, initial_heading_, circle_angle);

    reference.header.stamp    = time;
    reference.header.frame_id = "NED_odom";

    // Fly the circle in position mode and attitude mode.
    reference.translation_mode = creos_sdk_msgs::msg::StateReference::TRANSLATION_MODE_POSITION;
    reference.orientation_mode = creos_sdk_msgs::msg::StateReference::ORIENTATION_MODE_ATTITUDE;

    RCLCPP_DEBUG(
        logger_,
        "StateReference -> Position: [%f, %f, %f] Heading: [%f] Middle point: [%f, %f, %f]",
        reference.pose.position.x, reference.pose.position.y, reference.pose.position.z,
        initial_heading_, middle_point_circle_[AxisIndex::kXAxis],
        middle_point_circle_[AxisIndex::kYAxis], middle_point_circle_[AxisIndex::kZAxis]);

    sine_counter_++;
    if(sine_counter_ >= totalSteps())
    {
        sine_counter_ = 0;
    }
    return reference;
}

const creos_sdk_msgs::msg::StateReference CircleReferences::computeNewPosition(
    const std::array<float, 3> &circle_middle,
    const double                initial_heading,
    const double                circle_angle) const
{
    creos_sdk_msgs::msg::StateReference reference;

    // Compute the position of the drone in the circle
    double cos_angle          = std::cos(circle_angle + initial_heading);
    double sin_angle          = std::sin(circle_angle + initial_heading);
    reference.pose.position.x = circle_middle[AxisIndex::kXAxis] + (-circle_radius_m_ * cos_angle);
    reference.pose.position.y = circle_middle[AxisIndex::kYAxis] + (-circle_radius_m_ * sin_angle);

    // Compute feedforward velocity
    reference.twist.linear.x = circle_radius_m_ * sin_angle * (2 * M_PI / circle_execution_time_s_);
    reference.twist.linear.y =
        -circle_radius_m_ * cos_angle * (2 * M_PI / circle_execution_time_s_);

    // Compute feedforward acceleration
    double pow_2_pi          = std::pow(2 * M_PI / circle_execution_time_s_, 2);
    reference.accel.linear.x = circle_radius_m_ * cos_angle * pow_2_pi;
    reference.accel.linear.y = circle_radius_m_ * sin_angle * pow_2_pi;

    // Keep height constant at the start height
    reference.pose.position.z = circle_middle[AxisIndex::kZAxis];
    reference.twist.linear.z  = 0.0f;
    reference.accel.linear.z  = 0.0f;

    // Keep heading constant
    tf2::Quaternion quat;
    quat.setRPY(0, 0, initial_heading);
    reference.pose.orientation.x = quat.x();
    reference.pose.orientation.y = quat.y();
    reference.pose.orientation.z = quat.z();
    reference.pose.orientation.w = quat.w();
    return reference;
}

const std::array<float, 3> CircleReferences::computeCircleMiddle(
    const std::array<float, 3> &begin_position,
    const double                initial_heading) const
{
    // Compute the middle point of the circle based on the heading of the drone.
    std::array<float, 3> middle_point;
    middle_point[AxisIndex::kXAxis] =
        begin_position[AxisIndex::kXAxis] + std::cos(initial_heading) * circle_radius_m_; // x
    middle_point[AxisIndex::kYAxis] =
        begin_position[AxisIndex::kYAxis] + std::sin(initial_heading) * circle_radius_m_; // y
    middle_point[AxisIndex::kZAxis] = begin_position[AxisIndex::kZAxis];                  // z
    return middle_point;
}
