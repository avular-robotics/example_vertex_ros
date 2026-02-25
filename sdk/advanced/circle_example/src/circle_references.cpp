// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include "circle_references.hpp"

#include <spdlog/spdlog.h>
#include "eigen3/Eigen/Geometry"

CircleReferences::CircleReferences(double update_frequency_hz,
                                   double circle_radius_m,
                                   double speed_mps,
                                   double accel_mps2)
    : time_step_s_(1.0 / update_frequency_hz),
      circle_radius_m_(circle_radius_m),
      speed_mps_(speed_mps),
      accel_mps2_(accel_mps2)
{
    // Only support update frequencies above 10Hz.
    assert(update_frequency_hz >= 10);
}

void CircleReferences::Reset(const std::array<float, 3> position, const double yaw_heading)
{
    initial_heading_     = yaw_heading;
    middle_point_circle_ = computeCircleMiddle(position, initial_heading_);

    spdlog::debug("CircleReferences: Middle point: {}, {}, Heading: {}",
                  middle_point_circle_[AxisIndex::kXAxis], middle_point_circle_[AxisIndex::kYAxis],
                  initial_heading_);

    // Restart time so you will begin from the start of the circle
    time_s_ = 0.0;
}

creos_messages::StateReference CircleReferences::GetNewStateReference()
{
    // Compute the current angle in the circle.
    auto reference = computeNewPosition(middle_point_circle_, initial_heading_, time_s_);

    reference.timestamp = creos::RobotClock::now();
    reference.frame_id  = "map";

    // Fly the circle in position mode and attitude mode.
    reference.translation_mode = creos_messages::StateReference::TranslationMode::kPosition;
    reference.orientation_mode = creos_messages::StateReference::OrientationMode::kAttitude;

    spdlog::debug(
        "StateReference -> Position: [{}, {}, {}] Heading: [{}] Middle point: [{}, {}, {}]",
        reference.pose.position.x, reference.pose.position.y, reference.pose.position.z,
        initial_heading_, middle_point_circle_[AxisIndex::kXAxis],
        middle_point_circle_[AxisIndex::kYAxis], middle_point_circle_[AxisIndex::kZAxis]);

    // Update time for next iteration
    time_s_ += time_step_s_;

    return reference;
}

const creos_messages::StateReference CircleReferences::computeNewPosition(
    const std::array<float, 3> &circle_middle,
    const double                initial_heading,
    const double                time_s) const
{
    creos_messages::StateReference reference;

    // Compute the circle angle (and their time derivatives) based on time, speed, and acceleration.
    const double speed_accel_time_s = speed_mps_ / accel_mps2_;
    const double circle_angle       = -speed_mps_ / circle_radius_m_ *
                                (std::hypot(time_s, speed_accel_time_s) - speed_accel_time_s);
    const double d_angle_dt =
        (-speed_mps_ / circle_radius_m_) * (time_s / std::hypot(time_s, speed_accel_time_s));
    const double d2_angle_dt2 =
        (-speed_mps_ / circle_radius_m_) *
        (std::pow(speed_accel_time_s, 2) / std::pow(std::hypot(time_s, speed_accel_time_s), 3));

    // Compute the position of the drone in the circle
    double cos_angle          = std::cos(circle_angle + initial_heading);
    double sin_angle          = std::sin(circle_angle + initial_heading);
    reference.pose.position.x = circle_middle[AxisIndex::kXAxis] + (-circle_radius_m_ * cos_angle);
    reference.pose.position.y = circle_middle[AxisIndex::kYAxis] + (-circle_radius_m_ * sin_angle);

    // Compute feedforward velocity
    reference.velocity.linear.x = -circle_radius_m_ * -sin_angle * d_angle_dt;
    reference.velocity.linear.y = -circle_radius_m_ * cos_angle * d_angle_dt;

    // Compute feedforward acceleration
    reference.acceleration.linear.x = (-circle_radius_m_ * -cos_angle) * std::pow(d_angle_dt, 2) +
                                      (-circle_radius_m_ * -sin_angle) * d2_angle_dt2;
    reference.acceleration.linear.y = (-circle_radius_m_ * -sin_angle) * std::pow(d_angle_dt, 2) +
                                      (-circle_radius_m_ * cos_angle) * d2_angle_dt2;

    // Keep height constant at the start height
    reference.pose.position.z       = circle_middle[AxisIndex::kZAxis];
    reference.velocity.linear.z     = 0.0f;
    reference.acceleration.linear.z = 0.0f;

    // Keep heading constant
    Eigen::Quaterniond orientation(Eigen::AngleAxisd(initial_heading, Eigen::Vector3d::UnitZ()));
    reference.pose.orientation.x = orientation.x();
    reference.pose.orientation.y = orientation.y();
    reference.pose.orientation.z = orientation.z();
    reference.pose.orientation.w = orientation.w();
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
