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
                                   double speed_mps)
    : update_frequency_hz_(update_frequency_hz),
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

    spdlog::debug("CircleReferences: Middle point: {}, {}, Heading: {}",
                  middle_point_circle_[AxisIndex::kXAxis], middle_point_circle_[AxisIndex::kYAxis],
                  initial_heading_);

    // Restart counter so you will begin from the start of the circle
    sine_counter_ = 0;
}

creos_messages::StateReference CircleReferences::GetNewStateReference()
{
    // Compute the current angle in the circle.
    double circle_angle = sine_counter_ * angleSteps();
    auto   reference    = computeNewPosition(middle_point_circle_, initial_heading_, circle_angle);

    reference.timestamp = creos::RobotClock::now();
    reference.frame_id  = "NED_odom";

    // Fly the circle in position mode and attitude mode.
    reference.translation_mode = creos_messages::StateReference::TranslationMode::kPosition;
    reference.orientation_mode = creos_messages::StateReference::OrientationMode::kAttitude;

    spdlog::debug(
        "StateReference -> Position: [{}, {}, {}] Heading: [{}] Middle point: [{}, {}, {}]",
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

const creos_messages::StateReference CircleReferences::computeNewPosition(
    const std::array<float, 3> &circle_middle,
    const double                initial_heading,
    const double                circle_angle) const
{
    creos_messages::StateReference reference;

    // Compute the position of the drone in the circle
    double cos_angle          = std::cos(circle_angle + initial_heading);
    double sin_angle          = std::sin(circle_angle + initial_heading);
    reference.pose.position.x = circle_middle[AxisIndex::kXAxis] + (-circle_radius_m_ * cos_angle);
    reference.pose.position.y = circle_middle[AxisIndex::kYAxis] + (-circle_radius_m_ * sin_angle);

    // Compute feedforward velocity
    reference.velocity.linear.x =
        circle_radius_m_ * sin_angle * (2 * M_PI / circle_execution_time_s_);
    reference.velocity.linear.y =
        -circle_radius_m_ * cos_angle * (2 * M_PI / circle_execution_time_s_);

    // Compute feedforward acceleration
    double pow_2_pi                 = std::pow(2 * M_PI / circle_execution_time_s_, 2);
    reference.acceleration.linear.x = circle_radius_m_ * cos_angle * pow_2_pi;
    reference.acceleration.linear.y = circle_radius_m_ * sin_angle * pow_2_pi;

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
