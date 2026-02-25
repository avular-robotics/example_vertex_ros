// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#pragma once

#include <creos/messages/state_reference.hpp>
#include <creos/messages/generic.hpp>
#include <array>
#include <optional>


class RemoteControllerReferences
{
public:
    enum AxisIndex
    {
        kXAxis = 0,
        kYAxis = 1,
        kZAxis = 2
    };

    RemoteControllerReferences(double max_velocity_horizontal_m = 3,
                               double max_velocity_vertical_m   = 1,
                               double max_yaw_rate_deg          = 30);
    ~RemoteControllerReferences() = default;

    const creos_messages::StateReference CreateReference(
        const std::array<float, 2>         &left_stick,
        const std::array<float, 2>         &right_stick,
        const creos_messages::Quaterniond  &latest_attitude,
        const creos::RobotClock::time_point timestamp,
        const std::string                   frame_id = "base_link");

private:
    const double kMax_velocity_horizontal_m_;
    const double kMax_velocity_vertical_m_;
    const double kMax_yaw_rate_deg_;;
};
