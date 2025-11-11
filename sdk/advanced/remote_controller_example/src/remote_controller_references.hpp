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
                               double max_yaw_rate_deg          = 30,
                               bool   yaw_rate_mode             = false);
    ~RemoteControllerReferences() = default;

    const creos_messages::StateReference CreateReference(
        const std::array<float, 2>         &left_stick,
        const std::array<float, 2>         &right_stick,
        const float                         latest_heading,
        const float                         time_delta_s,
        const creos::RobotClock::time_point timestamp,
        const std::string                   frame_id = "odom");

    /**
     * @brief Reset the RemoteControllerReferences
     * @details This resets the heading reference so that the next time a reference is created, the
     * heading reference is updated with the latest heading.
     */
    void Reset();

    void SetYawRateMode(bool yaw_rate_mode);
    bool GetYawRateMode() const;

private:
    const double kMax_velocity_horizontal_m_;
    const double kMax_velocity_vertical_m_;
    const double kMax_yaw_rate_deg_;
    bool         yaw_rate_mode_;

    std::optional<float> heading_ref_ = std::nullopt;
};
