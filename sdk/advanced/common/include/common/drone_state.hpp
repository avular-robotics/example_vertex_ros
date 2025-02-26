// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include "drone_state_interface.hpp"

#include <creos/messages/control_source.hpp>
#include <creos/messages/generic.hpp>
#include <creos/messages/pose.hpp>
#include <creos/messages/state.hpp>

#include <mutex>

class DroneState : public IDroneState
{
public:
    DroneState(bool log_state_change = false);
    ~DroneState() = default;

    const std::array<float, 3>           GetPosition();
    const creos_messages::Quaterniond   &GetOrientation();
    double                               GetYaw() override;
    const creos_messages::State         &GetState();
    const creos_messages::ControlSource &GetControlSource() override;

    bool IsArmed() override;
    bool IsDisarmed() override;
    bool IsPerformingPreFlightChecks() override;
    bool IsReadyForTakeOff() override;
    bool IsInTakeOff() override;
    bool IsInFlight() override;
    bool IsLanding() override;
    bool IsInUserControlMode() override;

    std::function<void(const creos_messages::PoseWithCovarianceStamped &)> GetGlobalPoseCallback();
    std::function<void(const creos_messages::ControlSource &)> GetControlSourceCallback();
    std::function<void(const creos_messages::State &)>         GetStateCallback();

private:
    const bool kLog_state_change_;

    std::mutex                                global_pose_mutex_;
    creos_messages::PoseWithCovarianceStamped latest_global_pose_;
    void globalPoseCallback(const creos_messages::PoseWithCovarianceStamped &msg);

    std::mutex                    control_source_mutex_;
    creos_messages::ControlSource latest_control_source_;
    void                          controlSourceCallback(const creos_messages::ControlSource &msg);

    std::mutex            state_mutex_;
    creos_messages::State latest_state_;
    void                  stateCallback(const creos_messages::State &msg);
};
