// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#pragma once

#include <common/drone_state_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <chrono>

enum class FlightState
{
    kInActive,
    kUnknown,
    kWaitingForArming,
    kPerformingPreFlightChecks,
    kWaitingForTakeOff,
    kSendingTakeOff,
    kInTakeOff,
    kFlying,
    kLanding,
};

class FlightController
{
public:
    FlightController(IDroneState &drone_state, unsigned take_off_delay_s, rclcpp::Logger logger);
    ~FlightController();

    FlightState Run();
    void        RegisterTakeOffTrigger(std::function<void()> trigger_take_off);

private:
    FlightState  state_ = FlightState::kUnknown;
    IDroneState &drone_state_;

    rclcpp::Logger logger_;

    unsigned                              take_off_delay_s_;
    std::chrono::system_clock::time_point delay_start_;

    void        setState(FlightState state);
    std::string stateToString(FlightState state);

    void                  sendTakeOff();
    std::function<void()> trigger_take_off_;
    void                  resetTakeOffDelay();
    bool                  isTakeOffDelayExpired();
};
