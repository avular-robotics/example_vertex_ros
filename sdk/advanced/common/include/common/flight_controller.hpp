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

#include <spdlog/spdlog.h>
#include <chrono>

enum class FlightState
{
    kInActive,
    kUnknown,
    kWaitingForArming,
    kPerformingPreFlightChecks,
    kReadyForTakeOff,
    kSendingTakeOff,
    kInTakeOff,
    kFlying,
    kLanding,
};

class FlightController
{
public:
    explicit FlightController(IDroneState &drone_state);
    ~FlightController();

    FlightState UpdateStatus();
    void        RegisterTakeOffTrigger(std::function<void()> trigger_take_off);

    // Edge-triggered: if the drone is not ready at that exact moment the request is ignored so it
    // can never fire automatically at a later time.
    void RequestTakeOff();

private:
    FlightState  state_ = FlightState::kUnknown;
    IDroneState &drone_state_;

    void        setState(FlightState state);
    std::string stateToString(FlightState state);

    void                  sendTakeOff();
    std::function<void()> trigger_take_off_;
};
