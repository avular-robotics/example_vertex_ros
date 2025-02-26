// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include <common/flight_controller.hpp>
#include <spdlog/spdlog.h>

FlightController::FlightController(IDroneState &drone_state, unsigned take_off_delay_s)
    : drone_state_(drone_state), take_off_delay_s_(take_off_delay_s)
{
    resetTakeOffDelay();
}

FlightController::~FlightController() {}

void FlightController::setState(FlightState state)
{
    if(state_ != state)
    {
        spdlog::info("State changed from {} to {}", stateToString(state_), stateToString(state));

        // Log user feedback upon state change
        switch(state)
        {
        case FlightState::kInActive:
            spdlog::info("Put the drone in user control mode to enable the user flight controller");
            break;
        case FlightState::kWaitingForArming:
            spdlog::info("Ready for take-off. Arm the drone with your remote!");
            break;
        case FlightState::kPerformingPreFlightChecks:
            spdlog::info("Performing pre-flight checks");
            break;
        case FlightState::kWaitingForTakeOff:
            break;
        case FlightState::kSendingTakeOff:
            spdlog::info("Triggering Take-off");
            break;
        case FlightState::kInTakeOff:
            spdlog::info("Take-off in progress");
            break;
        case FlightState::kFlying:
            [[fallthrough]];
        case FlightState::kLanding:
            [[fallthrough]];
        case FlightState::kUnknown:
            [[fallthrough]];
        default:
            break;
        }
    }
    state_ = state;
}

void FlightController::RegisterTakeOffTrigger(std::function<void()> trigger_take_off)
{
    trigger_take_off_ = trigger_take_off;
}

void FlightController::sendTakeOff()
{
    if(trigger_take_off_)
    {
        trigger_take_off_();
    }
    else
    {
        spdlog::error("No take-off trigger registered");
    }
}

std::string FlightController::stateToString(FlightState state)
{
    switch(state)
    {
    case FlightState::kInActive:
        return "InActive";
    case FlightState::kUnknown:
        return "Unknown";
    case FlightState::kWaitingForArming:
        return "WaitingForArming";
    case FlightState::kWaitingForTakeOff:
        return "WaitingForTakeOff";
    case FlightState::kSendingTakeOff:
        return "SendingTakeOff";
    case FlightState::kInTakeOff:
        return "InTakeOff";
    case FlightState::kFlying:
        return "Flying";
    case FlightState::kLanding:
        return "Landing";
    default:
        return "Unknown";
    }
}

void FlightController::resetTakeOffDelay()
{
    delay_start_ = std::chrono::system_clock::now();
}

bool FlightController::isTakeOffDelayExpired()
{
    static std::chrono::system_clock::time_point previous_log_time =
        std::chrono::system_clock::now() - std::chrono::milliseconds(200);

    auto now       = std::chrono::system_clock::now();
    auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(now - delay_start_);

    // Log the time remaining until take-off every 200 ms
    auto elapsed_since_last_log =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - previous_log_time);
    if(elapsed_since_last_log > std::chrono::milliseconds(200))
    {
        previous_log_time = now;
        spdlog::info("Drone will take off in {} ms", (take_off_delay_s_ - elapsed_s.count()));
    }

    return elapsed_s.count() >= take_off_delay_s_;
}

FlightState FlightController::Run()
{
    // If the execution is not activated or when it is not in user control mode, force deactivate
    // the execution and reset all counters + states.
    if(!drone_state_.IsInUserControlMode())
    {
        setState(FlightState::kInActive);
        resetTakeOffDelay();
    }
    else if(drone_state_.IsDisarmed())
    {
        setState(FlightState::kWaitingForArming);
        resetTakeOffDelay();
    }
    else if(drone_state_.IsArmed())
    {
        if(drone_state_.IsPerformingPreFlightChecks())
        {
            setState(FlightState::kPerformingPreFlightChecks);
        }
        else if(drone_state_.IsReadyForTakeOff())
        {
            if(!isTakeOffDelayExpired())
            {
                setState(FlightState::kWaitingForTakeOff);
            }
            else
            {
                setState(FlightState::kSendingTakeOff);
                sendTakeOff();
            }
        }
        else if(drone_state_.IsInTakeOff())
        {
            setState(FlightState::kInTakeOff);
        }
        else if(drone_state_.IsInFlight())
        {
            setState(FlightState::kFlying);
        }
        else if(drone_state_.IsLanding())
        {
            setState(FlightState::kLanding);
        }
    }
    return state_;
}
