// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//

#include <common/flight_controller.hpp>

FlightController::FlightController(IDroneState &drone_state, rclcpp::Logger logger)
    : drone_state_(drone_state), logger_(logger)
{
}

FlightController::~FlightController() {}

void FlightController::setState(FlightState state)
{
    if(state_ != state)
    {
        RCLCPP_INFO(logger_, "State changed from %s to %s", stateToString(state_).c_str(),
                    stateToString(state).c_str());

        // Log user feedback upon state change
        switch(state)
        {
        case FlightState::kInActive:
            RCLCPP_INFO(logger_,
                        "Put the drone in user control mode to enable the user flight controller");
            break;
        case FlightState::kWaitingForArming:
            RCLCPP_INFO(logger_, "Ready for take-off. Arm the drone with your remote!");
            break;
        case FlightState::kPerformingPreFlightChecks:
            RCLCPP_INFO(logger_, "Performing pre-flight checks");
            break;
        case FlightState::kReadyForTakeOff:
            RCLCPP_INFO(logger_, "Ready for take-off. Press the activation button to take off!");
            break;
        case FlightState::kSendingTakeOff:
            RCLCPP_INFO(logger_, "Triggering Take-off");
            break;
        case FlightState::kInTakeOff:
            RCLCPP_INFO(logger_, "Take-off in progress");
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
        RCLCPP_ERROR(logger_, "No take-off trigger registered");
    }
}

void FlightController::RequestTakeOff()
{
    if(state_ != FlightState::kReadyForTakeOff)
    {
        RCLCPP_ERROR(logger_,
                     "Take-off request ignored: drone is not ready for take-off "
                     "(current state: %s)",
                     stateToString(state_).c_str());
        return;
    }

    setState(FlightState::kSendingTakeOff);
    sendTakeOff();
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
    case FlightState::kReadyForTakeOff:
        return "ReadyForTakeOff";
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

FlightState FlightController::UpdateStatus()
{
    // Continuously track the drone state so that RequestTakeOff() can rely on it always being
    // up-to-date, regardless of whether the activation button has been pressed yet.
    if(!drone_state_.IsInUserControlMode())
    {
        setState(FlightState::kInActive);
    }
    else if(drone_state_.IsDisarmed())
    {
        setState(FlightState::kWaitingForArming);
    }
    else if(drone_state_.IsArmed())
    {
        if(drone_state_.IsPerformingPreFlightChecks())
        {
            setState(FlightState::kPerformingPreFlightChecks);
        }
        else if(drone_state_.IsReadyForTakeOff())
        {
            setState(FlightState::kReadyForTakeOff);
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
