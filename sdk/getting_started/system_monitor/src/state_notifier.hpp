// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include <creos/messages/state.hpp>
#include <creos/diagnostics_interface.hpp>
#include <spdlog/spdlog.h>

/**
 * @brief Notifies the user of robot state changes
 * @param diagnostics_interface The diagnostics interface
 * @details This function registers a callback that will be called whenever a state message
 * is received. If the state changes, it will print the new state to the console.
 */
void state_notifier(creos::IDiagnosticsInterface *diagnostics_interface)
{
    diagnostics_interface->subscribeToState(
        [](const creos_messages::State &state)
        {
            static creos_messages::State previous_state;

            if(state.current_action != previous_state.current_action ||
               state.ready_state != previous_state.ready_state)
            {
                std::string ready_state = "";
                switch(state.ready_state)
                {
                case creos_messages::State::Ready::kUnknown:
                    ready_state = "Unknown";
                    break;
                case creos_messages::State::Ready::kNotReady:
                    ready_state = "NotReady";
                    break;
                case creos_messages::State::Ready::kPreArm:
                    ready_state = "PreArm";
                    break;
                case creos_messages::State::Ready::kActive:
                    ready_state = "Active";
                    break;
                case creos_messages::State::Ready::kWaiting:
                    ready_state = "Waiting";
                    break;
                case creos_messages::State::Ready::kFailsafe:
                    ready_state = "Failsafe";
                    break;
                case creos_messages::State::Ready::kError:
                    ready_state = "Error";
                    break;
                default:
                    throw std::runtime_error("Unknown ready state");
                }

                spdlog::info("State: {} - {}", ready_state, state.current_action);
            }
            previous_state = state;
        });
}
