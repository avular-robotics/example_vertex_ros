// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include <creos/messages/control_source.hpp>
#include <creos/setpoint_control_interface.hpp>
#include <spdlog/spdlog.h>

/**
 * @brief Notifies the user of control source changes
 * @param setpoint_control The setpoint control interface
 * @details This function registers a callback that will be called whenever a control source message
 * is received. If the control source changes, it will print the new control source to the console.
 */
void control_source_notifier(creos::ISetpointControlInterface *setpoint_control)
{
    setpoint_control->subscribeToCurrentControlSource(
        [](const creos_messages::ControlSource &control_source)
        {
            static creos_messages::ControlSource previous_control_source =
                static_cast<creos_messages::ControlSource>(-1);

            if(control_source != previous_control_source)
            {
                std::string control_source_str = "";
                switch(control_source)
                {
                case creos_messages::ControlSource::kDisabled:
                    control_source_str = "Disabled";
                    break;
                case creos_messages::ControlSource::kManual:
                    control_source_str = "Manual";
                    break;
                case creos_messages::ControlSource::kAutonomous:
                    control_source_str = "Autonomous";
                    break;
                case creos_messages::ControlSource::kUser:
                    control_source_str = "User";
                    break;
                default:
                    throw std::runtime_error("Unknown control source");
                }

                spdlog::info("Control source: {}", control_source_str);
            }
            previous_control_source = control_source;
        });
}
