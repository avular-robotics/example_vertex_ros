// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include <creos/messages/battery_status.hpp>
#include <creos/system_info_interface.hpp>
#include <spdlog/spdlog.h>

/**
 * @brief Notifies the user of battery status changes
 * @param system_info_interface The system info interface
 * @details This function registers a callback that will be called whenever a battery status message
 * is received. If the battery status changes, it will print the new battery status to the console.
 */
void battery_notifier(creos::ISystemInfoInterface *system_info_interface)
{
    system_info_interface->subscribeToBatteryStatus(
        [](const creos_messages::BatteryStatus &battery_status)
        {
            static creos_messages::BatteryStatus previous_battery_status;
            static bool                          first_call = true;

            if(battery_status.state != previous_battery_status.state ||
               std::abs(battery_status.voltage - previous_battery_status.voltage) > 1 ||
               battery_status.state_of_charge != previous_battery_status.state_of_charge ||
               battery_status.state_of_health != previous_battery_status.state_of_health ||
               battery_status.temperature != previous_battery_status.temperature ||
               battery_status.alerts != previous_battery_status.alerts || first_call)
            {
                spdlog::info("Battery status:");
            }

            if(battery_status.state != previous_battery_status.state || first_call)
            {
                std::string state = "";
                switch(battery_status.state)
                {
                case creos_messages::BatteryStatus::State::kUnknown:
                    state = "Unknown";
                    break;
                case creos_messages::BatteryStatus::State::kOffline:
                    state = "Offline";
                    break;
                case creos_messages::BatteryStatus::State::kCharging:
                    state = "Charging";
                    break;
                case creos_messages::BatteryStatus::State::kDischarging:
                    state = "Discharging";
                    break;
                case creos_messages::BatteryStatus::State::kBalancing:
                    state = "Balancing";
                    break;
                case creos_messages::BatteryStatus::State::kError:
                    state = "Error";
                    break;
                default:
                    throw std::runtime_error("Unknown battery state");
                }
                spdlog::info("  - State: {}", state);
            }

            if(std::abs(battery_status.voltage - previous_battery_status.voltage) > 1 || first_call)
            {
                spdlog::info("  - Voltage: {} V", battery_status.voltage);
            }

            if(battery_status.state_of_charge != previous_battery_status.state_of_charge ||
               first_call)
            {
                spdlog::info("  - State of charge: {:.2f}%", battery_status.state_of_charge * 100);
            }

            if(battery_status.state_of_health != previous_battery_status.state_of_health ||
               first_call)
            {
                spdlog::info("  - State of health: {:.2f}%", battery_status.state_of_health * 100);
            }

            if(battery_status.temperature != previous_battery_status.temperature || first_call)
            {
                spdlog::info("  - Temperature: {} °C", battery_status.temperature);
            }

            if(battery_status.alerts != previous_battery_status.alerts || first_call)
            {
                spdlog::info("  - Alerts:");

                bool has_alerts = false;
                if(battery_status.alerts.cell_under_voltage)
                {
                    spdlog::info("    - Cell under voltage");
                    has_alerts = true;
                }
                if(battery_status.alerts.cell_over_voltage)
                {
                    spdlog::info("    - Cell over voltage");
                    has_alerts = true;
                }
                if(battery_status.alerts.over_current_charging)
                {
                    spdlog::info("    - Over current charging");
                    has_alerts = true;
                }
                if(battery_status.alerts.over_current_discharging)
                {
                    spdlog::info("    - Over current discharging");
                    has_alerts = true;
                }
                if(battery_status.alerts.overload_discharging)
                {
                    spdlog::info("    - Overload discharging");
                    has_alerts = true;
                }
                if(battery_status.alerts.latched_overload_discharging)
                {
                    spdlog::info("    - Latched overload discharging");
                    has_alerts = true;
                }
                if(battery_status.alerts.short_circuit)
                {
                    spdlog::info("    - Short circuit");
                    has_alerts = true;
                }
                if(battery_status.alerts.latched_short_circuit)
                {
                    spdlog::info("    - Latched short circuit");
                    has_alerts = true;
                }
                if(battery_status.alerts.over_temperature_charging)
                {
                    spdlog::info("    - Over temperature charging");
                    has_alerts = true;
                }
                if(battery_status.alerts.over_temperature_discharging)
                {
                    spdlog::info("    - Over temperature discharging");
                    has_alerts = true;
                }
                if(battery_status.alerts.under_temperature_charging)
                {
                    spdlog::info("    - Under temperature charging");
                    has_alerts = true;
                }
                if(battery_status.alerts.under_temperature_discharging)
                {
                    spdlog::info("    - Under temperature discharging");
                    has_alerts = true;
                }
                if(battery_status.alerts.afe_alert)
                {
                    spdlog::info("    - AFE alert");
                    has_alerts = true;
                }
                if(battery_status.alerts.precharge_timeout)
                {
                    spdlog::info("    - Precharge timeout");
                    has_alerts = true;
                }
                if(battery_status.alerts.overcharge)
                {
                    spdlog::info("    - Overcharge");
                    has_alerts = true;
                }
                if(!has_alerts)
                {
                    spdlog::info("    - No alerts active");
                }
            }
            previous_battery_status = battery_status;
            first_call              = false;
        });
}
