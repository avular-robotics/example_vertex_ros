// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
#pragma once

#include <creos_sdk_msgs/msg/battery_status.hpp>

/**
 * @brief Notifies the user of battery status changes
 * @param battery_status The new battery status
 * @param node The node that is used to log the battery status
 * @details This function logs the battery status to the console if the status has changed since the
 * last call. The battery status is only logged if the state, voltage, state of charge, state of
 * health, temperature or alerts have changed.
 */
void battery_notifier(const creos_sdk_msgs::msg::BatteryStatus::SharedPtr battery_status,
                      rclcpp::Node::SharedPtr                             node)
{
    static std::shared_ptr<creos_sdk_msgs::msg::BatteryStatus> previous_battery_status =
        std::make_shared<creos_sdk_msgs::msg::BatteryStatus>();
    static bool first_call = true;

    if(battery_status->state != previous_battery_status->state ||
       std::abs(battery_status->voltage - previous_battery_status->voltage) > 1 ||
       battery_status->state_of_charge != previous_battery_status->state_of_charge ||
       battery_status->state_of_health != previous_battery_status->state_of_health ||
       battery_status->temperature != previous_battery_status->temperature ||
       battery_status->alerts != previous_battery_status->alerts || first_call)
    {
        RCLCPP_INFO(node->get_logger(), "Battery status:");
    }

    if(battery_status->state != previous_battery_status->state || first_call)
    {
        std::string state = "";
        switch(battery_status->state)
        {
        case creos_sdk_msgs::msg::BatteryStatus::STATE_UNKNOWN:
            state = "Unknown";
            break;
        case creos_sdk_msgs::msg::BatteryStatus::STATE_OFFLINE:
            state = "Offline";
            break;
        case creos_sdk_msgs::msg::BatteryStatus::STATE_CHARGING:
            state = "Charging";
            break;
        case creos_sdk_msgs::msg::BatteryStatus::STATE_DISCHARGING:
            state = "Discharging";
            break;
        case creos_sdk_msgs::msg::BatteryStatus::STATE_BALANCING:
            state = "Balancing";
            break;
        case creos_sdk_msgs::msg::BatteryStatus::STATE_ERROR:
            state = "Error";
            break;
        default:
            throw std::runtime_error("Unknown battery state");
        }
        RCLCPP_INFO(node->get_logger(), "  - State: %s", state.c_str());
    }

    if(std::abs(battery_status->voltage - previous_battery_status->voltage) > 1 || first_call)
    {
        RCLCPP_INFO(node->get_logger(), "  - Voltage: %f V", battery_status->voltage);
    }

    if(battery_status->state_of_charge != previous_battery_status->state_of_charge || first_call)
    {
        RCLCPP_INFO(node->get_logger(), "  - State of charge: %.2f%%",
                    battery_status->state_of_charge * 100);
    }

    if(battery_status->state_of_health != previous_battery_status->state_of_health || first_call)
    {
        RCLCPP_INFO(node->get_logger(), "  - State of health: %.2f%%",
                    battery_status->state_of_health * 100);
    }

    if(battery_status->temperature != previous_battery_status->temperature || first_call)
    {
        RCLCPP_INFO(node->get_logger(), "  - Temperature: %.2f °C", battery_status->temperature);
    }

    if(battery_status->alerts != previous_battery_status->alerts || first_call)
    {
        RCLCPP_INFO(node->get_logger(), "  - Alerts:");

        bool has_alerts = false;
        if(battery_status->alerts & creos_sdk_msgs::msg::BatteryStatus::ALERT_CELL_UNDERVOLTAGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Cell under voltage");
            has_alerts = true;
        }
        if(battery_status->alerts & creos_sdk_msgs::msg::BatteryStatus::ALERT_CELL_OVERVOLTAGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Cell over voltage");
            has_alerts = true;
        }
        if(battery_status->alerts & creos_sdk_msgs::msg::BatteryStatus::ALERT_OVERCURRENT_CHARGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Over current charging");
            has_alerts = true;
        }
        if(battery_status->alerts & creos_sdk_msgs::msg::BatteryStatus::ALERT_OVERCURRENT_DISCHARGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Over current discharging");
            has_alerts = true;
        }
        if(battery_status->alerts & creos_sdk_msgs::msg::BatteryStatus::ALERT_OVERLOAD_DISCHARGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Overload discharging");
            has_alerts = true;
        }
        if(battery_status->alerts &
           creos_sdk_msgs::msg::BatteryStatus::ALERT_OVERLOAD_DISCHARGE_LATCH)
        {
            RCLCPP_INFO(node->get_logger(), "    - Latched overload discharging");
            has_alerts = true;
        }
        if(battery_status->alerts &
           creos_sdk_msgs::msg::BatteryStatus::ALERT_SHORT_CIRCUIT_DISCHARGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Short circuit");
            has_alerts = true;
        }
        if(battery_status->alerts &
           creos_sdk_msgs::msg::BatteryStatus::ALERT_SHORT_CIRCUIT_DISCHARGE_LATCH)
        {
            RCLCPP_INFO(node->get_logger(), "    - Latched short circuit");
            has_alerts = true;
        }
        if(battery_status->alerts &
           creos_sdk_msgs::msg::BatteryStatus::ALERT_OVERTEMPERATURE_CHARGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Over temperature charging");
            has_alerts = true;
        }
        if(battery_status->alerts &
           creos_sdk_msgs::msg::BatteryStatus::ALERT_OVERTEMPERATURE_DISCHARGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Over temperature discharging");
            has_alerts = true;
        }
        if(battery_status->alerts &
           creos_sdk_msgs::msg::BatteryStatus::ALERT_UNDERTEMPERATURE_CHARGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Under temperature charging");
            has_alerts = true;
        }
        if(battery_status->alerts &
           creos_sdk_msgs::msg::BatteryStatus::ALERT_UNDERTEMPERATURE_DISCHARGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Under temperature discharging");
            has_alerts = true;
        }
        if(battery_status->alerts & creos_sdk_msgs::msg::BatteryStatus::ALERT_AFE_ALERT)
        {
            RCLCPP_INFO(node->get_logger(), "    - AFE alert");
            has_alerts = true;
        }
        if(battery_status->alerts &
           creos_sdk_msgs::msg::BatteryStatus::ALERT_PRECHARGE_TIMEOUT_SUSPEND)
        {
            RCLCPP_INFO(node->get_logger(), "    - Precharge timeout");
            has_alerts = true;
        }
        if(battery_status->alerts & creos_sdk_msgs::msg::BatteryStatus::ALERT_OVERCHARGE)
        {
            RCLCPP_INFO(node->get_logger(), "    - Overcharge");
            has_alerts = true;
        }
        if(!has_alerts)
        {
            RCLCPP_INFO(node->get_logger(), "    - No alerts active");
        }
    }
    previous_battery_status = battery_status;
    first_call              = false;
}
