// Copyright 2024 Avular B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Logs a safety message
 * @details This function logs a safety message to the console. This message is intended to be
 * displayed when the user is running the example for the first time.
 */
inline void log_safety_msg(rclcpp::Logger logger)
{
    RCLCPP_WARN(logger,
                "-------------------------------------------------------------------------------");
    RCLCPP_WARN(logger,
                "-------------------------------------------------------------------------------");
    RCLCPP_WARN(logger,
                "------  When testing for the first time, always remove the propellers!!  ------");
    RCLCPP_WARN(logger,
                "-------------------------------------------------------------------------------");
    RCLCPP_WARN(logger,
                "-------------------------------------------------------------------------------");
}

/**
 * @brief Sets up the logging for the example
 * @details This function logs the name of the example that is being run to the console.
 */
inline void setup_logging(std::string name, rclcpp::Logger logger)
{
    log_safety_msg(logger);
    RCLCPP_INFO(logger, "Running: %s", name.c_str());
}
