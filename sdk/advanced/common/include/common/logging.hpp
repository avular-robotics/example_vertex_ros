// Copyright 2024 Avular B.V.
// All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula

#pragma once

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

/**
 * @brief Logs a safety message
 * @details This function logs a safety message to the console. This message is intended to be
 * displayed when the user is running the example for the first time.
 */
inline void log_safety_msg()
{
    spdlog::warn("-------------------------------------------------------------------------------");
    spdlog::warn("-------------------------------------------------------------------------------");
    spdlog::warn("------  When testing for the first time, always remove the propellers!!  ------");
    spdlog::warn("-------------------------------------------------------------------------------");
    spdlog::warn("-------------------------------------------------------------------------------");
}

/**
 * @brief Sets up the logging for the example
 * @details This function sets up the logging for the example. It creates a rotating file sink
 * that will rotate the log file every 1 MB and keep the last 2 log files. It also creates a
 * console sink that will print the log messages to the console.
 */
inline void setup_logging(std::string name)
{
    auto file_name = name + ".log";
    auto rotating_sink =
        std::make_shared<spdlog::sinks::rotating_file_sink_mt>(file_name, 1024 * 1024, 2);
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto logger       = std::make_shared<spdlog::logger>(
        name, spdlog::sinks_init_list{rotating_sink, console_sink});
    spdlog::set_default_logger(logger);
    spdlog::set_level(spdlog::level::info);
    spdlog::flush_every(std::chrono::seconds(1));

    log_safety_msg();

    spdlog::info("Logging to file: {}", file_name);
    spdlog::info("Running: {}", name);
}
