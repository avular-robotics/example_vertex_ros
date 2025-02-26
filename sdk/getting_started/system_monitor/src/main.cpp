// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * This example demonstrates how to retrieve system information and monitor
 * the state of the robot.
 *
 * Upon running this example, the system information and battery status will
 * be logged once. The state of the robot will be printed initially and whenever it changes.
 ****************************************************************************/
#include <creos/client.hpp>
#include <creos/messages/state.hpp>
#include <iostream>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include "system_info.hpp"
#include "battery_notifier.hpp"
#include "state_notifier.hpp"
#include "control_source_notifier.hpp"

/**
 * @brief Sets up the logging for the example
 * @details This function sets up the logging for the example. It creates a rotating file sink
 * that will rotate the log file every 1 MB and keep the last 2 log files. It also creates a
 * console sink that will print the log messages to the console.
 */
void setup_logging()
{
    auto file_name = "system_monitor_example.log";
    auto rotating_sink =
        std::make_shared<spdlog::sinks::rotating_file_sink_mt>(file_name, 1024 * 1024, 2);
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto logger       = std::make_shared<spdlog::logger>(
        "system_monitor", spdlog::sinks_init_list{rotating_sink, console_sink});
    spdlog::set_default_logger(logger);
    spdlog::set_level(spdlog::level::info);
    spdlog::flush_every(std::chrono::seconds(1));
}

creos::Client createClient(const std::string_view host)
{
    if(host.empty())
    {
        return creos::Client();
    }
    return creos::Client(host);
}

int main(int argc, char *argv[])
{
    setup_logging();
    spdlog::info("System monitor example:");

    // Get the hostname from the environment variable or the command line argument
    std::string_view host = "";

    if(argc >= 2)
    {
        host = argv[1];
    }

    // Connect to the CreOS server
    creos::Client client = createClient(host);

    get_system_info(client.system_info());
    battery_notifier(client.system_info());
    state_notifier(client.diagnostics());
    control_source_notifier(client.setpoint_control());

    while(true)
    {
        sleep(1);
    }
    return 0;
}
