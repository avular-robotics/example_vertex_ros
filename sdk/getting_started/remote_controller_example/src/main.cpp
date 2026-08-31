// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Example of how to print remote controller input from the Jetson
 ****************************************************************************/
#include <CLI11.hpp>

#include <creos/client.hpp>
#include <creos/messages/controller_state.hpp>

#include <common/logging.hpp>

#include "rc_utils.hpp"
#include "remote_controller_example.hpp"

#include <chrono>
#include <thread>

creos::Client createClient(const std::string_view host)
{
    if(host.empty())
    {
        return creos::Client();
    }
    return creos::Client(host);
}

int main(int argc, char **argv)
{
    CLI::App app{"Remote Controller example showcasing how the remote controller input can be used "
                 "by showing live controller inputs in the console. "};

    std::string host = "";

    app.add_flag(
        "-v,--verbose", [](auto) { spdlog::set_level(spdlog::level::debug); },
        "Enable verbose output");
    app.add_option("--host", host,
                   "<hostname:port> of the CREOS agent. When omitted, the 'CREOS_HOST:CREOS_PORT' "
                   "environment variables are used or localhost:7200 is used as a fallback.");
    CLI11_PARSE(app, argc, argv);

    setup_logging("remote_controller_example");

    // Connect to the CreOS server
    creos::Client client = createClient(host);

    RcUtils rc_utils;

    // Setup Controller input
    client.sensors()->subscribeToRemoteController(
        [&rc_utils](const creos_messages::ControllerState &state)
        { handleRemoteControllerInput(state, rc_utils); });

    while(true)
    {
        sleep(1);
    }
    return 0;
}
