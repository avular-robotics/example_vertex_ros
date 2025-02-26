// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Example of how to use the remote controller with the drone via the Jetson
 ****************************************************************************/
#include <CLI11.hpp>
#include <iostream>

#include <creos/client.hpp>
#include <creos/messages/controller_state.hpp>
#include <creos/messages/state_reference.hpp>

#include <common/logging.hpp>
#include <common/drone_state.hpp>

#include "remote_controller_example.hpp"
#include "remote_controller_references.hpp"

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
                 "to control the drone using statereferences using the SDK."};

    double      max_velocity_horizontal_m = 3;
    double      max_velocity_vertical_m   = 1;
    double      max_yaw_rate_deg          = 30;
    bool        yaw_rate_mode             = false;
    std::string host                      = "";

    app.add_flag(
        "-v,--verbose", [](auto) { spdlog::set_level(spdlog::level::debug); },
        "Enable verbose output");
    app.add_option("--host", host,
                   "<hostname:port> of the CREOS agent. When omitted, the 'CREOS_HOST:CREOS_PORT' "
                   "environment variables are used or localhost:7200 is used as a fallback.");
    app.add_option("--vmax_xy", max_velocity_horizontal_m,
                   "Maximum velocity in the horizontal plane in m/s. Default is 5 m/s.");
    app.add_option("--vmax_z", max_velocity_vertical_m,
                   "Maximum velocity in the vertical plane in m/s. Default is 1 m/s.");
    app.add_option("--wmax_deg", max_yaw_rate_deg,
                   "Maximum yaw rate in degrees per second. Default is 10 deg/s.");
    app.add_flag("--yaw_rate_mode", yaw_rate_mode, "Enable yaw rate mode. Default is false.");
    CLI11_PARSE(app, argc, argv);

    setup_logging("remote_controller_example");

    // Connect to the CreOS server
    creos::Client client = createClient(host);

    // Setup DroneState
    std::shared_ptr<DroneState> drone_state = std::make_shared<DroneState>(true);
    client.sensors()->subscribeToPose(drone_state->GetGlobalPoseCallback());
    client.setpoint_control()->subscribeToCurrentControlSource(
        drone_state->GetControlSourceCallback());
    client.diagnostics()->subscribeToState(drone_state->GetStateCallback());

    // Remote Controller References
    RemoteControllerReferences remote_controller_references(
        max_velocity_horizontal_m, max_velocity_vertical_m, max_yaw_rate_deg, yaw_rate_mode);

    // Setup Controller input
    client.sensors()->subscribeToRemoteController(
        [&remote_controller_references, &drone_state,
         &client](const creos_messages::ControllerState &state)
        {
            handleRemoteControllerInput(state, remote_controller_references, drone_state,
                                        client.setpoint_control());
        });

    while(true)
    {
        sleep(1);
    }
    return 0;
}
