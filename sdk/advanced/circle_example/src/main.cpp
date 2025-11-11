// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Example of how to fly a circle with the drone.
 ****************************************************************************/
#include <CLI11.hpp>
#include <iostream>

#include <creos/client.hpp>
#include <creos/messages/controller_state.hpp>
#include <creos/messages/state_reference.hpp>
#include <creos/messages/odometry.hpp>

#include <common/logging.hpp>
#include <common/drone_state.hpp>
#include <common/remote_controller_interface.hpp>
#include <common/flight_controller.hpp>

#include "circle_references.hpp"

void spin(unsigned update_frequency_hz)
{
    usleep(1000000 / update_frequency_hz);
}

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
    CLI::App app{"Circle Example showcasing how to use the CreOS SDK to fly a circle."};

    double           circle_radius_m     = 2;
    double           speed_mps           = 0.5;
    unsigned         update_frequency_hz = 100;
    unsigned         take_off_delay_s    = 2;
    ControllerType   controller_type     = ControllerType::kHerelink;
    std::string_view host                = "";

    app.add_flag(
        "-v,--verbose", [](auto) { spdlog::set_level(spdlog::level::debug); },
        "Enable verbose output");
    app.add_option("--host", host,
                   "<hostname:port> of the CREOS agent. When omitted, the 'CREOS_HOST:CREOS_PORT' "
                   "environment variables are used or localhost:7200 is used as a fallback.");
    app.add_option("-r,--radius", circle_radius_m,
                   "Radius of the circle in meters. Default is 2 meters.");
    app.add_option("-s,--speed", speed_mps,
                   "Speed of the drone in meters per second. Default is 0.5 m/s.");
    app.add_option("-f,--frequency", update_frequency_hz,
                   "Update frequency in Hz. Default is 100 Hz.");
    app.add_option("-d,--delay", take_off_delay_s,
                   "Delay in seconds before the drone takes off when all take off conditions are "
                   "met. Default is 2 seconds.");
    app.add_flag_callback(
        "--jeti", [&controller_type]() { controller_type = ControllerType::kJeti; },
        "Use Jeti controller instead of Herelink controller");

    CLI11_PARSE(app, argc, argv);

    setup_logging("circle_example");

    // Connect to the CreOS server
    creos::Client client = createClient(host);

    // Setup DroneState
    std::shared_ptr<DroneState> drone_state = std::make_shared<DroneState>();
    client.sensors()->subscribeToOdometry(drone_state->GetOdometryCallback());
    client.setpoint_control()->subscribeToCurrentControlSource(
        drone_state->GetControlSourceCallback());
    client.diagnostics()->subscribeToState(drone_state->GetStateCallback());

    // Setup Controller input
    std::shared_ptr<IRemoteController> controller = CreateRemoteController(controller_type);
    client.sensors()->subscribeToRemoteController(controller->GetControllerStateCallback());

    // Setup FlightController
    FlightController flight_controller(*drone_state, take_off_delay_s);
    flight_controller.RegisterTakeOffTrigger(
        [&flight_controller, &client]()
        {
            try
            {
                client.setpoint_control()->sendCommand(
                    creos_messages::Command{.action = creos_messages::Command::Action::kTakeOff});
            }
            catch(const std::exception &e)
            {
                std::cerr << "Failed to send takeoff command: " << e.what() << std::endl;
            }
        });

    bool execution_active = false;
    controller->RegisterActivationButtonCallback(
        [&execution_active]()
        {
            execution_active = !execution_active;
            spdlog::info("Execution active: {}", execution_active ? "true" : "false");
        });

    CircleReferences circle_references =
        CircleReferences(update_frequency_hz, circle_radius_m, speed_mps);

    FlightState state = FlightState::kUnknown;
    while(true)
    {
        if(execution_active)
        {
            state = flight_controller.Run();
            if(state == FlightState::kFlying)
            {
                // Publish circle references when the drone is flying
                creos_messages::StateReference state_reference =
                    circle_references.GetNewStateReference();
                client.setpoint_control()->publishStateReference(state_reference);
            }
            else
            {
                // Reset the circle references when the drone is not flying
                circle_references.Reset(drone_state->GetPosition(), drone_state->GetYaw());
            }
        }
        // Reset the circle references when the exacution is not active since the drone could be
        // moving using the remote controller therefore the middle of circle should be updated
        else
        {
            circle_references.Reset(drone_state->GetPosition(), drone_state->GetYaw());
        }
        spin(update_frequency_hz);
    }
    return 0;
}
