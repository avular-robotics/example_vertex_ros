// Copyright (C) 2024 Avular B.V. - All Rights Reserved
// You may use this code under the terms of the Avular
// Software End-User License Agreement.
//
// You should have received a copy of the Avular
// Software End-User License Agreement license with
// this file, or download it from: avular.com/eula
//
/*****************************************************************************
 * Simple hello world example for the CreOS client library.
 *
 * This example demonstrates how to connect to the CreOS server and subscribe to
 * battery status messages.
 ****************************************************************************/
#include <creos/client.hpp>
#include <iostream>

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
    // Get the hostname from the environment variable or the command line argument
    std::string_view host = "";

    if(argc >= 2)
    {
        host = argv[1];
    }

    // Connect to the CreOS server
    creos::Client client = createClient(host);

    // Register a callback function for the battery messages
    client.system_info()->subscribeToBatteryStatus(
        [](creos_messages::BatteryStatus message) {
            std::cout << "Battery state of charge: " << message.state_of_charge * 100 << "%"
                      << std::endl;
        });

    while(true)
    {
        sleep(1);
    }
    return 0;
}
