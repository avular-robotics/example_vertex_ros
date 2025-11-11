# Take off and land example

The example showcases how to perform a takeoff and land. It will take off after the drone has been armed and is in the correct state. The drone will take of after a specified delay and will land as soon as the drone has completed the takeoff.

## Structure

The code is split up into multiple parts:

- [RemoteController](../common/include/common/remote_controller_interface.hpp): Interprets the raw controller data and triggers a action callback.
- [DroneState](../common/include/common/drone_state_interface.hpp): Collects the state and positon of the drone.
- [FlightController](../common/include/common/flight_controller.hpp): Basic flight controller that can automatically takeoff when the drone is in the correct state.
- [Main](src/main.cpp): Main sets up all the classes and runs the main loop.

## Main

The main function sets up all the classes and runs the main loop. It will take care of the CLI arguments and will subscribe to the correct API's and register the correct callbacks.

The execution of the example can be toggled using the D button (SF switch when using Jeti Controller). Once the execution is active, the FlightController `Run` function will be called. This function will check if the drone is in the correct state and automatically takeoff. This will only happen if the drone is `armed` and is in `user` control mode. The takeoff will happen after x seconds, where x is the value of the `takeoff_delay` variable. Once the drone has completed the takeoff, the drone will land. This logic can all be found inside the `main` method.

## Usage

The example can be executed by running the following command:

```bash
./build/bin/take_off_land_example
```

The following CLI arguments can be used:

- `--verbose`: Enable verbose output.
- `--frequency`: Update frequency in Hz. Default is 100 Hz.
- `--delay`: Delay in seconds before the drone takes off when all take off conditions are met. Default is 2 seconds.
- `--jeti`: Use Jeti controller instead of Herelink controller.

Example:

```bash
./build/bin/take_off_land_example --delay 5 --jeti
```
