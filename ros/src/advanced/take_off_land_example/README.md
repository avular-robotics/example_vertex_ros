# Take off and land example

The example showcases how to perform a takeoff and land. Pressing the activation button will take off the drone instantly, provided it is armed and in the correct state, and will land as soon as the drone has completed the takeoff.

> **Pre-condition:** The drone must be stationary on the ground before starting this example.

## Structure

The code is split up into multiple parts:

- [RemoteController](../common/include/common/remote_controller_interface.hpp): Interprets the raw controller data and triggers an action callback.
- [DroneState](../common/include/common/drone_state_interface.hpp): Collects the state and position of the drone.
- [FlightController](../common/include/common/flight_controller.hpp): Basic flight controller that can trigger a takeoff when the drone is in the correct state.
- [Main](src/main.cpp): Main sets up all the classes and runs the main loop.

## Main

The main function sets up all the classes and runs the main loop. It will take care of the ROS parameters and will subscribe to the correct topics and register the correct callbacks.

The D button (SF switch when using Jeti Controller) triggers an instant take-off request. This request is only evaluated at the exact moment the button is pressed: if the drone is `armed` and in **SDK mode** (referred to as `user` control mode in the API) and ready for take-off, the takeoff command is sent immediately. Otherwise, the request is ignored and an error is logged, and it will **not** take off later on its own (e.g. after the drone gets armed), the button must be pressed again once the drone is ready. Once the drone has completed the takeoff, the drone will land automatically. This logic can all be found inside the `main` method.

> **Note:** The takeoff height is determined by the `take_off_height` parameter configured on the drone. See the [flight controller parameters](https://avular-robotics.github.io/vertex_one_user_documentation/latest/flight_controller/parameters.html) for the default value and how to change it.

## Usage

The example can be executed by running the following command:

```bash
source install/setup.bash
ros2 run take_off_land_example take_off_land_example
```

The following ROS parameters can be used:

- `frequency`: Update frequency in Hz. Default is 100 Hz.
- `controller`: Controller type to use: `herelink` or `jeti`. Default is `herelink`.

Example:

```bash
ros2 run take_off_land_example take_off_land_example --ros-args -p controller:=jeti
```
