# Remote Controller Example

This example demonstrates how to subscribe to remote controller input and print the live stick and button state in the console.

## Structure

The code is split up into multiple parts:

- [RcUtils](src/rc_utils.hpp): Helper that formats controller axes and button states for console output.
- [Main](src/main.cpp): Main sets up all the classes and runs the main loop.

## RcUtils

This class is responsible for turning each `sensor_msgs::msg::Joy` message into a readable console display. Axes are printed with their numeric value and a small centered bar. Each button shows a rolling 20-cycle history: `*` is pressed and `.` is released.

## Main

The main function sets up the ROS node, subscribes to the controller topic, and registers the callback.

The main logic of the example can be found in the `handleRemoteControllerInput` function. This function is called every time a controller data message is received on `/robot/joy`, then logs the formatted input state.

## Usage

The example can be executed by running the following command:

```bash
ros2 run remote_controller_example remote_controller_example
```

The example can be stopped by pressing `Ctrl + C`.
