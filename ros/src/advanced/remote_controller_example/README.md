# Remote Controller Example

This example demonstrates how to use the remote controller input to control the Vertex drone via state references. The drone will respond to the remote controller input and move accordingly. The maximum velocities and yaw rate can be changed using CLI arguments.

## Structure

The code is split up into multiple parts:

- [DroneState](../common/include/common/drone_state_interface.hpp): Collects the state and position of the drone.
- [RemoteControllerReferences](src/remote_controller_references.hpp): Class that calculates the setpoints for the drone based on remote controller input.
- [Main](src/main.cpp): Main sets up all the classes and runs the main loop.

## RemoteControllerReferences

This class is responsible for calculating the setpoints using the controller input. Initially it uses the current state of the drone to calculate the setpoints. The maximum velocities and yaw rate can be changed using CLI arguments. The setpoints are calculated and a new setpoint is returned upon every call to `CreateReference`. The drone's movement is controlled in the horizontal and vertical planes, as well as its yaw.

## Main

The main function sets up all the classes and runs the main loop. It handles the CLI arguments, subscribes to the correct APIs, and registers the appropriate callbacks.

The main logic of the example can be found in the `handleRemoteControllerInput` function. This function is called every time a controller data message is received by the SDK client. The function keeps track of the current and last control mode. By switching the drone into `user` control mode, the drone will start listening to the setpoints calculated by the `RemoteControllerReferences` class.

If at any point you want to control the drone without the SDK, you can switch the control mode to `manual` and the drone will stop listening to the setpoints sent by the example and the will be controlled using the regular RC input.
