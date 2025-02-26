# Cirlce example

This example will make the Vertex fly in a circle. The example showcases how to perform a takeoff and fly in a circle. It wil use it's current positon to start flying in circles. The radius and the execution time can be changed using CLI arguments.

## Structure

The code is split up into multiple parts:

- [RemoteController](../common/include/common/remote_controller_interface.hpp): Interprets the raw controller data and triggers a action callback.
- [DroneState](../common/include/common/drone_state_interface.hpp): Collects the state and positon of the drone.
- [FlightController](../common/include/common/flight_controller.hpp): Basic flight controller that can automatically takeoff when the drone is in the correct state.
- [CircleReferences](src/circle_references.hpp): Class that calculates the setpoints for the drone to fly in a circle.
- [Main](src/main.cpp): Main sets up all the classes and runs the main loop.

## CircleReferences

This class is responsible for calculating the setpoints for the drone to fly in a circle. It uses the current position of the drone to calculate the setpoints. The radius and the execution time can be changed in the code.
Based on the spin rate of the example, the setpoints are calculated and a new setpoint is returned upon every call to `GetNewStateReference`. The drone will only change position in the horizontal plane. The altitude and heading will remain the same.

## Main

The main function sets up all the classes and runs the main loop. It will take care of the CLI arguments and will subscribe to the correct API's and register the correct callbacks.

The execution of the example can be toggled using the C button (SF switch when using Jeti Controller). Once the execution is active, the FlightController `Run` function will be called. This function will check if the drone is in the correct state and automatically takeoff. This will only happen if the drone is `armed` and is in `user` control mode. The takeoff will happen after x seconds, where x is the value of the `takeoff_delay` variable. Once the drone is in the air, the `CircleReferences` class will be called to calculate the setpoints for the drone to fly in a circle.

If at any point the execution is toggled off, the drone will stay still in the air. If at any point you want to take control of the drone, you can do so by switching the drone into `manual` mode. The drone will then stop listening to the setpoints and will only listen to the controller input.
