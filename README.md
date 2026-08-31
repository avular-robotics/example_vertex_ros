# Vertex SDK examples

This repository contains examples for the Avular Vertex One. The examples are divided into two categories: SDK examples and ROS2 examples. The SDK examples are written in C++ and use the Avular Software Development Kit to control the Vertex One. The ROS2 examples are written in C++ and use the Robot Operating System (ROS2) interface to control the Vertex One.

The main workspaces for these examples are:

- [sdk](sdk/README.md): Contains the SDK examples.
- [ros](ros/README.md): Contains the ROS2 examples.

Inside each workspace, you will find instructions on how to setup the workspace and execute the examples.

Both workspaces make use of a Docker container. This means that you can build and run the examples in a Docker container. This way, you don't have to worry about installing dependencies on your local machine.

Start by cloning this repository on the robot or your local machine:

```bash
git clone https://github.com/avular-robotics/example_vertex_ros
cd example_vertex_ros
```

Once you have cloned the repository and navigated into the project directory, you can follow the instructions in the respective workspace folders (`sdk` or `ros`) to build and run the examples.

## Examples

The behavior of each example is the same for both the SDK and ROS examples. Inside the workspace folder you will find a readme file that explains how to build and run the examples. In the following sections, we will give a brief overview of the examples.

### Getting started

The `getting_started` directory contains a few simple examples that show how to connect to the Vertex and get some basic information from it. This example is a good starting point to get familiar with the Avular API.
Inside each folder of the examples you will find a readme file that explains how the example works.

### Advanced

The `advanced` directory contains more complex examples that show how to control the Vertex. These examples are a good starting point to get familiar with controlling the Vertex using the Avular API. It is recommended to first learn how to control the Vertex using the remote controller before trying these examples. Always make sure that you are in a safe environment and that you know how to take over control of the Vertex using the remote controller.

> **Important:** Before activating any advanced example, make sure the drone is in the correct state:
> - Examples that perform **takeoff** (e.g. takeoff/land, waypoint) require the drone to be **stationary on the ground**.
> - Examples that control an **airborne** drone (e.g. circle) require the drone to be **hovering stably in the air** in **SDK mode**.
>
> Activating an example while the drone is flying at high speed in a different mode can lead to unexpected behaviour and may cause the drone to crash. Always make sure you are in a safe environment and that you know how to take over control of the Vertex using the remote controller.
