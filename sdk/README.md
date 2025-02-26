# SDK Examples

This directory contains examples that demonstrate how to use the SDK to interact with the Vertex robot.
The instructions below will guide you through running the SDK examples and making changes to the examples.

Currently, the following examples are available:

- Getting started
  - [Hello World](getting_started/hello_world/README.md)
  - [System Monitor](getting_started/system_monitor/README.md)
- Advanced
  - [Takeoff and Land Example](advanced/take_off_land_example/README.md)
  - [Circle Example](advanced/circle_example/README.md)
  - [Remote Controller Example](advanced/remote_controller_example/README.md)

**Environments:**

The SDK example can be build and run from inside a Docker container on the robot. The Docker container is already setup with the necessary tools to build and run the SDK examples.

There are two ways to run/setup the docker container:

1. [Using SSH terminal connection with the robot](#using-ssh-terminal-connection-with-the-robot) (quickest)
2. [Using a "VSCode Remote SSH: Dev Container"](#using-a-vscode-remote-ssh-dev-container) (recommended for development)

The instructions below assume that you already know how to connect to the robot using a SSH connection and that you've already cloned this repository on the robot.

## Using SSH terminal connection with the robot

### Environment setup

This environment setup is the quickest way to run the SDK examples. We have provided a docker compose file that will build the SDK example container and run it on the robot.

Build the docker container:

Run from the root of the repository

```bash
docker compose -f docker/docker-compose.yml up -d
```

Run & enter the docker container:

```bash
docker exec -it sdk-example-container bash
```

### Building

Build the SDK example application(s):

*Run in the root of the `~/ws` folder*

```bash
mkdir -p build
cd build
cmake ..
cmake --build . --config Debug --target all
```

The SDK example application(s) will be built in the `build/bin` directory.

### Running

Run the SDK example application(s) (from root of the repository or ws):

```bash
./build/bin/<example_name>
```

For example, to run the `getting_started` example:

```bash
./build/bin/hello_world_example
```

## Using a "VSCode Remote SSH: Dev Container"

### Environment setup

#### Pre-requisites

- [VSCode](https://code.visualstudio.com/)
- [Remote - SSH](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh) extension
- [Remote - Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension
- SSH connection to the robot
- Internet connection on the robot

#### Steps

1. Connect to the robot using the `Remote - SSH` extension
2. Open the SDK example directory in VSCode
3. Click on the `Reopen in Container` button in the bottom right corner of the VSCode window
   - If you don't see the button, open the command palette (Ctrl+Shift+P) and search for `Remote-Containers: Reopen in Container`
4. Select the `SDK Dev Container [Drone]` option
   - This will open VSCode inside a container with all the necessary tools to develop using the SDK examples
   - The first time you open the container, it will take a few minutes to build the container image
5. After the container has opened you will see on the bottom left corner of the VSCode window: `Dev Container: SDK Dev Container [Drone]` which indicates that you are inside the container

To go back to the host environment, click on the bottom left corner of the VSCode window and select `Reopen folder in SSH`.

### Building

Since the dev container is already setup with the necessary tools, you can build the SDK example application(s) using the CMake extension in VSCode.

1. Press `F7` to build the application(s)
   - When prompted to select a kit, select `GCC 12.x.x` 
2. The SDK example application(s) will be built in the `build/bin` directory

You can also open a terminal and use the instructions from the previous section to build the SDK examples.

### Running

You can run the SDK example application(s) from the terminal inside the dev container:

```bash
./build/bin/<example_name>
```

For example, to run the `getting_started` example:

```bash
./build/bin/hello_world_example
```

Or use the `Run and Debug` tab in VSCode to run the SDK examples:

1. Click on the `Run and Debug` tab in the sidebar
2. Select `(gdb) Debug example`
3. Select the example you want to run from the dropdown menu
