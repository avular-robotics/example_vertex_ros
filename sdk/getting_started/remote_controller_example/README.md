# Remote Controller Example

This example demonstrates how to subscribe to remote controller input and print the live stick and button state in the console.

The example doesn't provide a button name to button indices mapping. This is because the Vertex platform may have different buttons depending on the controller used.

For the Herelink controller the mapping is as follows:

| Button  | Index |
|---------|-------|
| A       | 0     |
| B       | 1     |
| C       | 2     |
| D       | 3     |
| Trigger | 4     |
| Home    | 5     |
| Wheel   | 6     |

## Structure

The code is split up into multiple parts:

- [RcUtils](src/rc_utils.hpp): Helper that formats controller axes and button states for console output.
- [Main](src/main.cpp): Main sets up all the classes and runs the main loop.

## RcUtils

This class is responsible for turning each `ControllerState` message into a readable console display. Axes are printed with their numeric value and a small centered bar. Each button shows a rolling 20-cycle history: `*` is pressed/up, `v` is down, `#` is a long press, and `.` is released or another state.

## Main

The main function sets up all the classes and runs the main loop. It handles the CLI arguments, subscribes to the correct APIs, and registers the appropriate callbacks.

The main logic of the example can be found in the `handleRemoteControllerInput` function. This function is called every time a controller data message is received by the SDK client, then logs the formatted input state.

## Usage

The example can be executed by running the following command:

```bash
./build/bin/remote_controller_example
```

The following CLI arguments can be used:

- `--verbose`: Enable verbose output.
- `--host`: CREOS agent host and port.

Example:

```bash
./build/bin/remote_controller_example --host localhost:7200
```

The example can be stopped by pressing `Ctrl + C`.
