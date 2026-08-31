# System monitor example

This example demonstrates how to use SystemInfo, Diagnostic and SetPointControl API's to monitor the system. The code is written in C++ and uses the CreOS ROS SDK API to interact with the drone. View the [main.cpp](src/main.cpp) file to see how to use the SDK to monitor the system.

## Usage

```bash
source install/setup.bash
ros2 run system_monitor_example system_monitor_example
```

This example can be used to observe the system status when running other examples/applications.

## Expected output

On startup, system information is logged once, followed by continuous status updates whenever the battery, robot state, or control source changes:

```
System monitor example:
System info:
  - Name: Vertex One
  - Hostname: vertex-one-XXXX
  ...
Battery status:
  - State: Discharging
  - State of charge: 85%
  ...
State: Active - In flight
Control source: User
```

> **Note:** `Control source: User` corresponds to **SDK mode** as shown in QGroundControl and in the manual.

State and battery lines are reprinted whenever their values change.
