# Hello world example

This example demonstrates the basic usage of the CreOS ROS SDK. The code is written in C++ and uses the CreOS ROS SDK API to interact with the SDK server. View the [main.cpp](src/main.cpp) file to see how to use the SDK to make an API call.

## Usage

```bash
source install/setup.bash
ros2 run hello_world_example hello_world_example
```

## Expected output

The example subscribes to the `/robot/battery` topic and logs the battery state of charge whenever a new message is received:

```
[hello_world_node]: Battery state of charge: 85.00%
[hello_world_node]: Battery state of charge: 85.00%
...
```
