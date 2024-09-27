# Circle example

This example can be used to control the Vertex through setpoints. The setpoints are created within the example.

Based on your current state of the drone, it will first arm and/or take-off. Once it is flying, it wil use it's current positon to start flying in circles. The radius and the execution time can be changed in the code together with the control mode.

#### Position mode
In position setpoint control mode, it will send position setpoints with velocity and acceleration setpoints as feedforward. In this mode, it will keep flying in the same spot all the time.

#### Velocity mode
In velocity setpoint control mode, it will send velocity setpoints with acceleration setpoints as feedforward. In this mode, it will keep flying with the same velocity all the time, but it might drift in position depending on external factors like wind.

#### Acceleration mode
In acceleration setpoint control mode, it will send acceleration setpoints to the flight controller. In this mode, it will keep flying with the same acceleration all the time. Depending on external factors like wind, it might not be able to reach a certain velocity.


Enabling the example can be done with the C button.

If the expected velocities are too high, it will not be able to follow the setpoints.

## How to run the example

You can start the example by running the commands below from the root folder of the example. In this case the `circle_example` folder.
This folder contains a docker-compose file which will start the container and run the example within the docker container. The docker compose file sets up the necessary ROS environment.

First of all, you need to start the container. You can do this by running the following command:
```bash
docker compose up -d
```

To enter the example container, you can run the following command:
```bash
docker exec -it circle_example /bin/bash
```

In the container, we have a user named `user`. This user has sudo rights, so you can install packages and run commands as root. When entering 
the container, you will be in the `/home/user/ws` directory. This is the workspace directory where you can start developing your code. This workspace directory is also mounted from the host OS, this is done so that you can easily `down` and `up` the container without losing your code. This directory also contains the example code.

> [!WARNING]
> Be aware that recreating the container will remove all files outside the workspace directory.

By default the container does not automatically start the example. You can start the example by running the following commands:
```bash
source install/setup.bash
ros2 run drone_circle_example circle_example
```

This will start the example and display instructions on how to use the example.

## Develop using the example

If you want to make changes to the example code you can do this by editing the files in the `circle_example/src` directory.

After you have made changes to the code, you can build the code by running the following command in the root of the `ws` directory:
```bash
colcon build
```

Once the build is completed, you can start the example with:
```
ros2 run drone_circle_example circle_example
```

If you change package dependencies, you need to re-install the dependencies by running the following command:
```bash
source install/setup.bash+
```

## Monitor ROS2 topics
If you want to monitor ROS2 topics, you first have to run `source install/setup.bash`. Now you can show topic outputs. For example by running:
```
ros2 topic echo /robot/sensor/battery
```
