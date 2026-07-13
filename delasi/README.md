# Fortress ROS2 Assessment

## Overview

This is a ROS2 project to move a Turtlebot3 robot to a specified pose using ROS2 services and topics.

The project makes use of ROS2 Humble and Gazebo 11 for simulating the movement of the Turtlebot3.


## Installing Dependencies
1. Clone this repository to your local machine

2. Run the following command inside the repository folder
```shell
colcon build
source script.bash
```
The `script.bash` file contains commands to source the environment and set up the necessary environment variables.


## Running the Simulation

1. To run the service node, run the following command
```shell
ros2 launch fortress_tut simulation_launch.py
```

2. To run the client node, run the following command
```shell
ros2 run fortress_tut commander <x_position> <y_position> <yaw>
```
This will send a request to the service to move the Turtlebot3 to the specified pose.


## Simulation Background

### ROS2 Nodes and Services

The simulation uses two ROS2 nodes:
1. `/robot` operates the service (i.e. service server).
2. `/commander` sends client requests to the service.

The service is named `move_turtlebot3`.
It accepts a  `fortress_tut_msgs/srv/MoveTurtlebot` interface.
```shell
float64 x
float64 y
float64 yaw
---
bool success
float64 x
float64 y
float64 yaw
```
The request sends the coordinates for the pose and the response contains a flag indicating the success of the operation and the current location of the turtlebot.

To send information to the Turtlebot3 simulation, two topics were used:
1. `/cmd_vel` allows for the publishing of linear and angular velocity commands to the turtlebot.
2. `/odom` allows for the retrieval of information on the robot's current odometry.


### Control of the Robot

To ensure accuracy, the service uses PID control logic to control the output commands to the turtlebot's node.
The service uses two controllers:
1. A linear motion controller which moves the robot to the specified x and y positional coordinates.
2. An angular motion controller which moves the robot to the specified yaw angle.

The linear motion controller is used to move the robot forward and rotate the robot towards the required x-y position.
This causes the robot to move forward while being steered towards the x-y coordinates provided.
It therefore uses two (2) control parameters:
the robot's distance to the x-y coordinates (obtained by converting the x-y coordinates to vector form)
and the yaw angle the robot has to steer to face the desired x-y coordinates.

After moving to the x-y coordinates, the angular motion controller rotates the robot to match the specified yaw angle.

The PID controller was operated as a digital one, thereby making use of a sampler.

### Node Parameters

The /robot node uses the following parameters, which are contained in the `./src/fortress_tut/config/robot.yaml` file
```
    Kpp: The proportional controller constant for the linear motion controller.
    Kp, Ki, Kd: The PID constants for the yaw controller of the linear motion controller.
    Kpo: The proportional controller constant of the angular motion controller.
    Ts: The sampling time of the controller.
    tol_x: Controller tolerance on the final x-axis position
    tol_y: Controller tolerance on the final y-axis position
    tol_yaw: Controller tolerance on the final yaw angle.
```


## Codebase
Inside the `./src` folder are the following folders
```
DynamixelSDK
fortress_tut
fortress_tut_msgs
turtlebot3
turtlebot3_msgs
turtlebot3_simulations
```

`fortress_tut` contains the main code for the nodes for this project.
`fortress_tut_msgs` contains a custom interface for the pose service.

The remaining packages provide the functionality for running and operating the Turtlebot in Gazebo.