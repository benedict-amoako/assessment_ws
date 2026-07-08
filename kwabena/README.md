**# TurtleBot3 Pose Controller using ROS2 Humble**



**## Overview**

This project implements a custom pose controller for the TurtleBot3 Burger using ROS2 Humble and Gazebo Classic. The controller enables the robot to autonomously navigate to a requested target pose `(x, y, θ)` without using the ROS2 Navigation Stack (Nav2).

The project was developed as a demonstration of closed-loop mobile robot control using odometry feedback, proportional control, and custom ROS2 services.



**## Features**

\- Custom proportional pose controller

\- Closed-loop control using odometry feedback

\- Custom `MoveToPose` ROS2 service

\- Simultaneous linear and angular velocity control

\- Final orientation alignment at the target pose

\- Modular ROS2 package architecture

\- TurtleBot3 simulation in Gazebo Classic



**## Project Structure**

robot\_ws/

├── src/

│   ├── pse\_controller/

│   └── pse\_controller\_interfaces/



**### pse\_controller**

Contains the controller implementation, velocity computation, odometry processing, and service server.



**### pse\_controller\_interfaces**

Contains the custom ROS2 service definition (`MoveToPose.srv`) used for sending navigation goals.



**## Controller Architecture**

The controller operates as a continuous closed-loop feedback system.

1\. Subscribe to `/odom`

2\. Compute position and heading errors

3\. Generate linear and angular velocities using proportional control

4\. Publish velocity commands to `/cmd\_vel`

5\. Stop when both position and orientation tolerances are satisfied



**## Control Algorithm**

The controller computes:

\- Distance error

\- Heading error

\- Orientation error

A proportional controller continuously adjusts both linear and angular velocities while the robot approaches the goal.

Once the robot reaches the desired position, it performs a final orientation correction until the requested heading is achieved.



**## Installation**

Clone the repository:

```bash

git clone https://github.com/KB-Darko/Turtlebot3-pose-controller-ros2.git


cd robot\_ws

colcon build --symlink-install

source install/setup.bash



**## Running the Project**

Terminal 1

export TURTLEBOT3\_MODEL=burger

ros2 launch turtlebot3\_gazebo empty\_world.launch.py gui:=false

Terminal 2

gzclient

Terminal 3

source \~/robot\_ws/install/setup.bash

ros2 run pse\_controller controller



**## Sending a Goal**

Use the custom ROS2 service:

ros2 service call /move\_to\_pose pse\_controller\_interfaces/srv/MoveToPose \\

"{x: 2.0, y: 1.0, theta: 41.57}"



**## Results**

The controller successfully:

\- Navigates the TurtleBot3 to the requested position.

\- Simultaneously controls linear and angular motion.

\- Performs final orientation alignment.

\- Stops automatically once the requested pose has been reached.



**## Future Improvements**

\- PID controller tuning

\- Dynamic obstacle avoidance

\- Waypoint navigation

\- RViz goal visualization

\- Launch file automation



**## Author**

Kwabena Darko

Bsc. Electrical and Electronic Engineering , Ashesi University



