# TurtleBot3 Pose Control System (ROS 2 Humble)
This project implements a **custom goal-based navigation controller** for TurtleBot3 in Gazebo using ROS 2.  
The robot receives a target pose via a ROS 2 service and navigates using **odometry feedback + PID control**, without using Nav2.

---

# Getting Started

## 1. Set TurtleBot3 Model

This simulation uses the TurtleBot3 Burger model:

```bash
export TURTLEBOT3_MODEL=burger
```
# 2. Launch Gazebo Simulation
## Build the workspace
```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

# 3. Run the goal service node

```bash
source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot3_example turtlebot3_goal_service
```

# 4. Sending a Goal (Service Call)
The robot is controlled using a ROS2 service:

**turtlebot3_goal_server**

Service type:

**turtlebot3_msgs/srv/PoseCommand**

Service Call Format:

```bash
ros2 service call /turtlebot3_goal_server posecommand_msgs/srv/PoseCommand "{x: <float>, y: <float>, yaw: <float>}"
```
## Service Field Description

x (float64): Target position along the x-axis

y (float64): Target position along the y-axis

yaw (float64): Desired robot orientation in degrees
