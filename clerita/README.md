# TurtleBot3 Pose Controller

A custom closed-loop PD pose controller for TurtleBot3 built with ROS2 Humble. This package implements a three-phase proportional-derivative controller that drives the robot to a target (x, y, yaw) pose without using Nav2.

---

## Package Structure

```
ros2_wsp/
└── src/
    ├── turtlebot3_pose_controller/   # Main controller node (Python)
    └── pose_controller_interfaces/   # Custom SetPose service definition
```

---

## Dependencies

### System Requirements
- Ubuntu 22.04
- ROS2 Humble
- Python 3.10

### Install ROS2 Humble
Follow the official guide: https://docs.ros.org/en/humble/Installation.html

### Install TurtleBot3 packages
```bash
sudo apt update
sudo apt install -y ros-humble-turtlebot3 \
                   ros-humble-turtlebot3-simulations \
                   ros-humble-turtlebot3-gazebo
```

### Install Python dependencies
```bash
sudo apt install -y ros-humble-tf-transformations
pip install transforms3d
```

### Set TurtleBot3 model
Add this to your `~/.bashrc` so it persists across terminals:
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

---

## Building the Workspace

```bash
cd ~/ros2_wsp
colcon build
source install/setup.bash
```

> **Note:** Run `source install/setup.bash` in every new terminal before using the packages.

---

## Running the Simulation

### Terminal 1 — Launch Gazebo with TurtleBot3
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_wsp/install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Wait until Gazebo fully loads and the robot is visible in the world before proceeding.

---

## Starting the Control Node

### Terminal 2 — Run the pose controller
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_wsp/install/setup.bash
ros2 run turtlebot3_pose_controller pose_controller
```

The node will start silently and wait for a goal. You should see:
```
[pose_controller]: New goal received: x=..., y=..., yaw=... deg
```
once a service call is made.

---

## Calling the Service

### Terminal 3 — Send a target pose
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_wsp/install/setup.bash
ros2 service call /set_target_pose pose_controller_interfaces/srv/SetPose "{x: 1.0, y: 0.5, yaw: 45.0}"
```

### Service fields
| Field | Type | Description |
|-------|------|-------------|
| `x` | float64 | Target X position in metres |
| `y` | float64 | Target Y position in metres |
| `yaw` | float64 | Target orientation in **degrees** |

### Example goals
```bash
# Goal 1 — move forward and turn
ros2 service call /set_target_pose pose_controller_interfaces/srv/SetPose "{x: 1.0, y: 0.0, yaw: 0.0}"

# Goal 2 — move diagonally
ros2 service call /set_target_pose pose_controller_interfaces/srv/SetPose "{x: 1.5, y: 1.0, yaw: 90.0}"

# Goal 3 — return to origin
ros2 service call /set_target_pose pose_controller_interfaces/srv/SetPose "{x: 0.0, y: 0.0, yaw: 0.0}"
```

---

## Controller Behaviour

The controller uses a **three-phase PD control** strategy:

1. **Phase 1 — Rotate to face target:** If heading error > 0.5 rad (~28°), the robot rotates in place until it is roughly pointing at the target.
2. **Phase 2 — Drive to target:** The robot drives forward with a proportional speed capped at 0.2 m/s. Small heading corrections are applied if the robot drifts.
3. **Phase 3 — Correct final yaw:** Once within 5 cm of the target, the robot rotates in place to match the desired orientation.

### Tolerances
| Parameter | Value |
|-----------|-------|
| Position tolerance | ±5 cm |
| Heading tolerance (driving) | ±3° |
| Final yaw tolerance | ±5° |

### PD Gains
| Gain | Value | Purpose |
|------|-------|---------|
| `Kp_lin` | 0.4 | Proportional linear speed |
| `Kd_lin` | 0.1 | Derivative linear damping |
| `Kp_ang` | 1.0 | Proportional angular speed |
| `Kd_ang` | 0.2 | Derivative angular damping |
| `Kp_yaw` | 0.8 | Proportional final yaw speed |
| `Kd_yaw` | 0.1 | Derivative final yaw damping |

---

## Verifying the Node is Running

```bash
# Check node is active
ros2 node list

# Check service is available
ros2 service list

# Monitor odometry (robot position)
ros2 topic echo /odom --once
```

---

## Troubleshooting

**`ModuleNotFoundError: No module named 'pose_controller_interfaces'`**
```bash
cd ~/ros2_wsp
colcon build
source install/setup.bash
```

**Robot not moving after service call**
- Make sure Gazebo is fully loaded before running the controller
- Check that `TURTLEBOT3_MODEL=burger` is set in every terminal
- Verify `/odom` is publishing: `ros2 topic echo /odom --once`

**Service call returns error**
- Confirm the controller node is running in Terminal 2
- Run `ros2 service list` to verify `/set_target_pose` is listed

---

## Author
Clerita — ROS2 Robotics Assessment
