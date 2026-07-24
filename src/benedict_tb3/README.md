# TB3 Pose Controller

A closed-loop pose controller for the TurtleBot3 Burger. Call a service with
a target `(x, y, yaw)` and the robot drives to it under odometry feedback,
using a polar-coordinate control law rather than Nav2 or any other
navigation stack.

The motion is continuous: the robot translates and rotates at the same time
and curves onto the goal, rather than turning in place, then driving
straight, then turning again to fix its final heading.

## Package layout

```
assessment_ws/
└── src/
    ├── benedict_msgs/                        # SetGoalPose service definition
    └── benedict_tb3/
        ├── benedict_tb3/
        │   ├── tb3_pose_controller.py         # control node, control law, service
        │   └── tb3_pose_client.py             # CLI client for the service
        ├── launch/
        │   └── tb3_pose_controller.launch.py
        └── config/
            └── tb3_pose_controller.yaml
```

`benedict_msgs` is a separate package because custom service interfaces have
to be built with `ament_cmake`, while `benedict_tb3` is a plain
`ament_python` package.

## Requirements

- ROS 2 Humble
- Gazebo Classic, via `ros-humble-turtlebot3-gazebo`
- `ros-humble-tf-transformations`

```bash
sudo apt update
sudo apt install -y \
  ros-humble-turtlebot3 ros-humble-turtlebot3-msgs \
  ros-humble-turtlebot3-simulations ros-humble-turtlebot3-gazebo \
  ros-humble-tf-transformations
```

## Build

```bash
cd ~/assessment_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select benedict_msgs benedict_tb3
source install/setup.bash
```

## Running the simulation

Set the robot model and source the workspace in each terminal you use:

```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Launch Gazebo and the controller together:

```bash
ros2 launch benedict_tb3 tb3_pose_controller.launch.py
```

This brings up the TurtleBot3 empty world with the robot at the origin
(x = 0, y = 0, yaw = 0, following the REP-103 frame convention: x forward,
y left, yaw positive counter-clockwise about +z) and starts the controller
with the parameters in `config/tb3_pose_controller.yaml`.

If the simulation is already running elsewhere, start just the controller
with `ros2 run benedict_tb3 tb3_move_to_goal_node` (add
`--ros-args --params-file <path>` to load the same config file manually).

## Sending a goal

The service is `set_goal_pose` (`benedict_msgs/srv/SetGoalPose`). `x` and
`y` are in metres, `yaw` is in degrees. The service is fire-and-forget: it
accepts the goal and returns immediately, rather than blocking until the
robot arrives — watch the controller's log, or `/odom`, to see when that
happens. Only one goal is served at a time; a request sent while another
goal is active is rejected.

A small client is included:

```bash
ros2 run benedict_tb3 tb3_pose_client 1.0 0.5 45
ros2 run benedict_tb3 tb3_pose_client -1.0 0.8 90
ros2 run benedict_tb3 tb3_pose_client 0.5 -0.5 -90
```

You can also call the service directly:

```bash
ros2 service call /set_goal_pose benedict_msgs/srv/SetGoalPose \
  "{x: 1.0, y: 0.5, yaw: 45.0}"
```

## How it works

### The polar pose-regulation law

The control law is the polar-coordinate pose controller described in
Siegwart, Nourbakhsh & Scaramuzza, *Introduction to Autonomous Mobile
Robots* (2nd ed., MIT Press), Section 3.6.2, "Kinematic Position Control".

Every control tick (`compute_pose_errors` in `tb3_pose_controller.py`), the
pose error is expressed in polar form relative to the goal:

| Symbol | Meaning | Computation |
|--------|---------|-------------|
| `rho`   | Distance remaining to the goal position | `hypot(goal_x - x, goal_y - y)` |
| `alpha` | Heading error: angle between where the robot is pointing and the direction to the goal point | `wrap(atan2(dy, dx) - current_yaw)` |
| `beta`  | Orientation error: the extra turn still needed after arrival to reach the commanded final yaw | `wrap(goal_yaw - current_yaw - alpha)` |

`beta` is computed as `goal_yaw - current_yaw - alpha` rather than
`goal_yaw - atan2(dy, dx)` directly so both terms wrap consistently through
`current_yaw`; the two expressions are algebraically identical, since
`alpha = atan2(dy, dx) - current_yaw` cancels the `current_yaw` term out
either way.

These three errors drive the velocity commands:

```
v     = k_rho   * rho * cos(alpha)
omega = k_alpha * alpha + k_beta * beta
```

`v` and `omega` are commanded together every tick, which is what produces
the curved approach instead of turn-then-drive. The `cos(alpha)` factor
(an extension on top of the base textbook law) de-rates forward speed the
more the robot is misaligned, and lets it naturally back up rather than
drive forward when the goal is behind it, without disturbing local
stability near the goal where `cos(alpha) ~= 1`.

The gains (`k_rho = 0.3`, `k_alpha = 1.0`, `k_beta = -0.3`) satisfy the
textbook's necessary and sufficient conditions for local asymptotic
stability:

```
k_rho > 0            (0.3 > 0)
k_beta < 0            (-0.3 < 0)
k_alpha - k_rho > 0   (1.0 - 0.3 = 0.7 > 0)
```

### Continuous hand-off near the goal

Once `rho` drops within `pos_tolerance`, the controller only needs to settle
the final yaw, and switches to a pure `2.0 * yaw_err` rotation-in-place term.
Naively switching from the approach law to that final-orientation law at a
hard `rho == pos_tolerance` boundary would jump the commanded angular
velocity discontinuously whenever the two formulas disagree. Instead,
`proportional_controller` blends between them continuously as `rho`
approaches `pos_tolerance`, over a `blend_zone`-metre margin:

```
blend = clamp((rho - pos_tolerance) / blend_zone, 0, 1)
omega = blend * (k_alpha * alpha + k_beta * beta) + (1 - blend) * (2.0 * yaw_err)
```

`blend` is 1.0 (pure approach law) far from the goal and 0.0 (pure
final-orientation law) exactly at `pos_tolerance`, so the value it computes
matches what the tolerance branch below will compute the instant `rho`
crosses under the threshold.

### Settling, not just arriving

A real robot has rotational momentum: if the goal were declared reached the
instant `yaw_err` first dips under `ang_tolerance`, the robot could still be
turning and coast past that tolerance once commands are cut to zero. Instead
`yaw_err` must stay inside `ang_tolerance` for `settle_cycles` consecutive
ticks before the goal is declared reached, giving any residual rotation time
to settle out under active proportional control.

### Smooth commands

`clamp_velocity` bounds every command to the TurtleBot3 Burger's real
hardware limits (`max_lin_vel = 0.22` m/s, `max_ang_vel = 2.84` rad/s).
`slew_limit`, applied in `publish_velocity`, additionally bounds how much a
command may change per tick (`linear_accel_limit`, `angular_accel_limit`),
so velocity ramps smoothly instead of jumping — including the final stop,
which decelerates rather than cutting instantly.

## Parameters

All declared as ROS parameters (`config/tb3_pose_controller.yaml`) and
live-tunable via `ros2 param set`:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `k_rho`, `k_alpha`, `k_beta` | 0.3, 1.0, -0.3 | polar control-law gains |
| `max_lin_vel`, `max_ang_vel` | 0.22, 2.84 | hardware velocity limits (m/s, rad/s) |
| `linear_accel_limit`, `angular_accel_limit` | 0.5, 3.0 | slew-rate limits (m/s², rad/s²) |
| `pos_tolerance` | 0.04 m | position goal tolerance |
| `ang_tolerance` | 4.0 deg | final yaw goal tolerance |
| `blend_zone` | 0.15 m | margin above `pos_tolerance` over which the two control laws blend |
| `settle_cycles` | 5 | consecutive ticks `yaw_err` must stay settled before declaring the goal reached |

Point the launch file at a different config with
`params_file:=/path/to/your.yaml`.

## Interfaces

| Direction | Name | Type |
|-----------|------|------|
| Subscribe | `/odom` | `nav_msgs/Odometry` |
| Publish | `/cmd_vel` | `geometry_msgs/Twist` |
| Service | `/set_goal_pose` | `benedict_msgs/srv/SetGoalPose` |

## Assumptions and limitations

- The goal is interpreted in the same `odom` frame the robot starts in;
  the controller trusts odometry, which is fine for short moves but would
  drift over long distances without a map and localisation.
- No obstacle avoidance — this is pose control, not navigation.
- One goal is served at a time; a request during an active goal is rejected
  rather than queued.
- The service is fire-and-forget: a `success: true` response means the goal
  was accepted, not that the robot has arrived.

## License

Apache-2.0.
