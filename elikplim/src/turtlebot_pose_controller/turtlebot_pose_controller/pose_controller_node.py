#!/usr/bin/env python3
"""
TurtleBot3 Pose Controller Node

Implements a proportional closed-loop pose controller that drives the robot
to a commanded (x, y, yaw) pose via a ROS2 service.  Velocity commands are
published on /cmd_vel using geometry_msgs/Twist.

Control strategy
----------------
At each control cycle the node computes three error quantities:

  distance_error  – Euclidean distance from current position to goal.

  heading_error   – Angle from the robot's current heading to the direction
                    that points toward the goal (in the robot's body frame).
                    This steers the robot so it faces the goal while driving.

  yaw_error       – Angular difference between the robot's current yaw and
                    the commanded final yaw.  This is blended in once the
                    robot is close to the goal position so that the final
                    orientation is achieved smoothly.

Velocity mixing
---------------
Linear velocity is proportional to distance_error, clamped to a maximum.
Angular velocity combines heading_error (weighted heavily while far from goal)
and yaw_error (weighted more heavily once near the goal), giving simultaneous
drive-and-rotate behaviour throughout the motion.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from pose_interface.srv import GoToPose


def quaternion_to_yaw(q) -> float:
    """Extract yaw (rotation about Z) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PoseControllerNode(Node):

    # ------------------------------------------------------------------ #
    # Tolerances
    POSITION_TOLERANCE = 0.05   # metres  (±5 cm)
    YAW_TOLERANCE      = math.radians(5.0)  # radians (±5°)

    # Proportional gains
    KP_LINEAR  = 0.4   # linear  velocity gain  (m/s per metre of error)
    KP_HEADING = 1.2   # angular velocity gain for heading-to-goal error
    KP_YAW     = 0.8   # angular velocity gain for final yaw error

    # Velocity limits (TurtleBot3 Burger: 0.22 m/s, 2.84 rad/s)
    MAX_LINEAR  = 0.18  # m/s
    MAX_ANGULAR = 2.0   # rad/s

    # Distance threshold below which yaw_error starts to dominate
    YAW_BLEND_DIST = 0.3   # metres

    # Control loop frequency
    CONTROL_HZ = 20.0  # Hz
    # ------------------------------------------------------------------ #

    def __init__(self):
        super().__init__('pose_controller')

        self._cb_group = ReentrantCallbackGroup()

        # Current odometry state
        self._current_x:   float = 0.0
        self._current_y:   float = 0.0
        self._current_yaw: float = 0.0
        self._odom_ready:  bool  = False

        # Active goal (None when idle)
        self._goal_x:   float | None = None
        self._goal_y:   float | None = None
        self._goal_yaw: float | None = None
        self._active:   bool         = False

        # Subscribers / publishers
        self._odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self._odom_callback,
            10,
            callback_group=self._cb_group,
        )
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ROS2 service
        self._service = self.create_service(
            GoToPose,
            'go_to_pose',
            self._go_to_pose_callback,
            callback_group=self._cb_group,
        )

        # Control loop timer
        self._timer = self.create_timer(
            1.0 / self.CONTROL_HZ,
            self._control_loop,
            callback_group=self._cb_group,
        )

        self.get_logger().info('PoseControllerNode started – waiting for /odom …')

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _odom_callback(self, msg: Odometry) -> None:
        self._current_x   = msg.pose.pose.position.x
        self._current_y   = msg.pose.pose.position.y
        self._current_yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        if not self._odom_ready:
            self._odom_ready = True
            self.get_logger().info('Odometry received – controller ready.')

    def _go_to_pose_callback(
        self,
        request: GoToPose.Request,
        response: GoToPose.Response,
    ) -> GoToPose.Response:

        if not self._odom_ready:
            response.success = False
            response.message = 'Odometry not yet available.'
            return response

        if self._active:
            response.success = False
            response.message = 'Controller already executing a goal.'
            return response

        self._goal_x   = request.target_x
        self._goal_y   = request.target_y
        self._goal_yaw = math.radians(request.target_yaw_deg)
        self._active   = True

        self.get_logger().info(
            f'New goal received: x={self._goal_x:.3f} m, '
            f'y={self._goal_y:.3f} m, '
            f'yaw={request.target_yaw_deg:.1f}°'
        )

        # Block inside the service callback until the goal is reached.
        # Because we use a ReentrantCallbackGroup the control timer keeps
        # firing on a separate thread while this callback blocks.
        rate = self.create_rate(self.CONTROL_HZ)
        while rclpy.ok() and self._active:
            rate.sleep()

        response.success = True
        response.message = (
            f'Goal reached: x={self._goal_x:.3f}, '
            f'y={self._goal_y:.3f}, '
            f'yaw={request.target_yaw_deg:.1f}°'
        )
        return response

    # ------------------------------------------------------------------ #
    # Control loop
    # ------------------------------------------------------------------ #

    def _control_loop(self) -> None:
        """Proportional pose controller executed at CONTROL_HZ."""
        if not self._active or not self._odom_ready:
            return

        dx = self._goal_x - self._current_x
        dy = self._goal_y - self._current_y

        distance_error = math.hypot(dx, dy)

        # Angle from current position toward goal (world frame)
        angle_to_goal = math.atan2(dy, dx)

        # Heading error: difference between where the robot points and the
        # direction to the goal, expressed in the robot's body frame.
        heading_error = normalize_angle(angle_to_goal - self._current_yaw)

        # Final yaw error
        yaw_error = normalize_angle(self._goal_yaw - self._current_yaw)

        # ---- Check termination ---------------------------------------- #
        if (distance_error < self.POSITION_TOLERANCE and
                abs(yaw_error) < self.YAW_TOLERANCE):
            self._stop()
            self._active = False
            self.get_logger().info(
                f'Goal reached.  Final error: dist={distance_error*100:.1f} cm, '
                f'yaw={math.degrees(yaw_error):.1f}°'
            )
            return

        # ---- Velocity mixing ------------------------------------------ #
        # Linear: proportional to distance, reduced when not facing goal.
        # We attenuate forward velocity when the heading error is large so
        # the robot does not overshoot in the wrong direction.
        heading_factor = math.cos(heading_error)  # ∈ [-1, 1]
        # Only drive forward (do not reverse toward goal)
        heading_factor = max(0.0, heading_factor)

        linear_vel = self.KP_LINEAR * distance_error * heading_factor
        linear_vel = min(linear_vel, self.MAX_LINEAR)

        # Angular: blend heading error (steer toward goal) and yaw error
        # (achieve final orientation).  When far from goal, heading dominates;
        # when close, yaw_error gets a bigger weight.
        blend = max(0.0, 1.0 - distance_error / self.YAW_BLEND_DIST)  # 0→1
        angular_vel = (
            (1.0 - blend) * self.KP_HEADING * heading_error
            + blend       * self.KP_YAW     * yaw_error
        )
        angular_vel = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, angular_vel))

        # Stop linear motion when only yaw correction remains
        if distance_error < self.POSITION_TOLERANCE:
            linear_vel = 0.0

        self._publish_velocity(linear_vel, angular_vel)

    # ------------------------------------------------------------------ #
    # Helpers
    # ------------------------------------------------------------------ #

    def _stop(self) -> None:
        self._publish_velocity(0.0, 0.0)

    def _publish_velocity(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x  = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #

def main(args=None):
    rclpy.init(args=args)
    node = PoseControllerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
