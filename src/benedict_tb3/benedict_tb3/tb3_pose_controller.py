"""Closed-loop pose controller for a TurtleBot3, built around Siegwart's
polar-coordinate pose-regulation law (Siegwart, Nourbakhsh & Scaramuzza,
"Introduction to Autonomous Mobile Robots", 2nd ed., MIT Press, Sec. 3.6.2,
"Kinematic Position Control").

The robot is driven to a commanded (x, y, yaw) by decomposing the pose error
into three terms every control tick:

    rho   -- Euclidean distance remaining to the goal position
    alpha -- heading error: how far off the robot is from pointing at the
             goal point
    beta  -- the extra turn still needed after arrival to reach the
             commanded final yaw

and feeding them through

    v     = k_rho   * rho * cos(alpha)
    omega = k_alpha * alpha + k_beta * beta

which drives and steers simultaneously rather than rotating in place, then
driving, then rotating again. The textbook's necessary-and-sufficient local
stability conditions are k_rho > 0, k_beta < 0, k_alpha - k_rho > 0; the
cos(alpha) factor is a refinement on top of the base law that naturally
de-rates forward speed (and lets the robot back up) when it is badly
misaligned, without disturbing local stability near the goal, where
cos(alpha) ~= 1. See README.md for the full derivation.

Frames follow REP-103 (x forward, y left, yaw positive counter-clockwise
about +z), matching the odometry frame published on /odom.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from benedict_msgs.srv import SetGoalPose
from tf_transformations import euler_from_quaternion
from rcl_interfaces.msg import SetParametersResult
import math


class TB3PoseController(Node):
    """Drives a TurtleBot3 to a commanded pose on receipt of a /set_goal_pose
    service call.

    Subscribes to /odom for the current pose, publishes Twist commands on
    /cmd_vel, and runs the control law on a fixed-rate timer decoupled from
    the odometry callback rate. Only one goal is served at a time; a new
    request is rejected while a goal is already active. The service itself
    is fire-and-forget: it accepts a goal and returns immediately rather than
    blocking until the robot arrives (watch this node's log, or /odom, to see
    when that actually happens).
    """

    def __init__(self):
        super().__init__('tb3_pose_controller')

        # Define QoS profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Current pose, updated by odom_callback on every /odom message.
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0  # radians

        # Goal pose, set once per accepted /set_goal_pose request.
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0  # radians

        # Siegwart polar-law gains: k_rho > 0, k_beta < 0, k_alpha - k_rho > 0
        # for local asymptotic stability (see module docstring).
        self.declare_parameter('k_rho', 0.3)
        self.declare_parameter('k_alpha', 1.0)
        self.declare_parameter('k_beta', -0.3)
        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.k_beta = self.get_parameter('k_beta').value

        self.dx = 0.0
        self.dy = 0.0

        # Hardware velocity limits (TurtleBot3 Burger).
        self.declare_parameter('max_lin_vel', 0.22)
        self.declare_parameter('max_ang_vel', 2.84)
        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value

        # Acceleration limits: cap how much commanded velocity may change per
        # tick so it ramps instead of jumping, on top of the hard max above.
        self.declare_parameter('linear_accel_limit', 0.5)  # m/s^2
        self.declare_parameter('angular_accel_limit', 3.0)  # rad/s^2
        self.linear_accel_limit = self.get_parameter('linear_accel_limit').value
        self.angular_accel_limit = self.get_parameter('angular_accel_limit').value
        self.control_dt = 0.1

        # One-goal-at-a-time gating.
        self.goal_active = False
        self.declare_parameter('pos_tolerance', 0.04)  # metres
        self.declare_parameter('ang_tolerance', 4.0)  # degrees, converted to radians below
        # Width, in metres above pos_tolerance, over which the approach law
        # blends into the final-orientation law so the two phases meet
        # continuously instead of switching at a hard boundary.
        self.declare_parameter('blend_zone', 0.15)
        # Number of consecutive ticks yaw_err must stay inside ang_tolerance
        # before the goal is declared reached. Without this, the robot can
        # still have real rotational momentum the instant yaw_err first dips
        # under tolerance, and coasts past it once commands are cut to zero.
        self.declare_parameter('settle_cycles', 5)
        self.pos_tolerance = self.get_parameter('pos_tolerance').value
        self.ang_tolerance = math.radians(self.get_parameter('ang_tolerance').value)
        self.blend_zone = self.get_parameter('blend_zone').value
        self.settle_cycles = self.get_parameter('settle_cycles').value
        self.yaw_settle_count = 0

        self.add_on_set_parameters_callback(self.parameters_callback)

        self.cmd_vel = Twist()

        # Create subscribers
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        # Create publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile
        )

        # Create service
        self.goal_service = self.create_service(
            SetGoalPose,
            'set_goal_pose',
            self.set_goal_pose_callback
        )

        # Periodic control loop, decoupled from the /odom publish rate.
        self.control_timer = self.create_timer(self.control_dt, self.control_loop_callback)

        self.get_logger().info('TB3 Pose Controller initialized')

    def odom_callback(self, msg):
        """Updates current_x, current_y, current_yaw from an /odom message."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to euler angles
        orientation_q = msg.pose.pose.orientation
        euler = euler_from_quaternion([
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        ])

        self.current_yaw = euler[2]

    def parameters_callback(self, params):
        """Applies tunable parameter changes made at runtime (e.g. via
        `ros2 param set`), so gains and tolerances can be retuned live
        without restarting the node."""
        tunable = (
            'k_rho', 'k_alpha', 'k_beta',
            'max_lin_vel', 'max_ang_vel',
            'linear_accel_limit', 'angular_accel_limit',
            'pos_tolerance', 'ang_tolerance', 'blend_zone', 'settle_cycles',
        )
        for param in params:
            if param.name == 'ang_tolerance':
                self.ang_tolerance = math.radians(param.value)
            elif param.name in tunable:
                setattr(self, param.name, param.value)
        return SetParametersResult(successful=True)

    def set_goal_pose_callback(self, request, response):
        """Service callback for /set_goal_pose.

        Fire-and-forget: accepts the goal (converting yaw from the request's
        degrees to radians) and returns immediately rather than blocking
        until the robot arrives. Rejects the request if a goal is already in
        progress, since only one goal is served at a time.
        """
        if self.goal_active:
            self.get_logger().warn('Goal already in progress; rejecting new goal until current one completes')
            response.success = False
            return response

        self.goal_x = request.x
        self.goal_y = request.y
        self.goal_yaw = math.radians(request.yaw)
        self.goal_active = True

        self.get_logger().info(f'Goal set to x={self.goal_x}, y={self.goal_y}, yaw={math.degrees(self.goal_yaw)}')
        response.success = True
        return response

    def wrap_to_pi(self, angle):
        """Wraps an angle in radians to the interval [-pi, pi]."""
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def clamp_velocity(self, velocity, clamp):
        """Clamps velocity to the interval [-clamp, clamp]."""
        return max(-clamp, min(velocity, clamp))

    def slew_limit(self, target, current, max_step):
        """Limits the change from current to target to at most max_step, so
        commanded velocity ramps instead of jumping instantaneously."""
        return current + self.clamp_velocity(target - current, max_step)

    def publish_velocity(self, linear_target, angular_target):
        """Ramps toward the (already hardware-clamped) target velocities at
        the configured acceleration limits, publishes, and remembers the
        result in self.cmd_vel so the next tick ramps from where this one
        left off."""
        lin_step = self.linear_accel_limit * self.control_dt
        ang_step = self.angular_accel_limit * self.control_dt

        self.cmd_vel.linear.x = self.slew_limit(linear_target, self.cmd_vel.linear.x, lin_step)
        self.cmd_vel.angular.z = self.slew_limit(angular_target, self.cmd_vel.angular.z, ang_step)
        self.cmd_vel_publisher.publish(self.cmd_vel)

    def compute_pose_errors(self):
        """Derives Siegwart's (rho, alpha, beta) from the current and goal
        poses:

            rho   = |goal_position - current_position|
            alpha = wrap(angle_to_goal - current_yaw)
            beta  = wrap(goal_yaw - current_yaw - alpha)
                  = wrap(goal_yaw - angle_to_goal)      [algebraically equal]

        beta is computed via the current_yaw-cancelling form above rather
        than goal_yaw - angle_to_goal directly so it wraps consistently
        through the same reference as alpha; the two expressions are the
        same value.
        """
        self.dx = self.goal_x - self.current_x
        self.dy = self.goal_y - self.current_y

        pos_err = math.hypot(self.dx, self.dy)

        ang_to_goal = math.atan2(self.dy,self.dx)
        heading_err = self.wrap_to_pi(ang_to_goal - self.current_yaw)

        orr_err = self.wrap_to_pi(self.goal_yaw - self.current_yaw - heading_err)
        return (pos_err, heading_err, orr_err)

    def proportional_controller(self):
        """Runs one control tick of the polar pose-regulation law and
        publishes the resulting velocity command.

        While outside pos_tolerance, drives and steers simultaneously using
        the Siegwart law, blending continuously into the final-orientation
        law as rho approaches pos_tolerance (see blend_zone in __init__).
        Once inside pos_tolerance, rotates in place to close any remaining
        yaw error, and only declares the goal reached once that error has
        stayed inside ang_tolerance for settle_cycles consecutive ticks.
        """
        rho, alpha, beta = self.compute_pose_errors()
        yaw_err = self.wrap_to_pi(self.goal_yaw - self.current_yaw)

        self.get_logger().info(
            f"rho={rho:.3f} "
            f"alpha={math.degrees(alpha):.1f} "
            f"beta={math.degrees(beta):.1f} "
            f"yaw_err={math.degrees(yaw_err):.1f} "
            f"current_yaw={math.degrees(self.current_yaw):.1f} "
            f"goal_yaw={math.degrees(self.goal_yaw):.1f}"
        )

        if rho > self.pos_tolerance:
            # v = k_rho * rho * cos(alpha): de-rates (and can reverse)
            # forward speed while badly misaligned, instead of driving at
            # full speed regardless of heading.
            linear_controller = self.k_rho * rho * math.cos(alpha)

            approach_controller = (self.k_alpha * alpha) + (self.k_beta * beta)
            final_controller = 2.0 * yaw_err
            # blend goes from 1.0 (pure approach law) far from the goal to
            # 0.0 (pure final-orientation law) right at pos_tolerance, so it
            # matches what the tolerance branch below will compute the
            # instant rho crosses under pos_tolerance -- no discontinuity.
            blend = min(1.0, (rho - self.pos_tolerance) / self.blend_zone)
            angular_controller = (
                blend * approach_controller + (1.0 - blend) * final_controller
            )

            linear_target = self.clamp_velocity(linear_controller, self.max_lin_vel)
            angular_target = self.clamp_velocity(angular_controller, self.max_ang_vel)

            self.publish_velocity(linear_target, angular_target)
            return

        if abs(yaw_err) < self.ang_tolerance:
            self.yaw_settle_count += 1
            if self.yaw_settle_count >= self.settle_cycles:
                self.publish_velocity(0.0, 0.0)
                self.goal_active = False
                self.yaw_settle_count = 0
                self.get_logger().info('Goal reached')
                return
        else:
            self.yaw_settle_count = 0

        # Final-orientation-only phase: rotate in place to close yaw_err.
        angular_target = self.clamp_velocity(2.0 * yaw_err, self.max_ang_vel)
        self.publish_velocity(0.0, angular_target)

    def control_loop_callback(self):
        """Periodic control loop: drives toward the active goal and releases
        the one-goal-at-a-time lock once the robot arrives."""
        if not self.goal_active:
            return

        self.proportional_controller()


def main(args=None):
    """Entry point: ros2 run benedict_tb3 tb3_move_to_goal_node"""
    rclpy.init(args=args)
    controller = TB3PoseController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
