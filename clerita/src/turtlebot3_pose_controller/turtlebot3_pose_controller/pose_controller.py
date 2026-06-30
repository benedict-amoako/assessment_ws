#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt, pow, atan2, sin, cos, pi
import tf_transformations
from pose_controller_interface.srv import SetPose

class PoseController(Node):

    def __init__(self):
        super().__init__('pose_controller')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.srv = self.create_service(SetPose, 'set_target_pose', self.set_target_pose_callback)

        # Target pose
        self.target_x = None
        self.target_y = None
        self.target_yaw = None

        # Current pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # PD gains
        self.Kp_lin = 0.4
        self.Kd_lin = 0.1
        self.Kp_ang = 1.0
        self.Kd_ang = 0.2
        self.Kp_yaw = 0.8
        self.Kd_yaw = 0.1

        # Tolerances
        self.pos_tolerance     = 0.05                # 5 cm
        self.heading_tolerance = 0.05                # ~3 degrees while driving
        self.yaw_tolerance     = 5.0 * pi / 180.0   # 5 degrees final yaw

        # Previous errors for derivative term
        self.prev_e_pos     = 0.0
        self.prev_e_heading = 0.0
        self.prev_e_yaw     = 0.0
        self.prev_time      = None

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.yaw = tf_transformations.euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        self.control_loop()

    def set_target_pose_callback(self, request, response):
        self.target_x   = request.x
        self.target_y   = request.y
        self.target_yaw = request.yaw * (pi / 180)
        # Reset derivative state for fresh start
        self.prev_e_pos     = 0.0
        self.prev_e_heading = 0.0
        self.prev_e_yaw     = 0.0
        self.prev_time      = None
        self.get_logger().info(
            f'New goal received: x={request.x}, y={request.y}, yaw={request.yaw} deg')
        response.success = True
        return response

    def control_loop(self):
        if self.target_x is None or self.target_y is None or self.target_yaw is None:
            return

        # --- Time delta ---
        now = self.get_clock().now().nanoseconds / 1e9
        if self.prev_time is None:
            self.prev_time = now
            return
        dt = now - self.prev_time
        if dt <= 0.0:
            return

        # --- Position errors ---
        
        e_x   = self.target_x - self.x
        e_y   = self.target_y - self.y
        e_pos = sqrt(pow(e_x, 2) + pow(e_y, 2))

        # --- Heading error toward target ---

        theta_target = atan2(e_y, e_x)
        e_heading    = atan2(sin(theta_target - self.yaw), cos(theta_target - self.yaw))

        # --- Final yaw error ---

        e_yaw = atan2(sin(self.target_yaw - self.yaw), cos(self.target_yaw - self.yaw))

        # --- Derivative terms ---

        d_pos     = (e_pos     - self.prev_e_pos)     / dt
        d_heading = (e_heading - self.prev_e_heading) / dt
        d_yaw     = (e_yaw     - self.prev_e_yaw)     / dt

        # --- PD outputs ---

        linear_speed  = self.Kp_lin * e_pos     + self.Kd_lin * d_pos
        angular_speed = self.Kp_ang * e_heading + self.Kd_ang * d_heading
        yaw_speed     = self.Kp_yaw * e_yaw     + self.Kd_yaw * d_yaw

        # --- Save state for next iteration ---

        self.prev_e_pos     = e_pos
        self.prev_e_heading = e_heading
        self.prev_e_yaw     = e_yaw
        self.prev_time      = now

        # --- Log current state ---

        self.get_logger().info(
            f'e_pos: {e_pos:.3f}m | e_heading: {e_heading*180/pi:.1f} deg | e_yaw: {e_yaw*180/pi:.1f} deg')

        twist = Twist()

        # Phase 1: Large heading error — rotate in place to face target

        if e_pos > self.pos_tolerance and abs(e_heading) > 0.5:
            twist.linear.x  = 0.0
            twist.angular.z = angular_speed

        # Phase 2: Heading acceptable — drive toward target

        elif e_pos > self.pos_tolerance:
            twist.linear.x  = min(linear_speed, 0.2)
            twist.angular.z = angular_speed if abs(e_heading) > self.heading_tolerance else 0.0

        # Phase 3: Position reached — correct final yaw

        elif abs(e_yaw) > self.yaw_tolerance:
            twist.linear.x  = 0.0
            twist.angular.z = yaw_speed

        # Goal fully reached

        else:
            twist.linear.x  = 0.0
            twist.angular.z = 0.0
            self.get_logger().info(
                f'Goal reached! Final pos error: {e_pos:.3f}m | '
                f'Final yaw error: {abs(e_yaw)*180/pi:.1f} deg')
            # Reset targets so robot waits for next goal
            self.target_x   = None
            self.target_y   = None
            self.target_yaw = None
            self.prev_time  = None

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PoseController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()