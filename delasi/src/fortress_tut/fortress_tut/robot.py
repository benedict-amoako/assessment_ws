#! usr/bin/env python3

from math import atan, atan2, sqrt, radians
from functools import partial

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Twist
from fortress_tut_msgs.srv import MoveTurtlebot


class RobotNode(Node):

    def __init__(self):
        super().__init__("robot")
        self.get_logger().info("Started the robot service node\n")

        self.declare_parameter("Ts", 0.05)
        self.declare_parameter("Kpp", 0.1)
        self.declare_parameter("Kp", 1.0)
        self.declare_parameter("Ki", 0.075)
        self.declare_parameter("Kd", 2.5)
        self.declare_parameter("Kpo", 1.0)
        self.declare_parameter("tol_x", 0.05)
        self.declare_parameter("tol_y", 0.05)
        self.declare_parameter("tol_yaw", 0.087266)

        self.move_service = self.create_service(MoveTurtlebot, "move_turtlebot3", self.move_turtlebot3)
        self.pub_cmd = self.create_publisher(Twist, "cmd_vel", 10)

        self.pose_subscription = self.create_subscription(Odometry, "/odom", self.get_pose, 10)

        self.current_pose = Vector3()
        self.previous_cmd = Vector3()

        self.yaw_integral = 0.0
        self.yaw_prev_error = None

    def get_pose(self, odom: Odometry):
        self.current_pose.x = odom.pose.pose.position.x
        self.current_pose.y = odom.pose.pose.position.y
        orientation = odom.pose.pose.orientation
        self.current_pose.z = atan2(
            2 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1 - 2 * (orientation.y ** 2 + orientation.z ** 2)
        )

    def position_callback(self, desired_x: float, desired_y: float, desired_yaw: float):
        Ts = self.get_parameter("Ts").get_parameter_value().double_value
        error_x = (desired_x - self.current_pose.x)
        error_y = (desired_y - self.current_pose.y)
        if (
            (abs(error_x) < self.get_parameter("tol_x").get_parameter_value().double_value) and 
            (abs(error_y) < self.get_parameter("tol_y").get_parameter_value().double_value)
        ):
            stop_signal = Twist()
            self.pub_cmd.publish(stop_signal)
            self.get_logger().info("Position movement control complete\n")
            self.yaw_integral = 0.0
            self.yaw_prev_error = None
            self.destroy_timer(self.position_controller_sampler)
            self.position_controller_sampler = None
            self.orientation_controller_sampler = self.create_timer(Ts, partial(self.orientation_callback, desired_yaw))
            return
        else:
            error_position = sqrt(error_x ** 2 + error_y ** 2)
            error_yaw = atan2(error_y, error_x) - self.current_pose.z
            # if (error_x < 0) and (error_y < 0):
            #     error_position = 0 - error_position
            self.get_logger().info(f"Current position error\tx: {error_x:.4f}m\ty: {error_y:.4f}m\t")
            if self.yaw_prev_error is None:
                self.yaw_prev_error = error_yaw

            cmd_msg = Twist()
            cmd_msg.linear.x = error_position * self.get_parameter("Kpp").get_parameter_value().double_value
            self.yaw_integral += error_yaw * Ts
            cmd_msg.angular.z = (
                (error_yaw * self.get_parameter("Kp").get_parameter_value().double_value) + 
                (self.yaw_integral * self.get_parameter("Ki").get_parameter_value().double_value) + 
                (((error_yaw - self.yaw_prev_error) / Ts) * self.get_parameter("Kd").get_parameter_value().double_value)
            )
            self.pub_cmd.publish(cmd_msg)
            self.yaw_prev_error = error_yaw

    def orientation_callback(self, desired_yaw: float):
        error_yaw = desired_yaw - self.current_pose.z
        if (abs(error_yaw) < self.get_parameter("tol_yaw").get_parameter_value().double_value):
            stop_signal = Twist()
            self.pub_cmd.publish(stop_signal)
            self.get_logger().info("Orientation movement control complete")
            self.destroy_timer(self.orientation_controller_sampler)
            self.orientation_controller_sampler = None
            return
        else:
            self.get_logger().info(f"Current yaw error: {error_yaw}r")
            cmd_msg = Twist()
            cmd_msg.angular.z = error_yaw * self.get_parameter("Kpo").get_parameter_value().double_value
            self.pub_cmd.publish(cmd_msg)       

    def move_turtlebot3(self, request: MoveTurtlebot.Request, response: MoveTurtlebot.Response):
        response.success = False
        Ts = self.get_parameter("Ts").get_parameter_value().double_value
        self.position_controller_sampler = self.create_timer(Ts, partial(self.position_callback, request.x, request.y, radians(request.yaw)))
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()