import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from posecommand_msgs.srv import PoseCommand
from tf_transformations import euler_from_quaternion


class GoalService(Node):
    def __init__(self):
        super().__init__("turtlebot3_goal_service")
        self.service_ = self.create_service(PoseCommand, 'turtlebot3_goal_server', self.serviceCallback)

        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel',10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.kp_linear = 0.5
        self.ki_linear = 0.01
        self.kd_linear = 0.1
        self.kp_angular = 1.5
        self.ki_angular = 0.01
        self.kd_angular = 0.1
        self.linear_integral = 0.0
        self.angular_integral = 0.0
        self.prev_distance_error = 0.0
        self.prev_angular_error = 0.0
        self.linear_speed = 0.1
        self.angular_speed = 0.1

        self.prev_time = None

        self.cmd_vel = Twist()


        self.goal = None
        self.active = False

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Ready to receive goal inputs.')

    def serviceCallback(self, request, response):
        self.goal = (request.x, request.y, math.radians(request.yaw))
        self.active = True

        response.success = True
        response.message = "Goal received"
        return response

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def control_loop(self):
        if not self.active:
            return
        
        gx, gy, gyaw = self.goal

        dx = gx - self.x
        dy = gy - self.y
        distance = math.sqrt(dx*dx + dy*dy)
        t = self.get_clock().now()

        if self.prev_time is None:
            self.prev_time = t
            return
        dt = (t - self.prev_time).nanoseconds / S_TO_NS
        if dt <= 0.0:
            return

        self.get_logger().info(f"Position Error | x : {dx:.3f} y : {dy:.3f}")

        
    
        if distance > 0.05:
            angle_to_goal = math.atan2(dy, dx)

            #Normalizing angle
            angle_error = math.atan2(
                math.sin(angle_to_goal - self.yaw),
                math.cos(angle_to_goal - self.yaw)
            )
            #Linear PID
            self.linear_integral += 0.5 * (distance + self.prev_distance_error)*dt
            linear_derivative = (distance - self.prev_distance_error) / dt
            linear_u = (self.kp_linear * distance + self.ki_linear * self.linear_integral + self.kd_linear * linear_derivative)

            #Angular PID
            self.angular_integral += 0.5 * (angle_error + self.prev_angular_error)*dt
            angular_derivative = (angle_error - self.prev_angular_error) / dt
            angular_u = (self.kp_angular * angle_error + self.ki_angular * self.angular_integral + self.kd_angular * angular_derivative)
            self.get_logger().info(f"Heading error : {angle_error}")

            self.cmd_vel.angular.z = angular_u
            self.cmd_vel.linear.x = max(min(linear_u, self.linear_speed), 0.0)
            self.cmd_vel.angular.z = max(min(self.cmd_vel.angular.z, 1.5), -1.5)
            #self.cmd_vel.angular.z = self.angular_speed * angle_error

            #Store errors for next iteration
            self.prev_distance_error = distance
            self.prev_angular_error = angle_error

        else:
            self.cmd_vel.linear.x = 0.0
            
            #Normalizing angle
            yaw_error = math.atan2(
                math.sin(gyaw - self.yaw),
                math.cos(gyaw - self.yaw)
            )

            self.get_logger().info(f"Yaw error : {math.radians(yaw_error)}")

            if abs(yaw_error) > 0.087:
                self.cmd_vel.angular.z = self.angular_speed * yaw_error
            else:
                self.cmd_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.cmd_vel)
                self.active = False
                #Reset PID state for next goal
                self.linear_integral = 0.0
                self.angular_integral = 0.0
                self.prev_distance_error = 0.0
                self.prev_angular_error = 0.0
                self.prev_time = None
                self.get_logger().info("Goal reached!")
                return 
            
        self.prev_time = t

        self.cmd_vel_pub.publish(self.cmd_vel)


def main():
    rclpy.init()
    node = GoalService()
    rclpy.spin(node)
    rclpy.shutdown()