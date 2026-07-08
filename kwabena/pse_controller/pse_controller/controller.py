import math
from pse_controller_interfaces.srv import MoveToPose
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pse_controller.extraFn import normalize_angle, quaternion_to_yaw
from visualization_msgs.msg import Marker


class PoseController(Node):

    def __init__(self):

        super().__init__("pose_controller")

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Goal pose
        # self.goal_x = 2.0
        # self.goal_y = 5.0
        self.goal_active = False
        self.goal_theta = 0.0

        # Subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        # Publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            "/goal_marker",
            10  )
        

        self.move_service = self.create_service(
            MoveToPose,
            "move_to_pose",
            self.move_to_pose_callback
            )
        # Run the controller at 10 Hz
        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )
        # Controller gains
        self.k_linear = 0.5
        self.k_angular = 2.0

        # Goal tolerance
        self.position_tolerance = 0.05
        self.orientation_tolerance = 0.05

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(
            msg.pose.pose.orientation
        )

    def compute_errors(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        distance_error = math.hypot(dx, dy)
        desired_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(desired_heading - self.yaw)
        orientation_error = normalize_angle(self.goal_theta - self.yaw)
        return distance_error, heading_error,orientation_error
    
    # def goal_reached(self, distance_error):
    #     return distance_error < self.position_tolerance
    
    def compute_velocity( self,  distance_error,  heading_error ):
        cmd = Twist()
        cmd.linear.x = self.k_linear * distance_error
        cmd.angular.z = self.k_angular * heading_error
        cmd.linear.x = min(cmd.linear.x, 0.25)
        cmd.angular.z = max(min(cmd.angular.z, 1.5), -1.5 )
        return cmd
    
    def publish_velocity(self, cmd):
        self.cmd_pub.publish(cmd)

    def publish_goal_marker(self):
     marker = Marker()
     marker.header.frame_id = "odom"
     marker.header.stamp = self.get_clock().now().to_msg()
     marker.ns = "goal"
     marker.id = 0
     marker.type = Marker.SPHERE
     marker.action = Marker.ADD
    # Position
     marker.pose.position.x = self.goal_x
     marker.pose.position.y = self.goal_y
     marker.pose.position.z = 0.05
    # Orientation (not important for a sphere)
     marker.pose.orientation.w = 1.0
    # Sphere size
     marker.scale.x = 0.20
     marker.scale.y = 0.20
     marker.scale.z = 0.20
    # Green color
     marker.color.r = 0.0
     marker.color.g = 1.0
     marker.color.b = 0.0
     marker.color.a = 1.0
     self.marker_pub.publish(marker)

    def control_loop(self):
        if not self.goal_active:
            return
        distance_error, heading_error, orientation_error = self.compute_errors()
        if distance_error > self.position_tolerance:
            cmd = self.compute_velocity(
                distance_error,
                heading_error
        )
        elif abs(orientation_error) > self.orientation_tolerance:
            cmd = Twist()
            cmd.angular.z = self.k_angular * orientation_error
            cmd.angular.z = max(
                min(cmd.angular.z, 1.0),
                -1.0
            )
        else: 
            cmd = Twist()
            self.goal_active = False

            self.get_logger().info(
        "========== GOAL REACHED =========="
        )

            self.get_logger().info(
                f"Target Pose : "
                f"x={self.goal_x:.2f} m, "
                f"y={self.goal_y:.2f} m, "
                f"θ={math.degrees(self.goal_theta):.1f}°"
             )

            self.get_logger().info(
                f"Actual Pose : "
                f"x={self.x:.2f} m, "
                f"y={self.y:.2f} m, "
                f"θ={math.degrees(self.yaw):.1f}°"
          )
        self.publish_velocity(cmd)

    def move_to_pose_callback(self, request, response):
        self.goal_active = True
        self.goal_x = request.x
        self.goal_y = request.y
        self.goal_theta = math.radians(request.theta)
        self.publish_goal_marker()
        response.success = True
        response.message = "Goal accepted."
        self.get_logger().info(
            f"New goal: ({request.x:.2f}, "
            f"{request.y:.2f}, "
            f"{request.theta:.2f})"
        )
        return response

def main(args=None):

    rclpy.init(args=args)

    node = PoseController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()