import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from pse_controller.extraFn import quaternion_to_yaw

class OdomListener(Node):

    def __init__(self):
        super().__init__("odom_listener")

        self.subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        # Target position
        self.goal_x = 2.0
        self.goal_y = 1.0

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        distance = math.sqrt(
            (self.goal_x - x) ** 2 +
            (self.goal_y - y) ** 2
        )

        heading = math.atan2(
            self.goal_y - y,
            self.goal_x - x
        )
        
        self.get_logger().info(
            f"""Current Position : ({x:.2f}, {y:.2f})
                Goal Position    : ({self.goal_x:.2f}, {self.goal_y:.2f})
                Distance to Goal : {distance:.2f}
                Heading = {math.degrees(heading):.2f}
                Current Yaw = {math.degrees(yaw):.2f}
        """       )


def main(args=None):

    rclpy.init(args=args)

    node = OdomListener()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()