import sys

import rclpy
from rclpy.node import Node
from fortress_tut_msgs.srv import MoveTurtlebot


class CommanderNode(Node):

    def __init__(self):
        super().__init__("commander")
        self.get_logger().info("starting commander node")

        self.move_client = self.create_client(MoveTurtlebot, "move_turtlebot3")
        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('move_turtlebot3 service not available, waiting again...')
        self.request = MoveTurtlebot.Request()

    def send_request(self, x, y, yaw) -> rclpy.Future:
        self.request.x = x
        self.request.y = y
        self.request.yaw = yaw
        return self.move_client.call_async(self.request)


def main():
    rclpy.init()
    node = CommanderNode()
    future = node.send_request(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    rclpy.spin_until_future_complete(node, future)
    response: MoveTurtlebot.Response = future.result()
    node.get_logger().info(f"Success: {response.success}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()