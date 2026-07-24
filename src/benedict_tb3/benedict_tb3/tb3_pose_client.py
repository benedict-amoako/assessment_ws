"""CLI client for /set_goal_pose:  ros2 run benedict_tb3 tb3_pose_client X Y YAW_DEG"""

import sys

import rclpy
from rclpy.node import Node

from benedict_msgs.srv import SetGoalPose


class PoseClient(Node):
    """One-shot client for the /set_goal_pose service.

    TB3PoseController's service is fire-and-forget: it accepts a goal and
    returns immediately rather than blocking until the robot arrives, so a
    successful response here only means the goal was accepted, not reached.
    Watch the controller's own log (or /odom) to see when it actually gets
    there.
    """

    def __init__(self):
        super().__init__('tb3_pose_client')
        self._client = self.create_client(SetGoalPose, 'set_goal_pose')

    def send(self, x: float, y: float, yaw_deg: float):
        if not self._client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('set_goal_pose service not available.')
            return None
        request = SetGoalPose.Request()
        request.x = x
        request.y = y
        request.yaw = yaw_deg
        self.get_logger().info(
            f'Requesting pose: x={x:.2f} m, y={y:.2f} m, yaw={yaw_deg:.1f} deg ...'
        )
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    """Entry point: ros2 run benedict_tb3 tb3_pose_client X Y YAW_DEG"""
    if len(sys.argv) != 4:
        print('Usage: ros2 run benedict_tb3 tb3_pose_client X Y YAW_DEG')
        sys.exit(1)
    x, y, yaw = (float(v) for v in sys.argv[1:4])

    rclpy.init(args=args)
    node = PoseClient()
    result = node.send(x, y, yaw)

    if result is None:
        print('No response received -- is the controller node running?')
    elif result.success:
        print(f'Goal accepted: x={x:.2f} m, y={y:.2f} m, yaw={yaw:.1f} deg')
        print("The service returns as soon as the goal is accepted -- watch "
              "the controller's log (or /odom) to see when it actually "
              "arrives.")
    else:
        print('Goal rejected: a goal is likely already in progress.')

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
