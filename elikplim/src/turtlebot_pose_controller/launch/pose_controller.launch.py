"""Launch file for the TurtleBot pose controller.

Usage (after building the workspace):

  # Terminal 1 – Gazebo simulation
  export TURTLEBOT3_MODEL=burger
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

  # Terminal 2 – Pose controller
  ros2 launch turtlebot_pose_controller pose_controller.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pose_controller_node = Node(
        package='turtlebot_pose_controller',
        executable='pose_controller',
        name='pose_controller',
        output='screen',
        parameters=[],
    )

    return LaunchDescription([pose_controller_node])
