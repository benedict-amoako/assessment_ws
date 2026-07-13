from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fortress_tut',
            executable='robot',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('fortress_tut'),
                    'config',
                    'robot.yaml'
                ])
            ]
        )
    ])