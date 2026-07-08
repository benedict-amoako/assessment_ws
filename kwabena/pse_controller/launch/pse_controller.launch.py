from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    'turtlebot3_gazebo'
                ),
                'launch',
                'empty_world.launch.py'
            )
        ),
        launch_arguments={
            'gui': 'false'
        }.items()
    )

    controller = Node(
        package='pse_controller',
        executable='controller',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        controller
    ])