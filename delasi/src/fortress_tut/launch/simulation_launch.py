#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('fortress_tut'), 'launch')
    turtlebot_gz_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    gz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot_gz_dir, 'empty_world.launch.py')
        )
    )

    robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'robot_launch.py')
        )
    )

    ld = LaunchDescription()

    ld.add_action(gz_cmd)
    ld.add_action(robot_cmd)

    return ld
