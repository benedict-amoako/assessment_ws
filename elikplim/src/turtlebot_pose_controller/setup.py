from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot_pose_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'srv'),
            glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Candidate',
    maintainer_email='ghnewton10@gmail.com',
    description='Proportional pose controller for TurtleBot3 using ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pose_controller = turtlebot_pose_controller.pose_controller_node:main',
        ],
    },
)
