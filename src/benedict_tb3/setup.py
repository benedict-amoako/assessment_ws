import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'benedict_tb3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benedict',
    maintainer_email='benedict.amoako@fortressaisolutions.com',
    description='Closed-loop TurtleBot3 pose controller (Siegwart polar control law) with a custom SetGoalPose service.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'tb3_move_to_goal_node = benedict_tb3.tb3_pose_controller:main',
            'tb3_pose_client = benedict_tb3.tb3_pose_client:main',
        ],
    },
)
