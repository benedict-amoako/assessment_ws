from setuptools import find_packages
from setuptools import setup

package_name = 'turtlebot3_example'

setup(
    name=package_name,
    version='2.3.7',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author=['Louis',],
    author_email=[''],
    description=(
        'Examples of Different TurtleBot3 Usage.'
    ),
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            "turtlebot3_goal_service = turtlebot3_example.turtlebot3_goal_service:main",
        ],
    },
)
