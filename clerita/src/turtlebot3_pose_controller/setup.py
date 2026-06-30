from setuptools import find_packages, setup


package_name = 'turtlebot3_pose_controller'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/turtlebot3_pose_controller']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/srv', ['srv/SetPose.srv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='clerita',
    maintainer_email='clerita@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'pose_controller = turtlebot3_pose_controller.pose_controller:main',
        ],
    },
)