from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'roboracer_racing'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        (os.path.join('share', package_name, 'routes'), glob('routes/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alfonso D.',
    maintainer_email='alfonso@tec.mx',
    description='RoboRacer autonomous racing stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'pure_pursuit = roboracer_racing.pure_pursuit_node:main',
            'waypoint_recorder = roboracer_racing.waypoint_recorder:main',
            'keyboard_teleop = roboracer_racing.keyboard_teleop:main',
            'goal_navigator = roboracer_racing.goal_navigator:main',
            'multi_goal = roboracer_racing.multi_goal_navigator:main',
            'odom_tf = roboracer_racing.odom_tf_broadcaster:main',
            'track_viz = roboracer_racing.track_visualizer:main',
            'planner_gui = roboracer_racing.waypoint_planner_gui:main',
            'telemetry = roboracer_racing.telemetry_dashboard:main',
            'telemetry_fast = roboracer_racing.telemetry_dashboard_fast:main',
            'lane_detector = roboracer_racing.lane_detector:main',
        ],
    },
)
