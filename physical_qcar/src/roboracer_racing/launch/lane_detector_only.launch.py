"""
Lane Detector Only — sin Pure Pursuit
======================================
Arranca solo la percepcion (camara + lane_detector + lidar_processor).
El lane_follower_pp NO se inicia — arrancar aparte cuando estes listo:

    ros2 run roboracer_racing lane_follower_pp

Uso:
    ros2 launch roboracer_racing lane_detector_only.launch.py
    ros2 launch roboracer_racing lane_detector_only.launch.py publish_debug:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('image_topic',   default_value='/qcar/csi_front'),
        DeclareLaunchArgument('scan_topic',    default_value='/qcar/scan'),
        DeclareLaunchArgument('publish_debug', default_value='true'),
        DeclareLaunchArgument('lane_lateral_bias', default_value='0.0'),
        DeclareLaunchArgument('lane_half_width_px', default_value='75.0'),
        DeclareLaunchArgument('hough_threshold',    default_value='12'),
        DeclareLaunchArgument('min_line_length',    default_value='18'),
        DeclareLaunchArgument('max_line_gap',       default_value='35'),
        DeclareLaunchArgument('debug_draw_hough_candidates', default_value='false'),
    ]

    lane_detector = Node(
        package='roboracer_racing',
        node_executable='lane_detector',
        node_name='lane_detector',
        output='screen',
        parameters=[{
            'image_topic':                  LaunchConfiguration('image_topic'),
            'publish_debug':                LaunchConfiguration('publish_debug'),
            'lane_lateral_bias':            LaunchConfiguration('lane_lateral_bias'),
            'lane_half_width_px':           LaunchConfiguration('lane_half_width_px'),
            'hough_threshold':              LaunchConfiguration('hough_threshold'),
            'min_line_length':              LaunchConfiguration('min_line_length'),
            'max_line_gap':                 LaunchConfiguration('max_line_gap'),
            'debug_draw_hough_candidates':  LaunchConfiguration('debug_draw_hough_candidates'),
        }],
    )

    lidar_processor = Node(
        package='roboracer_racing',
        node_executable='lidar_processor',
        node_name='lidar_processor',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
        }],
    )

    return LaunchDescription(args + [lane_detector, lidar_processor])
