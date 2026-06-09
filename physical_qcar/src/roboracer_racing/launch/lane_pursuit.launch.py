"""
Lane Pursuit Launch
===================
Arranca la pipeline completa de lane following con Pure Pursuit:

  camara CSI ──► lane_detector ──► /lane_target_point_m ──┐
                                   /lane/center_offset    │
                                   /lane/confidence       │
                                                          ▼
                  /qcar/scan ──► lidar_processor ──► lane_follower_pp
                                  /lidar/min_distance      │
                                  /lidar/obstacle          ▼
                                                  /qcar/user_command

El physical_dashboard.py se levanta APARTE en otra terminal:
    python3 src/racing_logic/roboracer_racing/roboracer_racing/physical_dashboard.py

(no se incluye aqui porque suele requerir ventana interactiva propia).

Uso:
    ros2 launch roboracer_racing lane_pursuit.launch.py
    ros2 launch roboracer_racing lane_pursuit.launch.py speed_straight:=0.20
    ros2 launch roboracer_racing lane_pursuit.launch.py launch_dashboard:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Args expuestos ────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('platform',            default_value='qcar'),
        DeclareLaunchArgument('image_topic',         default_value='/qcar/csi_front'),
        DeclareLaunchArgument('scan_topic',          default_value='/qcar/scan'),
        DeclareLaunchArgument('cmd_topic',           default_value='/qcar/user_command'),

        # Pure Pursuit tuning
        DeclareLaunchArgument('speed_straight',      default_value='0.15'),
        DeclareLaunchArgument('speed_curve',         default_value='0.0775'),
        DeclareLaunchArgument('max_steering_angle',  default_value='0.30'),
        DeclareLaunchArgument('steering_sign',       default_value='-1.0'),
        DeclareLaunchArgument('curve_threshold',     default_value='0.20'),

        # Safety LiDAR
        DeclareLaunchArgument('use_lidar_safety',    default_value='true'),
        DeclareLaunchArgument('lidar_stop_distance', default_value='0.6'),
        DeclareLaunchArgument('lidar_slow_distance', default_value='1.5'),

        # Lane detector
        DeclareLaunchArgument('publish_debug',       default_value='true'),
        DeclareLaunchArgument('lane_lateral_bias',   default_value='0.0'),

        # Dashboard opcional (off por defecto: lo arrancas tu en otra terminal)
        DeclareLaunchArgument('launch_dashboard',    default_value='false'),
    ]

    # ── Nodo: Lane Detector (camara → target point en metros) ────────────────
    lane_detector = Node(
        package='roboracer_racing',
        node_executable='lane_detector',
        node_name='lane_detector',
        output='screen',
        parameters=[{
            'image_topic':       LaunchConfiguration('image_topic'),
            'publish_debug':     LaunchConfiguration('publish_debug'),
            'lane_lateral_bias': LaunchConfiguration('lane_lateral_bias'),
        }],
    )

    # ── Nodo: LiDAR Processor (scan → min_distance / obstacle) ───────────────
    lidar_processor = Node(
        package='roboracer_racing',
        node_executable='lidar_processor',
        node_name='lidar_processor',
        output='screen',
        parameters=[{
            'scan_topic': LaunchConfiguration('scan_topic'),
        }],
    )

    # ── Nodo: Lane Follower Pure Pursuit (target + lidar → cmd) ──────────────
    lane_follower_pp = Node(
        package='roboracer_racing',
        node_executable='lane_follower_pp',
        node_name='lane_follower_pp',
        output='screen',
        parameters=[{
            'platform':            LaunchConfiguration('platform'),
            'cmd_topic':           LaunchConfiguration('cmd_topic'),
            'speed_straight':      LaunchConfiguration('speed_straight'),
            'speed_curve':         LaunchConfiguration('speed_curve'),
            'max_steering_angle':  LaunchConfiguration('max_steering_angle'),
            'steering_sign':       LaunchConfiguration('steering_sign'),
            'curve_threshold':     LaunchConfiguration('curve_threshold'),
            'use_lidar_safety':    LaunchConfiguration('use_lidar_safety'),
            'lidar_stop_distance': LaunchConfiguration('lidar_stop_distance'),
            'lidar_slow_distance': LaunchConfiguration('lidar_slow_distance'),
        }],
    )

    # ── Dashboard opcional (Tk/Qt: muchas veces mejor lanzarlo aparte) ───────
    dashboard = ExecuteProcess(
        cmd=['ros2', 'run', 'roboracer_racing', 'physical_dashboard'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_dashboard')),
    )

    return LaunchDescription(args + [
        lane_detector,
        lidar_processor,
        lane_follower_pp,
        dashboard,
    ])
