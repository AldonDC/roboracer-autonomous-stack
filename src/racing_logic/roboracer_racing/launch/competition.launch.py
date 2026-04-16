from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Argumento para escoger el mundo (por defecto test_world)
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='test_world.sdf',
        description='Archivo de mundo .sdf en roboracer_gazebo'
    )
    
    world_config = LaunchConfiguration('world')

    # 2. Ruta de los paquetes
    gazebo_pkg = get_package_share_directory('roboracer_gazebo')
    racing_pkg = get_package_share_directory('roboracer_racing')

    # 3. Incluir el simulador Gazebo
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'world': world_config}.items()
    )

    # 4. Nodo: Odom TF Broadcaster ( world -> base_link )
    odom_tf = Node(
        package='roboracer_racing',
        executable='odom_tf',
        name='odom_tf_broadcaster',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 5. Nodo: Track Visualizer ( Marker de la pista en RViz )
    track_viz = Node(
        package='roboracer_racing',
        executable='track_viz',
        name='track_visualizer',
        parameters=[{'use_sim_time': True, 'world': world_config}],
        output='screen'
    )

    # 6. Nodo: Telemetry Dashboard ( GUI de telemetría pro - FAST EDITION )
    telemetry = Node(
        package='roboracer_racing',
        executable='telemetry_fast',
        name='telemetry_dashboard',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 7. Nodo: Lane Detector ( Visión artificial )
    lane_detector = Node(
        package='roboracer_racing',
        executable='lane_detector',
        name='lane_detector',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        world_arg,
        simulation,
        odom_tf,
        track_viz,
        telemetry,
        lane_detector
    ])
