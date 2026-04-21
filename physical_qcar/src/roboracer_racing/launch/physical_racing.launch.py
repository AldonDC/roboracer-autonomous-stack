from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Paquetes
    qcar_pkg = 'qcar'
    racing_pkg = 'roboracer_racing'

    # 1. HARDWARE QCAR
    qcar_node = Node(
        package=qcar_pkg,
        node_executable='qcar',
        node_name='qcar_node',
        output='screen',
        parameters=[{
            'publishers': ['imu', 'velocity', 'battery'],
            'velocity_publish_frequency': 30
        }]
    )

    # 2. LIDAR
    lidar_node = Node(
        package=qcar_pkg,
        node_executable='lidar_qos',
        node_name='lidar_node',
        output='screen',
        parameters=[{'lidar_Number_Samples': 100}]
    )

    # 3. CSI CAMERA
    csi_node = Node(
        package=qcar_pkg,
        node_executable='csi',
        node_name='csi_node',
        output='screen',
        parameters=[{
            'publishers': ['csi_front'],
            'csi_front_resolution': [640, 480],
            'csi_front_freq': 30
        }]
    )

    # 4. STATIC TF (Importante para que el coche aparezca en el mapa)
    tf_node = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        node_name='static_tf_pub',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'odom']
    )

    return LaunchDescription([
        qcar_node,
        lidar_node,
        csi_node,
        tf_node
    ])
