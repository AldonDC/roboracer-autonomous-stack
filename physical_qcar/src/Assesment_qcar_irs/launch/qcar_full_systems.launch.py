import ast
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    _csi_nodes = ["csi_right", "csi_left", "csi_front", "csi_back"]
    _rgbd_nodes = ["rgbd_color","rgbd_depth"]

    nodes_to_launch = LaunchConfiguration('nodes').perform(context)
    nodes_list = [node.strip() for node in nodes_to_launch.split(',') if node.strip()]
    csi_nodes = LaunchConfiguration('csi_cameras').perform(context)
    csi_list = [node.strip() for node in csi_nodes.split(',') if (node.strip() and node.strip() in _csi_nodes)]
    rgbd_nodes = LaunchConfiguration('rgbd_cameras').perform(context)
    rgbd_list = [node.strip() for node in rgbd_nodes.split(',') if (node.strip() and node.strip() in _rgbd_nodes)]
    
    print(csi_list, rgbd_list)
    # Define all potential parameters
    parameters = {
        "command": [],
        "qcar": [
            {'publishers': ['imu', 'battery', 'velocity']},
            {'imu_publish_frequency': 100},
            {'battery_publish_frequency': 1},
            {'velocity_publish_frequency': 10}
        ],
        "csi": [
            {'publishers': csi_list},
            {'csi_right_resolution': [820, 410]},
            {'csi_left_resolution': [820, 410]},
            {'csi_front_resolution': [820, 410]},
            {'csi_back_resolution': [820, 410]},
            {'csi_right_freq': 120},
            {'csi_left_freq': 120},
            {'csi_front_freq': 120},
            {'csi_back_freq': 120}
        ],
        "rgbd": [
            {'publishers': rgbd_list},
            {'rgbd_color_resolution': [640, 480]},
            {'rgbd_depth_resolution': [640, 480]},
            {'rgbd_color_freq': 60},
            {'rgbd_depth_freq': 60}
        ],
        "lidar_qos": [
            {'lidar_Number_Samples': 100}
        ],
        "imu_external": [],
    }

    _supported_nodes = ["command", "qcar", "csi", "rgbd", "lidar_qos", "imu_external"]

    # Nodes to launch
    launch_nodes = [
            Node(
                package='qcar',
		#node_namespace='qcar_green',
                node_executable=n,
                node_name=n,
                prefix=['stdbuf -o L'],
                output='screen',
                parameters=parameters.get(n, [])
            ) for n in nodes_list if n in _supported_nodes
    ]

    return launch_nodes
	
	
def generate_launch_description():
    

    return LaunchDescription([
        DeclareLaunchArgument('nodes', default_value='command,qcar,csi,rgbd,lidar_qos,imu_external', description='List of nodes to launch: command, qcar, csi, rgbd, lidar_qos, imu_external. Separate with commas and select as needed.'),
        DeclareLaunchArgument('csi_cameras', default_value='csi_right,csi_left,csi_front,csi_back', description='List of CSI cameras: csi_front, csi_left, csi_right, csi_back. All cameras publish CompressedImage topics. Separate with commas and select as needed.'),
        DeclareLaunchArgument('rgbd_cameras', default_value='rgbd_color,rgbd_depth', description='List of RGBD cameras: rgbd_color, rgbd_depth. Separate with commas and select as needed.'),
        #*launch_nodes
    ]+[OpaqueFunction(function=launch_setup)])
    
