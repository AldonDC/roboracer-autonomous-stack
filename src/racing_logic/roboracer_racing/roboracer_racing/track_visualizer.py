"""
Track Visualizer — Publica el mesh de la pista como un Marker en RViz.

Esto permite ver la pista de Gazebo directamente en RViz para
colocar goals con precisión.
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory
import os


class TrackVisualizer(Node):
    def __init__(self):
        super().__init__('track_visualizer')

        self.marker_pub = self.create_publisher(Marker, '/viz/track', 10)

        # Declarar parámetro de mundo (default a test_world para no romper nada)
        self.declare_parameter('world', 'test_world.sdf')
        world_name = self.get_parameter('world').get_parameter_value().string_value
        
        # Publicar cada 2 segundos (latched marker)
        self.timer = self.create_timer(2.0, self.publish_track)

        pkg_dir = get_package_share_directory('roboracer_gazebo')
        self.walls = []

        if 'oschersleben' in world_name.lower():
            self.get_logger().info(f'🏎️  ROBORACER VIZ: Cambiando a modo OSCHERSLEBEN PRO')
            self.track_mesh_path = os.path.join(pkg_dir, 'models', 'oschersleben_pro', 'meshes', 'track.obj')
            # En Oschersleben la posición es 0,0
            self.track_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        else:
            self.get_logger().info(f'🧪 ROBORACER VIZ: Modo Entorno de Pruebas')
            self.track_mesh_path = os.path.join(pkg_dir, 'models', 'track', 'meshes', 'qcar_track.obj')
            self.track_pose = {'x': -0.0036, 'y': 0.0002, 'z': 0.0012}
            
            # Solo añadir paredes en mundos de pruebas
            short_wall_mesh = os.path.join(pkg_dir, 'models', 'wall_4_8x0_25', 'meshes', 'wood_wall_short.obj')
            long_wall_mesh = os.path.join(pkg_dir, 'models', 'wall_6_1x0_25', 'meshes', 'wood_wall_long.obj')
            
            if os.path.exists(short_wall_mesh):
                self.walls.append({'mesh': short_wall_mesh, 'x': 0.0, 'y': -3.05, 'z': 0.125, 'yaw': 0.0})
                self.walls.append({'mesh': short_wall_mesh, 'x': 0.0, 'y': 3.05, 'z': 0.125, 'yaw': 0.0})
            if os.path.exists(long_wall_mesh):
                self.walls.append({'mesh': long_wall_mesh, 'x': 2.4, 'y': 0.0, 'z': 0.125, 'yaw': 1.5708})
                self.walls.append({'mesh': long_wall_mesh, 'x': -2.4, 'y': 0.0, 'z': 0.125, 'yaw': 1.5708})

        if os.path.exists(self.track_mesh_path):
            self.get_logger().info(f'🏁 Track mesh found: {self.track_mesh_path}')
        else:
            self.get_logger().error(f'❌ Track mesh NOT found: {self.track_mesh_path}')

        # Visualizador de paredes
        self.wall_pub = self.create_publisher(Marker, '/viz/walls', 10)
        self.wall_timer = self.create_timer(2.0, self.publish_walls)

    def publish_track(self):
        if not os.path.exists(self.track_mesh_path):
            return

        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'track'
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.action = Marker.ADD
        m.mesh_resource = f'file://{self.track_mesh_path}'
        m.mesh_use_embedded_materials = True

        # Posición del track dinámica
        m.pose.position.x = self.track_pose['x']
        m.pose.position.y = self.track_pose['y']
        m.pose.position.z = self.track_pose['z']
        m.pose.orientation.w = 1.0

        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0

        # Color fallback si no tiene materiales
        m.color.r = 0.3
        m.color.g = 0.3
        m.color.b = 0.3
        m.color.a = 0.8

        self.marker_pub.publish(m)

    def publish_walls(self):
        import math
        for i, wall in enumerate(self.walls):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'walls'
            m.id = i
            m.type = Marker.MESH_RESOURCE
            m.action = Marker.ADD
            m.mesh_resource = f'file://{wall["mesh"]}'
            m.mesh_use_embedded_materials = True

            m.pose.position.x = wall['x']
            m.pose.position.y = wall['y']
            m.pose.position.z = wall['z']

            # Convertir yaw a quaternion
            yaw = wall['yaw']
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)

            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0

            m.color.r = 0.6
            m.color.g = 0.4
            m.color.b = 0.2
            m.color.a = 0.9

            self.wall_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = TrackVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
