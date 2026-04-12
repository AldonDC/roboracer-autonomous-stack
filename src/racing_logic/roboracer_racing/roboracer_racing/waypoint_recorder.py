"""
Waypoint Recorder — Graba waypoints mientras manejas el QCar manualmente.

Uso:
  1. Lanza la simulación: ./scripts/launch_sim.sh
  2. En otra terminal: ros2 run roboracer_racing waypoint_recorder
  3. Mueve el carro con joystick o con ros2 topic pub
  4. Cuando termines, presiona Ctrl+C y se guarda el archivo .json

El archivo guardado se usa después con el nodo pure_pursuit.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import os
import math
from datetime import datetime

class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        self.declare_parameter('output_dir', '')
        self.declare_parameter('min_distance', 0.15)  # metros entre waypoints
        self.declare_parameter('odom_topic', '/qcar_sim/odom')

        odom_topic = self.get_parameter('odom_topic').value
        self.min_dist = self.get_parameter('min_distance').value
        self.output_dir = self.get_parameter('output_dir').value

        self.waypoints = []
        self.last_x = None
        self.last_y = None

        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10)

        self.get_logger().info(f'🔴 RECORDING waypoints from [{odom_topic}]')
        self.get_logger().info(f'   Min distance between points: {self.min_dist}m')
        self.get_logger().info(f'   Drive the car around the track, then Ctrl+C to save.')

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extraer yaw del cuaternión
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.last_x is not None:
            dist = math.hypot(x - self.last_x, y - self.last_y)
            if dist < self.min_dist:
                return

        self.waypoints.append({'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 4)})
        self.last_x, self.last_y = x, y

        if len(self.waypoints) % 10 == 0:
            self.get_logger().info(f'   📍 {len(self.waypoints)} waypoints recorded')

    def save_route(self):
        if not self.waypoints:
            self.get_logger().warn('No waypoints recorded!')
            return

        if self.output_dir:
            out_dir = self.output_dir
        else:
            # Guardar junto al paquete
            out_dir = os.path.expanduser('~/Documents/Assesment-Auto/src/racing_logic/roboracer_racing/routes')

        os.makedirs(out_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(out_dir, f'route_{timestamp}.json')

        route_data = {
            'name': f'Route {timestamp}',
            'total_points': len(self.waypoints),
            'waypoints': self.waypoints
        }

        with open(filepath, 'w') as f:
            json.dump(route_data, f, indent=2)

        self.get_logger().info(f'✅ SAVED {len(self.waypoints)} waypoints → {filepath}')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Guardar ruta y limpiar (fuera del try para evitar doble shutdown)
    node.save_route()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

if __name__ == '__main__':
    main()
