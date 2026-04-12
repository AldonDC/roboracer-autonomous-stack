"""
Pure Pursuit Controller — Sigue una ruta pregrabada a máxima velocidad.

Uso:
  ros2 run roboracer_racing pure_pursuit --ros-args \
    -p route_file:=/path/to/route.json \
    -p behavior_style:=RACING

Estilos:
  CONSERVATIVE — Seguro y lento (para probar)
  BALANCED     — Balance velocidad/seguridad
  RACING       — Modo competencia, máxima velocidad
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import math
import numpy as np
import json
import os


def normalize_angle(angle):
    """Normaliza un ángulo al rango [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit')

        # --- PARÁMETROS ---
        self.declare_parameter('route_file', '')
        self.declare_parameter('behavior_style', 'BALANCED')
        self.declare_parameter('v_ref', 0.8)
        self.declare_parameter('odom_topic', '/qcar_sim/odom')
        self.declare_parameter('cmd_topic', '/qcar_sim/user_command')
        self.declare_parameter('scan_topic', '/qcar_sim/scan')
        self.declare_parameter('loop', True)  # Repetir la ruta en circuito

        # Parámetros del vehículo
        self.L = 0.256  # Wheelbase del QCar [m]
        self.max_steer = 0.523  # Max steering angle [rad] (~30 deg)

        # Estilos de conducción: (agresividad, escala_velocidad, base_seguridad)
        self.styles = {
            'CONSERVATIVE': (0.4, 0.5, 1.4),
            'BALANCED':     (0.7, 0.8, 1.0),
            'RACING':       (1.0, 1.2, 0.7),
        }

        style_name = self.get_parameter('behavior_style').value
        self.embedding = self.styles.get(style_name, self.styles['BALANCED'])
        self.v_ref = self.get_parameter('v_ref').value
        self.loop = self.get_parameter('loop').value

        # --- ESTADO ---
        self.cx, self.cy, self.cyaw = [], [], []
        self.target_ind = 0
        self.current_v = 0.0
        self.route_active = False
        self.obstacle_detected = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.laps_completed = 0

        # --- PUBLISHERS ---
        cmd_topic = self.get_parameter('cmd_topic').value
        self.cmd_pub = self.create_publisher(Vector3Stamped, cmd_topic, 10)
        self.path_pub = self.create_publisher(Path, '/viz/racing_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/viz/racing_hud', 10)

        # --- SUBSCRIBERS ---
        odom_topic = self.get_parameter('odom_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        # --- CARGAR RUTA ---
        route_file = self.get_parameter('route_file').value
        if route_file:
            self.load_route(route_file)

        # --- LOOP DE CONTROL a 50Hz ---
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(f'🏎️ PURE PURSUIT ACTIVO | Estilo: {style_name} | v_ref: {self.v_ref}')
        if not route_file:
            self.get_logger().warn('⚠️  No route file specified! Use: -p route_file:=/path/to/route.json')

    def load_route(self, filepath):
        """Carga waypoints desde un archivo JSON."""
        if not os.path.exists(filepath):
            self.get_logger().error(f'❌ Route file not found: {filepath}')
            return

        with open(filepath, 'r') as f:
            data = json.load(f)

        self.cx = [wp['x'] for wp in data['waypoints']]
        self.cy = [wp['y'] for wp in data['waypoints']]
        self.cyaw = [wp.get('yaw', 0.0) for wp in data['waypoints']]

        self.target_ind = 0
        self.route_active = True

        self.get_logger().info(f'✅ Loaded {len(self.cx)} waypoints from {os.path.basename(filepath)}')

        # Publicar la ruta en RViz
        self.publish_path_viz()

    def publish_path_viz(self):
        """Publica la ruta como Path para visualización en RViz."""
        msg = Path()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in zip(self.cx, self.cy):
            p = PoseStamped()
            p.header = msg.header
            p.pose.position.x = x
            p.pose.position.y = y
            msg.poses.append(p)
        self.path_pub.publish(msg)

    def odom_callback(self, msg):
        """Actualiza la posición actual del vehículo."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        """Detecta obstáculos cercanos con el LiDAR."""
        close_ranges = [r for r in msg.ranges
                        if 0.15 < r < 0.5 and not (math.isnan(r) or math.isinf(r))]
        self.obstacle_detected = len(close_ranges) > 5

    def control_loop(self):
        """Loop principal de Pure Pursuit a 50Hz."""
        if not self.route_active or len(self.cx) < 2:
            return

        x = self.current_x
        y = self.current_y
        yaw = self.current_yaw

        A, V_scale, S_base = self.embedding
        v_target = self.v_ref * V_scale

        # Freno de emergencia si hay obstáculo
        if self.obstacle_detected:
            v_target *= 0.3  # Reducir velocidad drásticamente

        # Rampa de aceleración suave
        accel_rate = 0.015 * A  # Más agresivo = aceleración más rápida
        if self.current_v < v_target:
            self.current_v = min(self.current_v + accel_rate, v_target)
        elif self.current_v > v_target:
            self.current_v = max(self.current_v - 0.03, v_target)  # Frenado más rápido

        # --- PURE PURSUIT ---
        # Lookahead dinámico basado en velocidad
        lookahead = (0.5 * S_base) + (self.current_v * 0.4)
        lookahead = max(lookahead, 0.3)  # mínimo 30cm

        # Buscar el punto objetivo
        best_ind = self.target_ind
        for i in range(self.target_ind, len(self.cx)):
            dist = math.hypot(self.cx[i] - x, self.cy[i] - y)
            if dist > lookahead:
                best_ind = i
                break
        else:
            # Llegamos al final de los waypoints
            if self.loop:
                self.target_ind = 0
                self.laps_completed += 1
                self.get_logger().info(f'🏁 LAP {self.laps_completed} COMPLETED!')
                return
            else:
                # Parar el carro
                self.stop_vehicle()
                self.route_active = False
                self.get_logger().info('🏁 ROUTE COMPLETED! Stopping.')
                return

        self.target_ind = best_ind

        # Ángulo al punto objetivo
        alpha = normalize_angle(
            math.atan2(self.cy[self.target_ind] - y, self.cx[self.target_ind] - x) - yaw
        )

        # Ley de Pure Pursuit
        if abs(lookahead) > 0.01:
            delta = math.atan2(2.0 * self.L * math.sin(alpha), lookahead)
        else:
            delta = 0.0

        # Clamp steering
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))

        # Reducir velocidad en curvas cerradas
        curve_factor = 1.0 - (abs(delta) / self.max_steer) * 0.4
        final_v = self.current_v * curve_factor

        # --- PUBLICAR COMANDO ---
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = float(final_v)
        cmd.vector.y = float(delta)
        self.cmd_pub.publish(cmd)

        # --- HUD EN RVIZ ---
        self.publish_hud(final_v, delta)

    def stop_vehicle(self):
        """Envía comando de parada."""
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = 0.0
        cmd.vector.y = 0.0
        self.cmd_pub.publish(cmd)

    def publish_hud(self, vel, steer):
        """Muestra datos de telemetría en RViz."""
        m = Marker()
        m.header.frame_id = 'base_link'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'racing_hud'
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.z = 0.6
        m.scale.z = 0.1
        style = self.get_parameter('behavior_style').value
        obs = '⚠️ OBS' if self.obstacle_detected else '✅ CLEAR'
        m.text = f'{style} | v={vel:.2f} | δ={math.degrees(steer):.1f}° | Lap:{self.laps_completed} | {obs}'
        m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.3, 1.0
        self.marker_pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_vehicle()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
