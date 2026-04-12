"""
Multi-Goal Navigator — Pon waypoints en RViz y el carro los sigue.

Uso en RViz:
  1. Click el botón "Publish Point" (el último de la toolbar)
  2. Click en varios puntos de la pista — se van acumulando
  3. El carro empieza a moverse al primer punto automáticamente
  4. Cuando llega, va al siguiente, y así hasta el último

  Para limpiar los waypoints: publica en /waypoints/clear
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Empty
import math
import numpy as np
import threading
import sys
import json
import os
from datetime import datetime


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class MultiGoalNavigator(Node):
    def __init__(self):
        super().__init__('multi_goal_navigator')

        # Parámetros
        self.declare_parameter('v_ref', 1.0)
        self.declare_parameter('arrival_radius', 0.3)
        self.declare_parameter('odom_topic', '/qcar_sim/odom')
        self.declare_parameter('cmd_topic', '/qcar_sim/user_command')
        self.declare_parameter('loop', False)

        self.v_ref = self.get_parameter('v_ref').value
        self.arrival_radius = self.get_parameter('arrival_radius').value
        self.loop = self.get_parameter('loop').value
        self.L = 0.256
        self.max_steer = 0.5

        # Estado
        self.waypoints = []  # Lista de (x, y)
        self.current_wp_index = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_v = 0.0
        self.navigating = False

        # Publishers
        cmd_topic = self.get_parameter('cmd_topic').value
        self.cmd_pub = self.create_publisher(Vector3Stamped, cmd_topic, 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/viz/waypoints', 10)
        self.path_pub = self.create_publisher(Path, '/viz/waypoint_path', 10)
        self.hud_pub = self.create_publisher(Marker, '/viz/nav_hud', 10)

        # Subscribers
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)

        # Escuchar clicks de "Publish Point" en RViz
        self.point_sub = self.create_subscription(
            PointStamped, '/clicked_point', self.point_cb, 10)

        # Escuchar "2D Goal Pose" también como waypoint
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10)

        # Limpiar waypoints
        self.clear_sub = self.create_subscription(
            Empty, '/waypoints/clear', self.clear_cb, 10)

        # Control loop 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        # Publicar markers cada 0.5s
        self.viz_timer = self.create_timer(0.5, self.publish_viz)

        self.get_logger().info('🗺️  MULTI-GOAL NAVIGATOR LISTO')
        self.get_logger().info('   👆 Usa "Publish Point" en RViz para armar la ruta.')
        self.get_logger().info('   ⌨️  Usa la consola de esta terminal para dar comandos (Go, Clear, Save)')
        
        # Hilo para leer comandos de terminal
        self.cmd_thread = threading.Thread(target=self.terminal_cmd_loop, daemon=True)
        self.cmd_thread.start()

    def terminal_cmd_loop(self):
        print("\n" + "="*50)
        print("🤖 ROBO RACER CLI PLANNER")
        print("1. En RViz haz click en 'Publish Point' para añadir waypoints repetidamente")
        print("2. Escribe comandos aquí abajo:")
        print("   [g] Go! (Inicia el recorrido)")
        print("   [c] Clear (Borra waypoints)")
        print("   [s] Save (Guarda ruta a JSON)")
        print("   [q] Quit")
        print("="*50 + "\n")
        
        while True:
            cmd = input("roboracer> ").strip().lower()
            if cmd == 'g':
                if not self.waypoints:
                    print("⚠️  No hay waypoints para recorrer.")
                    continue
                if self.navigating:
                    print("⚠️  Ya está navegando.")
                    continue
                self.navigating = True
                self.current_wp_index = 0
                self.current_v = 0.0
                print(f"🚀 ¡GO! Recorriendo {len(self.waypoints)} waypoints.")
                self.publish_viz()
            elif cmd == 'c':
                self.clear_cb(None)
            elif cmd == 's':
                self.save_wps()
            elif cmd == 'q':
                self.stop()
                print("👋 Saliendo...")
                os._exit(0)
            else:
                print("❌ Comando no válido.")

    def save_wps(self):
        if not self.waypoints:
            print("⚠️ No hay waypoints para guardar.")
            return
        out_dir = os.path.expanduser('~/Documents/Assesment-Auto/src/racing_logic/roboracer_racing/routes')
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        fp = os.path.join(out_dir, f'route_rviz_{ts}.json')
        data = {
            'name': f'RViz Route {ts}',
            'total_points': len(self.waypoints),
            'waypoints': [{'x': round(x, 4), 'y': round(y, 4), 'yaw': 0.0} for x, y in self.waypoints]
        }
        with open(fp, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"💾 Ruta guardada en: {fp}")

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def point_cb(self, msg):
        """Agrega un waypoint desde "Publish Point"."""
        x, y = msg.point.x, msg.point.y
        self.add_waypoint(x, y)

    def goal_cb(self, msg):
        """Agrega un waypoint desde "2D Goal Pose"."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.add_waypoint(x, y)

    def add_waypoint(self, x, y):
        self.waypoints.append((x, y))
        idx = len(self.waypoints)
        self.get_logger().info(
            f'📍 Waypoint #{idx}: ({x:.2f}, {y:.2f}) | '
            f'Total: {len(self.waypoints)} waypoints'
        )

        self.publish_viz()

    def clear_cb(self, msg):
        """Limpia todos los waypoints y para el carro."""
        self.waypoints.clear()
        self.current_wp_index = 0
        self.navigating = False
        self.current_v = 0.0
        self.stop()

        # Limpiar markers
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = 'world'
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.marker_array_pub.publish(ma)

        self.get_logger().info('🧹 Waypoints LIMPIADOS')

    def publish_viz(self):
        """Publica los waypoints como markers numerados y el path."""
        if not self.waypoints:
            return

        # MarkerArray con esferas numeradas
        ma = MarkerArray()
        for i, (wx, wy) in enumerate(self.waypoints):
            # Esfera
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'waypoint_spheres'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.05
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.1

            if i < self.current_wp_index:
                # Visitado — gris
                m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.5, 0.5, 0.5
            elif i == self.current_wp_index:
                # Activo — verde brillante
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.3, 1.0
            else:
                # Pendiente — amarillo
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.9, 0.0, 0.9
            ma.markers.append(m)

            # Número
            mt = Marker()
            mt.header.frame_id = 'world'
            mt.header.stamp = self.get_clock().now().to_msg()
            mt.ns = 'waypoint_labels'
            mt.id = i + 1000
            mt.type = Marker.TEXT_VIEW_FACING
            mt.action = Marker.ADD
            mt.pose.position.x = wx
            mt.pose.position.y = wy
            mt.pose.position.z = 0.3
            mt.scale.z = 0.2
            mt.text = str(i + 1)
            mt.color.r, mt.color.g, mt.color.b, mt.color.a = 1.0, 1.0, 1.0, 1.0
            ma.markers.append(mt)

        self.marker_array_pub.publish(ma)

        # Path entre waypoints
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in self.waypoints:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = wx
            p.pose.position.y = wy
            path.poses.append(p)
        self.path_pub.publish(path)

    def control_loop(self):
        if not self.navigating or not self.waypoints:
            return

        if self.current_wp_index >= len(self.waypoints):
            if self.loop:
                self.current_wp_index = 0
                self.get_logger().info('🔄 LOOP — reiniciando ruta')
            else:
                self.stop()
                self.navigating = False
                self.get_logger().info('🏁 ¡TODOS LOS WAYPOINTS COMPLETADOS!')
                return

        # Objetivo actual
        goal_x, goal_y = self.waypoints[self.current_wp_index]
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        dist = math.hypot(dx, dy)

        # ¿Llegamos a este waypoint?
        if dist < self.arrival_radius:
            self.get_logger().info(
                f'✅ Waypoint #{self.current_wp_index + 1} alcanzado! '
                f'({goal_x:.2f}, {goal_y:.2f})'
            )
            self.current_wp_index += 1
            self.publish_viz()
            return

        # Velocidad adaptativa
        remaining = len(self.waypoints) - self.current_wp_index
        if dist > 1.0:
            v_target = self.v_ref
        elif dist > 0.5:
            v_target = self.v_ref * 0.7
        else:
            v_target = max(0.2, self.v_ref * dist * 0.8)

        # Rampa suave
        if self.current_v < v_target:
            self.current_v = min(self.current_v + 0.02, v_target)
        else:
            self.current_v = max(self.current_v - 0.04, v_target)

        # Pure Pursuit
        alpha = normalize_angle(math.atan2(dy, dx) - self.current_yaw)
        lookahead = max(dist, 0.3)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), lookahead)
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))

        # Reducir velocidad en curvas
        curve_factor = 1.0 - (abs(delta) / self.max_steer) * 0.4
        final_v = self.current_v * curve_factor

        # Publicar comando
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = float(final_v)
        cmd.vector.y = float(delta)
        self.cmd_pub.publish(cmd)

        # HUD
        hud = Marker()
        hud.header.frame_id = 'base_link'
        hud.header.stamp = self.get_clock().now().to_msg()
        hud.ns = 'nav_hud'
        hud.id = 0
        hud.type = Marker.TEXT_VIEW_FACING
        hud.action = Marker.ADD
        hud.pose.position.z = 0.6
        hud.scale.z = 0.1
        hud.text = (f'WP {self.current_wp_index + 1}/{len(self.waypoints)} | '
                    f'dist: {dist:.2f}m | v: {final_v:.2f}')
        hud.color.r, hud.color.g, hud.color.b, hud.color.a = 0.0, 1.0, 0.5, 1.0
        self.hud_pub.publish(hud)

    def stop(self):
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = 0.0
        cmd.vector.y = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
