"""
Waypoint Planner GUI — Mapa interactivo para poner waypoints.

Click izquierdo = Agregar waypoint
Click derecho   = Borrar último waypoint
Tecla Enter     = Iniciar navegación
Tecla C         = Limpiar todos
Tecla S         = Guardar ruta a JSON
Tecla Q         = Salir

Se abre una ventana con vista top-down de la pista.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, PoseStamped, Vector3Stamped
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Empty
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.patches import FancyArrowPatch
import numpy as np
import math
import threading
import json
import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
import matplotlib.image as mpimg


class WaypointPlannerGUI(Node):
    def __init__(self):
        super().__init__('waypoint_planner_gui')

        self.declare_parameter('odom_topic', '/qcar_sim/odom')
        self.declare_parameter('cmd_topic', '/qcar_sim/user_command')
        self.declare_parameter('v_ref', 1.0)
        self.declare_parameter('arrival_radius', 0.3)

        self.v_ref = self.get_parameter('v_ref').value
        self.arrival_radius = self.get_parameter('arrival_radius').value
        self.L = 0.256
        self.max_steer = 0.5

        # Estado del carro
        self.car_x = 0.0
        self.car_y = 0.0
        self.car_yaw = 0.0

        # Waypoints
        self.waypoints = []
        self.current_wp_index = 0
        self.navigating = False
        self.current_v = 0.0

        # LiDAR map accumulation
        self.scan_points_x = []
        self.scan_points_y = []
        self.max_scan_points = 8000  # Máximo puntos acumulados

        # Trail del carro
        self.trail_x = []
        self.trail_y = []
        self.max_trail = 500

        # ROS
        odom_topic = self.get_parameter('odom_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/qcar_sim/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Vector3Stamped, cmd_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/viz/waypoints', 10)
        self.path_pub = self.create_publisher(Path, '/viz/waypoint_path', 10)

        # Control loop
        self.control_timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('🗺️  WAYPOINT PLANNER GUI iniciando...')
        self.get_logger().info('   📡 Acumulando mapa LiDAR en tiempo real...')

    def odom_cb(self, msg):
        self.car_x = msg.pose.pose.position.x
        self.car_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.car_yaw = math.atan2(siny, cosy)

        # Trail
        if (not self.trail_x or
                math.hypot(self.car_x - self.trail_x[-1],
                           self.car_y - self.trail_y[-1]) > 0.05):
            self.trail_x.append(self.car_x)
            self.trail_y.append(self.car_y)
            if len(self.trail_x) > self.max_trail:
                self.trail_x.pop(0)
                self.trail_y.pop(0)

    def scan_cb(self, msg):
        """Transforma puntos LiDAR a coordenadas world y los acumula."""
        angle = msg.angle_min
        cos_yaw = math.cos(self.car_yaw)
        sin_yaw = math.sin(self.car_yaw)

        for r in msg.ranges:
            if msg.range_min < r < msg.range_max and not (math.isnan(r) or math.isinf(r)):
                # Punto en frame local
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                # Transformar a world
                wx = self.car_x + cos_yaw * lx - sin_yaw * ly
                wy = self.car_y + sin_yaw * lx + cos_yaw * ly
                self.scan_points_x.append(wx)
                self.scan_points_y.append(wy)
            angle += msg.angle_increment

        # Limitar memoria
        if len(self.scan_points_x) > self.max_scan_points:
            excess = len(self.scan_points_x) - self.max_scan_points
            del self.scan_points_x[:excess]
            del self.scan_points_y[:excess]

    def publish_markers(self):
        ma = MarkerArray()
        # Limpiar primero
        clear = Marker()
        clear.header.frame_id = 'world'
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for i, (wx, wy) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'gui_waypoints'
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
                m.color.r, m.color.g, m.color.b, m.color.a = 0.5, 0.5, 0.5, 0.5
            elif i == self.current_wp_index and self.navigating:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.3, 1.0
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.9, 0.0, 0.9
            ma.markers.append(m)

            # Label
            mt = Marker()
            mt.header.frame_id = 'world'
            mt.header.stamp = self.get_clock().now().to_msg()
            mt.ns = 'gui_labels'
            mt.id = i + 500
            mt.type = Marker.TEXT_VIEW_FACING
            mt.action = Marker.ADD
            mt.pose.position.x = wx
            mt.pose.position.y = wy
            mt.pose.position.z = 0.3
            mt.scale.z = 0.2
            mt.text = str(i + 1)
            mt.color.r, mt.color.g, mt.color.b, mt.color.a = 1.0, 1.0, 1.0, 1.0
            ma.markers.append(mt)

        self.marker_pub.publish(ma)

        # Path
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
            self.stop()
            self.navigating = False
            self.get_logger().info('🏁 ¡RUTA COMPLETADA!')
            return

        gx, gy = self.waypoints[self.current_wp_index]
        dx = gx - self.car_x
        dy = gy - self.car_y
        dist = math.hypot(dx, dy)

        if dist < self.arrival_radius:
            self.get_logger().info(f'✅ WP #{self.current_wp_index + 1} alcanzado!')
            self.current_wp_index += 1
            self.publish_markers()
            return

        v_target = self.v_ref if dist > 1.0 else max(0.2, self.v_ref * dist * 0.8)
        if self.current_v < v_target:
            self.current_v = min(self.current_v + 0.02, v_target)
        else:
            self.current_v = max(self.current_v - 0.04, v_target)

        alpha = self.normalize_angle(math.atan2(dy, dx) - self.car_yaw)
        lookahead = max(dist, 0.3)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), lookahead)
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))
        curve_factor = 1.0 - (abs(delta) / self.max_steer) * 0.4
        final_v = self.current_v * curve_factor

        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = float(final_v)
        cmd.vector.y = float(delta)
        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi: a -= 2 * math.pi
        while a < -math.pi: a += 2 * math.pi
        return a


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPlannerGUI()

    # ROS spinning en background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ---- MATPLOTLIB GUI ----
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    fig.canvas.manager.set_window_title('RoboRacer Waypoint Planner')
    ax.set_facecolor('#1a1a2e')
    fig.patch.set_facecolor('#16213e')

    # Cargar y dibujar la textura de la pista
    pkg_dir = get_package_share_directory('roboracer_gazebo')
    texture_path = os.path.join(pkg_dir, 'models', 'track', 'materials', 'textures', 'SDCS_MapLayout.png')
    if os.path.exists(texture_path):
        img = mpimg.imread(texture_path)
        # Ajustar la rotación de la imagen para que coincida con la pista real
        # Si rot90(img, 2) la puso en X, intentamos rotarla 90 grados a la izquierda,
        # y espejarla horizontalmente si es necesario.
        img = np.rot90(img, 1)
        # Usamos np.fliplr(img) para voltearla como espejo, ayuda a alinear rutas
        img = np.fliplr(img)
        ax.imshow(img, extent=[-2.4, 2.4, -3.05, 3.05], origin='upper', alpha=0.9, zorder=0)

    # Dibujar las paredes para referencia
    walls = [
        # Short walls (horizontal, Y=-3.05 and Y=3.05)
        plt.Rectangle((-2.4, -3.05 - 0.125), 4.8, 0.25, color='#8B6914', alpha=0.8, zorder=1),
        plt.Rectangle((-2.4, 3.05 - 0.125), 4.8, 0.25, color='#8B6914', alpha=0.8, zorder=1),
        # Long walls (vertical, X=-2.4 and X=2.4)
        plt.Rectangle((-2.4 - 0.125, -3.05), 0.25, 6.1, color='#8B6914', alpha=0.8, zorder=1),
        plt.Rectangle((2.4 - 0.125, -3.05), 0.25, 6.1, color='#8B6914', alpha=0.8, zorder=1),
    ]
    for w in walls:
        ax.add_patch(w)

    ax.set_xlim(-3.5, 3.5)
    ax.set_ylim(-4.0, 4.0)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.15, color='#444444')
    ax.set_xlabel('X (metros)', color='white', fontsize=12)
    ax.set_ylabel('Y (metros)', color='white', fontsize=12)
    ax.tick_params(colors='white')

    # Elementos dinámicos
    lidar_scatter = ax.scatter([], [], s=1.5, c='#00ff88', alpha=0.6, zorder=2)  # Mapa LiDAR
    trail_line, = ax.plot([], [], '-', color='#00aaff', alpha=0.4, linewidth=1.5, zorder=3)  # Trail
    
    # Carro (Polígono "Pro" en lugar de un punto)
    car_patch = patches.Polygon(np.zeros((5, 2)), closed=True, facecolor='#ff3333', edgecolor='white', linewidth=1.5, zorder=15)
    ax.add_patch(car_patch)
    
    wp_scatter = ax.scatter([], [], c=[], s=120, zorder=5, edgecolors='white', linewidth=2)
    wp_lines, = ax.plot([], [], '--', color='#ffaa00', alpha=0.7, linewidth=2)
    wp_texts = []

    status_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                          fontsize=11, color='#00ff88', verticalalignment='top',
                          fontfamily='monospace',
                          bbox=dict(boxstyle='round', facecolor='#0a0a1a', alpha=0.8))

    instructions = (
        "[Click Izq] Agregar WP    [Click Der] Borrar ultimo\n"
        "[Enter] GO!    [C] Limpiar    [S] Guardar    [Q] Salir"
    )
    ax.text(0.5, 0.02, instructions, transform=ax.transAxes,
            fontsize=9, color='#aaaaaa', ha='center', va='bottom',
            fontfamily='monospace',
            bbox=dict(boxstyle='round', facecolor='#0a0a1a', alpha=0.8))

    def on_click(event):
        if event.inaxes != ax:
            return
        if event.button == 1:  # Click izquierdo = agregar
            node.waypoints.append((event.xdata, event.ydata))
            idx = len(node.waypoints)
            node.get_logger().info(f'📍 WP #{idx}: ({event.xdata:.2f}, {event.ydata:.2f})')
            node.publish_markers()
        elif event.button == 3:  # Click derecho = borrar último
            if node.waypoints:
                removed = node.waypoints.pop()
                node.get_logger().info(f'🗑️  Removed WP ({removed[0]:.2f}, {removed[1]:.2f})')
                node.publish_markers()

    def on_key(event):
        if event.key == 'enter':
            if node.waypoints and not node.navigating:
                node.current_wp_index = 0
                node.current_v = 0.0
                node.navigating = True
                node.get_logger().info(f'🚗 GO! Navegando {len(node.waypoints)} waypoints')
                node.publish_markers()
        elif event.key == 'c':
            node.waypoints.clear()
            node.current_wp_index = 0
            node.navigating = False
            node.current_v = 0.0
            node.stop()
            node.publish_markers()
            for t in wp_texts:
                t.remove()
            wp_texts.clear()
            node.get_logger().info('🧹 Waypoints limpiados')
        elif event.key == 's':
            if node.waypoints:
                out_dir = os.path.expanduser(
                    '~/Documents/Assesment-Auto/src/racing_logic/roboracer_racing/routes')
                os.makedirs(out_dir, exist_ok=True)
                ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                fp = os.path.join(out_dir, f'route_gui_{ts}.json')
                data = {
                    'name': f'GUI Route {ts}',
                    'total_points': len(node.waypoints),
                    'waypoints': [{'x': round(x, 4), 'y': round(y, 4), 'yaw': 0.0}
                                  for x, y in node.waypoints]
                }
                with open(fp, 'w') as f:
                    json.dump(data, f, indent=2)
                node.get_logger().info(f'💾 Ruta guardada: {fp}')
        elif event.key == 'q':
            node.stop()
            plt.close(fig)

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)

    def update_plot(_=None):
        # LiDAR map
        if node.scan_points_x:
            lidar_scatter.set_offsets(
                np.column_stack([node.scan_points_x, node.scan_points_y]))

        # Car trail
        if node.trail_x:
            trail_line.set_data(node.trail_x, node.trail_y)

        # Atualizar Carro Pro (Shape del QCar)
        L = 0.42  # Largo del carro
        W = 0.19  # Ancho del carro
        # Puntos base del carro (centro en la mitad)
        # [x, y] -> frente es +x
        car_pts = np.array([
            [ L/2, -W/2],       # Esquina inferior derecha
            [ L/2,  W/2],       # Esquina superior derecha
            [-L/2,  W/2],       # Esquina superior izquierda
            [-L/2, -W/2],       # Esquina inferior izquierda
            [ L/2 + 0.12, 0.0]  # Nariz del carro para indicar frente
        ])
        
        # Matriz de rotación basada en el yaw del carro
        c, s = math.cos(node.car_yaw), math.sin(node.car_yaw)
        R = np.array([[c, -s],
                      [s,  c]])
        
        # Rotar y trasladar los puntos
        car_pts_rot = np.dot(car_pts, R.T)
        car_pts_trans = car_pts_rot + np.array([node.car_x, node.car_y])
        car_patch.set_xy(car_pts_trans)

        # Waypoints
        for t in wp_texts:
            t.remove()
        wp_texts.clear()

        if node.waypoints:
            xs = [w[0] for w in node.waypoints]
            ys = [w[1] for w in node.waypoints]
            colors = []
            for i in range(len(node.waypoints)):
                if node.navigating and i < node.current_wp_index:
                    colors.append('#666666')
                elif node.navigating and i == node.current_wp_index:
                    colors.append('#00ff44')
                else:
                    colors.append('#ffcc00')

            wp_scatter.set_offsets(np.column_stack([xs, ys]))
            wp_scatter.set_color(colors)
            wp_lines.set_data(xs, ys)

            for i, (wx, wy) in enumerate(node.waypoints):
                t = ax.text(wx + 0.1, wy + 0.1, str(i + 1),
                           color='white', fontsize=10, fontweight='bold', zorder=12)
                wp_texts.append(t)
        else:
            wp_scatter.set_offsets(np.empty((0, 2)))
            wp_lines.set_data([], [])

        # Status
        if node.navigating:
            wp_idx = min(node.current_wp_index, len(node.waypoints) - 1)
            if node.current_wp_index < len(node.waypoints):
                gx, gy = node.waypoints[wp_idx]
                dist = math.hypot(gx - node.car_x, gy - node.car_y)
                status = (f'[NAVIGATING] WP {node.current_wp_index+1}/{len(node.waypoints)}\n'
                          f'   dist: {dist:.2f}m  vel: {node.current_v:.2f} m/s')
            else:
                status = '[ROUTE COMPLETE]'
        else:
            status = (f'[IDLE] | {len(node.waypoints)} waypoints\n'
                      f'   Car: ({node.car_x:.2f}, {node.car_y:.2f})')
        status_text.set_text(status)

        fig.canvas.draw_idle()

    timer = fig.canvas.new_timer(interval=100)  # 10 Hz refresh
    timer.add_callback(update_plot)
    timer.start()

    plt.title('RoboRacer Waypoint Planner', color='white', fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.show()

    # Cleanup
    node.stop()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
