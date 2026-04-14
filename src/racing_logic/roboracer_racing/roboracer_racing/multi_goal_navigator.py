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
from std_msgs.msg import Empty, Float32, Float32MultiArray
from sensor_msgs.msg import LaserScan
import math
import numpy as np
import threading
import sys
import json
import os
from datetime import datetime
import subprocess
import time


LIDAR_RANGE = 4.0


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

        # Obstacle avoidance state
        self.aeb_active = False
        self.avoid_steering_offset = 0.0
        self.min_dist_front = 10.0
        self.warning_active = False
        self.aeb_hold_ticks = 0  # counts consecutive AEB-HOLD ticks → triggers reverse
        
        # Fase 7: Formal APF & Sector Analysis State
        self.f_att = (0.0, 0.0)
        self.f_rep = (0.0, 0.0)
        self.sector_clearance = [LIDAR_RANGE] * 12
        self.last_delta = 0.0

        # Fase Visión: Lane-Assist (amarillo) state
        self.lane_offset = 0.0       # normalizado [-1, 1], neg = carril a la izq
        self.lane_conf = 0.0         # 0..1
        self.declare_parameter('k_lane', 0.18)   # ganancia lane-assist sobre delta
        self.declare_parameter('lane_conf_min', 0.25)

        
        # Fase 6: Hybrid Controller State
        self.repel_vec_x = 0.0
        self.repel_vec_y = 0.0
        self.gap_angles = (0.0, 0.0)
        self.last_delta = 0.0  # Para Histeresis

        
        # Virtual obstacles (Fase 3+)
        self.virtual_obstacles = [] # Lista de (x, y)
        self.spawned_obstacles = [] # Nombres en Gazebo

        # Publishers
        cmd_topic = self.get_parameter('cmd_topic').value
        self.cmd_pub = self.create_publisher(Vector3Stamped, cmd_topic, 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, '/viz/waypoints', 10)
        self.path_pub = self.create_publisher(Path, '/viz/waypoint_path', 10)
        self.hud_pub = self.create_publisher(Marker, '/viz/nav_hud', 10)
        self.telemetry_pub = self.create_publisher(Float32MultiArray, '/viz/telemetry', 10)
        self.obs_marker_pub = self.create_publisher(MarkerArray, '/viz/virtual_obstacles', 10)
        self.lidar_fan_pub = self.create_publisher(Marker, '/viz/lidar_fan', 10)

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

        # LiDAR Scan (Fase 3)
        self.scan_sub = self.create_subscription(
            LaserScan, '/qcar_sim/scan', self.scan_cb, 10
        )

        # Lane-Assist (Fase Visión)
        self.lane_off_sub = self.create_subscription(
            Float32, '/lane/center_offset', self._lane_off_cb, 10
        )
        self.lane_conf_sub = self.create_subscription(
            Float32, '/lane/confidence', self._lane_conf_cb, 10
        )

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
        print("1. RViz 'Publish Point' -> Waypoint (Green/Yellow)")
        print("2. RViz '2D Nav Goal'    -> Virtual Obstacle (Red Box)")
        print("3. Escribe comandos aquí abajo:")
        print("   [g] Go! (Inicia el recorrido)")
        print("   [c] Clear (Borra todo)")
        print("   [s] Save (Guarda ruta + obstáculos)")
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
        if not self.waypoints and not self.virtual_obstacles:
            print("⚠️ No hay datos para guardar.")
            return
        out_dir = os.path.expanduser('~/Documents/Assesment-Auto/src/racing_logic/roboracer_racing/routes')
        os.makedirs(out_dir, exist_ok=True)
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        fp = os.path.join(out_dir, f'race_scenario_{ts}.json')
        data = {
            'name': f'Race Case {ts}',
            'total_waypoints': len(self.waypoints),
            'total_obstacles': len(self.virtual_obstacles),
            'waypoints': [{'x': round(x, 4), 'y': round(y, 4), 'yaw': 0.0} for x, y in self.waypoints],
            'virtual_obstacles': [{'x': round(x, 4), 'y': round(y, 4)} for x, y in self.virtual_obstacles]
        }
        with open(fp, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"💾 Escenario guardado en: {fp}")

    def scan_cb(self, msg):
        """Procesa el radar para detectar obstáculos y calcular evasión."""
        ranges      = np.array(msg.ranges)
        angles      = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filtrar puntos válidos
        mask   = np.isfinite(ranges) & (ranges > 0.1) & (ranges < msg.range_max)
        sd     = ranges[mask]
        av     = angles[mask]
        sx     = sd * np.cos(av)
        sy     = sd * np.sin(av)

        # FOV frontal para alertas
        FOV_FRONT = math.radians(40)
        idx_front = np.where(np.abs(av) < FOV_FRONT)[0]

        if len(idx_front) > 0:
            self.min_dist_front = float(np.min(sd[idx_front]))
        else:
            self.min_dist_front = 10.0

        CRITICAL = 0.42
        WARNING  = 1.80

        if self.min_dist_front < CRITICAL:
            self.aeb_active     = True
            self.warning_active = True
        elif self.min_dist_front < WARNING:
            self.aeb_active     = False
            self.warning_active = True
        else:
            self.aeb_active     = False
            self.warning_active = False

        # ── APF: Repulsive Force — front-biased, capped ──────────────────────
        # Fix: rho0 reduced so distant walls don't accumulate into explosion.
        # Fix: cosine weighting — obstacles straight ahead contribute more
        #      than side walls (which the robot naturally avoids by steering).
        # Fix: hard magnitude cap prevents F_rep overwhelming F_att.
        eta  = 0.12
        rho0 = 1.1   # [m] — only genuinely close obstacles contribute
        rep_x, rep_y = 0.0, 0.0

        FOV_REP   = math.radians(100)           # ignore pure-rear points
        cos_w     = np.maximum(0.0, np.cos(av)) # front=1, ±90°=0, rear=0
        mask_near = (sd < rho0) & (np.abs(av) < FOV_REP)
        if np.any(mask_near):
            coeffs = (1.0 / sd[mask_near] - 1.0 / rho0) / (sd[mask_near] ** 2.0)
            w      = cos_w[mask_near]
            rep_x  = eta * float(np.sum(-sx[mask_near] / sd[mask_near] * coeffs * w))
            rep_y  = eta * float(np.sum(-sy[mask_near] / sd[mask_near] * coeffs * w))
            # Lateral bias: if one side is significantly clearer, prefer it
            # This helps avoid 'shaking' in narrow corridors
            left_clear = np.sum(sd[av > 0]) if np.any(av > 0) else 0.0
            right_clear = np.sum(sd[av < 0]) if np.any(av < 0) else 0.0
            bias = 1.1 if left_clear > right_clear else -1.1
            rep_y += bias * (eta * 0.1) / max(self.min_dist_front, 0.4)

            # Hard cap: walls must never drown out goal attraction
            rep_mag = math.hypot(rep_x, rep_y)
            if rep_mag > 1.8:
                rep_x = rep_x / rep_mag * 1.8
                rep_y = rep_y / rep_mag * 1.8

        self.f_rep = (rep_x, rep_y)

        # 2. Sector Clearance Analysis (12 sectors of 15°)
        # Analizar de -90 a +90 grados (180 deg total)
        sectors = [LIDAR_RANGE] * 12
        for i in range(12):
            s_min = -math.pi/2 + i * math.radians(15)
            s_max = s_min + math.radians(15)
            mask_s = (av >= s_min) & (av < s_max)
            if np.any(mask_s):
                sectors[i] = float(np.min(sd[mask_s]))
        self.sector_clearance = sectors

        # El offset reactivo ahora es puramente el componente lateral de la repulsión
        self.avoid_steering_offset = float(np.clip(rep_y * 1.5, -0.55, 0.55))
        
        if not self.warning_active:
             self.avoid_steering_offset *= 0.5
        
        self.publish_lidar_fan()

    def publish_lidar_fan(self):
        """Publica un abanico en RViz que muestra la zona que el coche está vigilando."""
        m = Marker()
        m.header.frame_id = 'base_link' # Pegado al carro
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'lidar_brain'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 0.05 # Grosor de línea
        
        # Color según estado
        if self.aeb_active:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.8 # Rojo
        elif self.warning_active:
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 0.8 # Naranja
        else:
            m.color.r, m.color.g, m.color.b, m.color.a = 0.0, 1.0, 0.2, 0.6 # Verde

        # Dibujar arco de +-35 grados
        fov = math.radians(35)
        steps = 10
        points = []
        # Punto inicial (el sensor)
        p_center = PointStamped().point
        p_center.x, p_center.y, p_center.z = 0.0, 0.0, 0.1
        
        # Radio del abanico (la distancia que vigila)
        radius = 1.4 
        
        points.append(p_center)
        for i in range(steps + 1):
            angle = -fov + (2 * fov * i / steps)
            p = PointStamped().point
            p.x = radius * math.cos(angle)
            p.y = radius * math.sin(angle)
            p.z = 0.1
            points.append(p)
        points.append(p_center)
        
        m.points = points
        self.lidar_fan_pub.publish(m)

    def _lane_off_cb(self, msg):
        self.lane_offset = float(msg.data)

    def _lane_conf_cb(self, msg):
        self.lane_conf = float(msg.data)

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
        """Agrega un OBSTÁCULO VIRTUAL desde '2D Nav Goal' y lo spawnea en Gazebo."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.virtual_obstacles.append((x, y))
        
        # Generar nombre único
        obs_name = f"obs_{len(self.virtual_obstacles)}_{int(time.time() % 1000)}"
        self.spawned_obstacles.append(obs_name)
        
        self.get_logger().info(f'🚫 OBSTÁCULO VIRTUAL [{obs_name}] en: ({x:.2f}, {y:.2f})')
        self.spawn_in_gazebo(x, y, obs_name)
        self.publish_viz()

    def spawn_in_gazebo(self, x, y, name):
        """Usa ros_gz_sim para meter un cubo físico en la simulación."""
        sdf = (
            f"<sdf version='1.6'><model name='{name}'><static>true</static>"
            "<link name='link'><collision name='c'><geometry><box><size>0.4 0.4 0.4</size></box></geometry></collision>"
            "<visual name='v'><geometry><box><size>0.4 0.4 0.4</size></box></geometry>"
            "<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>"
        )
        cmd = [
            "ros2", "run", "ros_gz_sim", "create",
            "-world", "empty",
            "-string", sdf,
            "-x", str(x), "-y", str(y), "-z", "0.2",
            "-name", name
        ]
        # Lanzar en hilo separado para no bloquear el nodo de ROS
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def remove_from_gazebo(self, name):
        """Elimina el modelo de Gazebo."""
        cmd = [
            "ros2", "run", "ros_gz_sim", "remove",
            "-world", "empty",
            "-name", name
        ]
        subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def add_waypoint(self, x, y):
        self.waypoints.append((x, y))
        idx = len(self.waypoints)
        self.get_logger().info(
            f'📍 Waypoint #{idx}: ({x:.2f}, {y:.2f}) | '
            f'Total: {len(self.waypoints)} waypoints'
        )

        self.publish_viz()

    def clear_cb(self, msg):
        """Limpia todo y para el carro."""
        # Limpiar de Gazebo primero
        for name in self.spawned_obstacles:
            self.remove_from_gazebo(name)
        self.spawned_obstacles.clear()

        self.waypoints.clear()
        self.virtual_obstacles.clear()
        self.current_wp_index = 0
        self.navigating = False
        self.current_v = 0.0
        self.stop()

        # Limpiar todos los markers
        ma = MarkerArray()
        m = Marker()
        m.header.frame_id = 'world'
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.marker_array_pub.publish(ma)
        self.obs_marker_pub.publish(ma)

        self.get_logger().info('🧹 Escenario LIMPIADO')

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

        # Markers de Obstáculos Virtuales
        mo = MarkerArray()
        for i, (ox, oy) in enumerate(self.virtual_obstacles):
            m = Marker()
            m.header.frame_id = 'world'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'virtual_obstacles'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = ox
            m.pose.position.y = oy
            m.pose.position.z = 0.15 # Mitad de su altura
            m.scale.x = 0.3
            m.scale.y = 0.3
            m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 0.0, 0.7 # Rojo semitransparente
            mo.markers.append(m)
        self.obs_marker_pub.publish(mo)

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

        # 1. LiDAR Real (Ya calculado en scan_cb)
        
        # 2. Obstáculos Virtuales (Calculamos cercanía manual)
        v_obs_dist = 10.0
        v_obs_side = 0.0 # Positivo = Obstáculo a la izquierda, hay que esquivar a la derecha
        v_obs_rel_x = 0.0
        v_obs_rel_y = 0.0
        
        _cos_y = math.cos(self.current_yaw)
        _sin_y = math.sin(self.current_yaw)
        for ox, oy in self.virtual_obstacles:
            dx_obs = ox - self.current_x
            dy_obs = oy - self.current_y
            d = math.hypot(dx_obs, dy_obs)
            if d < v_obs_dist:
                # Only consider obstacles ahead of the robot (not already passed)
                _rel_x_chk = dx_obs * _cos_y + dy_obs * _sin_y
                if _rel_x_chk < -0.25:
                    continue  # obstacle is behind — ignore
                v_obs_dist = d
                v_obs_rel_x = _rel_x_chk
                v_obs_rel_y = -dx_obs * _sin_y + dy_obs * _cos_y
                # Ver si está a la izquierda o derecha relativa al heading
                alpha_obs = normalize_angle(math.atan2(dy_obs, dx_obs) - self.current_yaw)
                v_obs_side = 1.0 if alpha_obs > 0 else -1.0
        
        # Unificamos el sensor real con el virtual (tomamos el más restrictivo)
        effective_dist = min(self.min_dist_front, v_obs_dist)
        
        # Coordenada del obstáculo dominante para el Dash
        if v_obs_dist < self.min_dist_front:
            dom_obs_rel_x, dom_obs_rel_y = v_obs_rel_x, v_obs_rel_y
        else:
            # Para el LiDAR real, aproximamos la posición frontal
            dom_obs_rel_x, dom_obs_rel_y = self.min_dist_front, 0.0

        
        # ── Thresholds (matched to scan_cb) ──────────────────────────────────
        CRITICAL = 0.42
        WARNING  = 1.80
        CAUTION  = 2.50

        if effective_dist < CRITICAL:
            aeb_active     = True
            warning_active = True
        elif effective_dist < WARNING:
            aeb_active     = False
            warning_active = True
        else:
            aeb_active     = False
            warning_active = False

        # ── Pre-compute combined evasion offset (LiDAR + virtual obstacle) ────
        # Must happen BEFORE the velocity decision so has_escape is correct.
        _pre_offset = self.avoid_steering_offset
        if abs(_pre_offset) < 0.05 and v_obs_dist < 2.5:
            _repel_v    = 1.0 / max(v_obs_dist, 0.35) ** 1.2
            _pre_offset = float(np.clip(-0.22 * _repel_v * v_obs_side, -0.45, 0.45))

        # ── Velocity profile ──────────────────────────────────────────────────
        if aeb_active:
            has_escape = abs(_pre_offset) > 0.08
            if effective_dist < 0.30:
                # Truly stuck: back up immediately
                v_target = -0.5
                self.aeb_hold_ticks = 0
                self.get_logger().warn(
                    f'⏪ REVERSE ESCAPE  dist={effective_dist:.2f}m',
                    throttle_duration_sec=1.0)
            elif has_escape:
                # Clear side found — creep forward while steering hard
                v_target = self.v_ref * 0.18
                self.aeb_hold_ticks = 0
                self.get_logger().warn(
                    f'🔀 EVADING at {effective_dist:.2f}m  offset={_pre_offset:+.2f}',
                    throttle_duration_sec=0.5)
            else:
                # No clear side yet — hold, but escalate to reverse after 2 s
                self.aeb_hold_ticks += 1
                if self.aeb_hold_ticks > 100:        # 2 s at 50 Hz → forced reverse
                    v_target = -0.4
                    if self.aeb_hold_ticks > 150:    # 1 s reverse done → retry
                        self.aeb_hold_ticks = 0
                    self.get_logger().warn(
                        f'⏪ REVERSE TIMEOUT  ticks={self.aeb_hold_ticks}',
                        throttle_duration_sec=1.0)
                else:
                    v_target = 0.0
                    self.get_logger().warn(
                        f'🚨 AEB HOLD  dist={effective_dist:.2f}m',
                        throttle_duration_sec=1.0)
        elif warning_active:
            self.aeb_hold_ticks = 0
            # Proportional slow-down: v_ref → 35% as dist drops from WARNING to CRITICAL
            alpha   = max(0.0, (effective_dist - CRITICAL) / (WARNING - CRITICAL))
            v_target = self.v_ref * (0.35 + 0.65 * alpha)
            src = "VIRTUAL" if v_obs_dist < self.min_dist_front else "LiDAR"
            self.get_logger().info(
                f'⚠️  {src} at {effective_dist:.2f}m  v={v_target:.2f}',
                throttle_duration_sec=1.0)
        elif effective_dist < CAUTION:
            self.aeb_hold_ticks = 0
            # Gentle slow-down zone: v_ref → 80% at CAUTION edge
            alpha    = (effective_dist - WARNING) / (CAUTION - WARNING)
            v_target = self.v_ref * (0.80 + 0.20 * alpha)
        else:
            self.aeb_hold_ticks = 0
            # S-Curve Speed Profile: v_ref * sqrt(dist/slowdown) for natural arrival
            slowdown_range = 1.8
            if dist > slowdown_range:
                v_target = self.v_ref
            else:
                v_target = max(0.25, self.v_ref * math.sqrt(dist / slowdown_range))

        # Pure Pursuit logic
        alpha = normalize_angle(math.atan2(dy, dx) - self.current_yaw)
        lookahead = max(dist * 0.7, 0.5)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), lookahead)
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))

        # Curvature-Aware Velocity Limit: v = v_ref / (1 + k * |delta|)
        # This mimics professional racing where you brake for tight corners.
        k_curv = 1.25 # Aggressive braking in corners
        v_curv_limit = v_target / (1.0 + k_curv * abs(delta))
        final_v = float(np.clip(v_curv_limit, 0.20, v_target))

        # Rampa suave (Second-order smoothing for "Premium" feel)
        accel_step = 0.08
        brake_step = 0.12
        if self.current_v < final_v:
            self.current_v = min(self.current_v + accel_step, final_v)
        else:
            self.current_v = max(self.current_v - brake_step, final_v)

        # ── Evasion steering (applied in ALL states, including AEB) ─────────
        # _pre_offset already merges LiDAR + virtual obstacle repulsion (computed above)
        effective_offset = _pre_offset

        if effective_offset != 0.0:
            # Scale mix: full evasion when AEB, partial during warning
            mix = 1.0 if aeb_active else 0.85
            delta = float(np.clip(
                delta + effective_offset * mix,
                -self.max_steer, self.max_steer))

        # ── Gap-Following blend ──────────────────────────────────────────────
        # When an obstacle is close, steer toward the widest free sector.
        # Uses sector_clearance (12 × 15° bins, −90°…+90°) already computed
        # in scan_cb.  Blend strength grows from 0 → 0.72 as dist approaches
        # CRITICAL, so Pure Pursuit still dominates in open space.
        if (warning_active or aeb_active) and len(self.sector_clearance) == 12:
            secs      = self.sector_clearance
            # GOAL-BIASED GAP SELECTION:
            # Instead of the absolute widest gap, we look for a gap that is "good enough"
            # (> 1.5m) and is the closest to our desired target heading.
            goal_alpha = normalize_angle(math.atan2(dy, dx) - self.current_yaw)
            
            # Find all sectors with reasonable clearance
            safe_threshold = 1.5
            safe_indices = [i for i, d in enumerate(secs) if d > safe_threshold]
            
            if not safe_indices:
                # If nothing is safe, pick the least-dangerous gap
                best_sec = int(np.argmax(secs))
            else:
                # Pick the safe sector that minimizes angular error to goal
                def sec_angle(idx): return -math.pi/2 + (idx + 0.5) * math.radians(15)
                best_sec = min(safe_indices, key=lambda i: abs(normalize_angle(sec_angle(i) - goal_alpha)))

            gap_angle = -math.pi / 2 + (best_sec + 0.5) * math.radians(15)
            gap_steer = float(np.clip(gap_angle * 0.75, -self.max_steer, self.max_steer))
            
            # Blend factor: 0.0 (no obstacles) -> 0.7 (very close)
            # Reduced blend to prevent the gap from totally ignoring the goal
            blend     = float(np.clip(
                1.0 - (effective_dist - CRITICAL) / (WARNING - CRITICAL),
                0.0, 0.65)) 
            delta = float(np.clip(
                delta * (1.0 - blend) + gap_steer * blend,
                -self.max_steer, self.max_steer))

        # ── Fase Visión: Lane-Assist (amarillo) ───────────────────────────
        # Solo cuando la cámara confía y NO estamos esquivando un obstáculo
        # (para que LiDAR/APF siempre manden en emergencias).
        k_lane = float(self.get_parameter('k_lane').value)
        conf_min = float(self.get_parameter('lane_conf_min').value)
        if (self.lane_conf > conf_min
                and not aeb_active
                and abs(_pre_offset) < 0.15):
            # offset > 0 → carril a la derecha → gira derecha (delta negativo en este convenio).
            lane_delta = -k_lane * self.lane_offset * self.lane_conf
            delta = float(np.clip(delta + lane_delta, -self.max_steer, self.max_steer))

        # ── Histeresis: suavizado de steering ───────────────────────────────
        delta = 0.7 * delta + 0.3 * self.last_delta
        self.last_delta = delta

        # Reverse: flip steering for better escape trajectory
        if v_target < 0:
            delta = float(np.clip(-delta * 1.4, -self.max_steer, self.max_steer))

        # Publicar comando
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = float(final_v)
        cmd.vector.y = float(delta)
        self.cmd_pub.publish(cmd)

        # ── Trayectoria Predictiva (Fase 5) ───────────────────────────
        # Calculamos 5 puntos a futuro basados en el steering actual
        prediction_pts = []
        temp_x, temp_y, temp_yaw = 0.0, 0.0, 0.0 # Relativo al robot actual
        dt_step = 0.2 # 200ms por paso
        pred_v = max(0.2, final_v)
        for _ in range(5):
            # Modelo cinemático simple (Ackermann)
            temp_x += pred_v * math.cos(temp_yaw) * dt_step
            temp_y += pred_v * math.sin(temp_yaw) * dt_step
            temp_yaw += (pred_v / self.L) * math.tan(delta) * dt_step
            prediction_pts.extend([float(temp_x), float(temp_y)])

        # ── Fase 7: Attractive Force ($F_{att}$) ──────────────────────────
        # Vector del robot al waypoint (coordenadas relativas)
        cos_y = math.cos(self.current_yaw)
        sin_y = math.sin(self.current_yaw)
        rel_goal_x = dx * cos_y + dy * sin_y
        rel_goal_y = -dx * sin_y + dy * cos_y
        
        # Escalar atracción
        xi = 0.55 # Coeficiente de atracción — even stronger for v12
        att_x = rel_goal_x * xi
        att_y = rel_goal_y * xi
        self.f_att = (att_x, att_y)

        # Force Telemetry logging (Throttle to 1Hz)
        f_rep_mag = math.hypot(self.f_rep[0], self.f_rep[1])
        f_att_mag = math.hypot(att_x, att_y)
        self.get_logger().info(
            f'🦾 INTELLIGENT RACING | v: {final_v:.2f} | Δ: {delta:+.2f} | F_att: {f_att_mag:.2f}',
            throttle_duration_sec=1.5
        )

        # Publicar telemetría (Fase 7: Blueprint Tech)
        telemetry = Float32MultiArray()
        # [0:VT, 1:VA, 2:Steer, 3:Err, 4:Obs, 5..14:Pts, 15:AttX, 16:AttY, 17:RepX, 18:RepY, 19..30:Sectors]
        obs_state = 2.0 if aeb_active else (1.0 if warning_active else 0.0)
        base_data = [float(v_target), float(final_v), float(delta), float(dist), obs_state]
        force_data = [float(att_x), float(att_y), float(self.f_rep[0]), float(self.f_rep[1])]
        
        telemetry.data = base_data + prediction_pts + force_data + [float(s) for s in self.sector_clearance]
        self.telemetry_pub.publish(telemetry)

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
        try:
            cmd = Vector3Stamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.vector.x = 0.0
            cmd.vector.y = 0.0
            self.cmd_pub.publish(cmd)
        except Exception:
            pass  # context already invalid during shutdown


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
