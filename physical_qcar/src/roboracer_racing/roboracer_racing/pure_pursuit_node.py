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
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.publisher import Publisher
from visualization_msgs.msg import Marker, MarkerArray
import math
import sys
import time
import select
import termios
import tty
import threading
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
        self.declare_parameter('behavior_style', 'CONSERVATIVE')   # default seguro
        self.declare_parameter('v_ref', 0.15)                       # MUY bajo (era 0.8)
        # Topics del QCar fisico (cambia con --ros-args si usas sim)
        self.declare_parameter('odom_topic', '/qcar/odom_fused')
        self.declare_parameter('cmd_topic',  '/qcar/user_command')
        self.declare_parameter('scan_topic', '/qcar/scan')
        self.declare_parameter('loop', True)                        # Repetir en circuito
        # Cap absoluto de velocidad: nunca superar esto, pase lo que pase
        self.declare_parameter('v_max_safety', 0.18)
        # Steering gain: amplifica el comando (compensa zona muerta servo)
        self.declare_parameter('steering_gain', 1.0)
        # Signo del steering segun montaje del servo (-1 o +1)
        self.declare_parameter('steering_sign', -1.0)
        # Arranque: false = espera ENTER/SPACE, true = arranca solo
        self.declare_parameter('auto_start', False)
        # Velocidad minima del motor: si el comando es menor pero no cero,
        # se eleva a este valor para cruzar la zona muerta del motor/servo.
        # Sin esto, el carro puede recibir 0.03 y no moverse (zona muerta).
        self.declare_parameter('min_motor_speed', 0.12)
        # Tolerancia para verificar que el carro este en el origen de la ruta.
        # Si esta mas lejos del primer waypoint que esto, alerta al arrancar.
        self.declare_parameter('origin_tolerance_m', 0.30)

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
        self.steering_sign = float(self.get_parameter('steering_sign').value)

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
        # Control de arranque manual (igual que lane_follower_pp)
        self.running = bool(self.get_parameter('auto_start').value)

        # --- PUBLISHERS ---
        cmd_topic = self.get_parameter('cmd_topic').value

        # QoS BEST_EFFORT depth=1 — matchea con qcarnode (sin retransmision
        # ni bloqueo, igual que el teleop y lane_follower_pp)
        cmd_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability  = QoSDurabilityPolicy.VOLATILE,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 1,
        )
        self.cmd_pub = self.create_publisher(Vector3Stamped, cmd_topic, cmd_qos)
        self.path_pub = self.create_publisher(Path, '/viz/racing_path', 10)
        self.marker_pub = self.create_publisher(Marker, '/viz/racing_hud', 10)
        # active_pub: indica al teleop que NO publique para no competir
        self.active_pub = self.create_publisher(Bool, '/lane/auto_active', 10)
        # Publisher de reset_odom: tecla R lo dispara para resetear el origen
        self.reset_odom_pub = self.create_publisher(Bool, '/qcar/reset_odom', 10)

        # --- SUBSCRIBERS ---
        odom_topic = self.get_parameter('odom_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, scan_topic, self.scan_callback, 10)

        # --- CARGAR RUTA ---
        # Si no se especifico route_file por parametro, buscar automaticamente
        # la mas reciente en ~/Assesment_qcar_irs/src/roboracer_racing/routes/.
        # Esto evita el problema de pasar paths largos por linea de comando.
        route_file = self.get_parameter('route_file').value
        if not route_file:
            route_file = self._find_latest_route()
            if route_file:
                self.get_logger().info(
                    f'📂 Auto-detectada ruta mas reciente: {os.path.basename(route_file)}'
                )
        if route_file:
            self.load_route(route_file)

        # --- LOOP DE CONTROL a 50Hz ---
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(f'🏎️ PURE PURSUIT ACTIVO | Estilo: {style_name} | v_ref: {self.v_ref}')
        if not route_file:
            self.get_logger().warn('⚠️  No route file specified! Use: -p route_file:=/path/to/route.json')

    def _find_latest_route(self):
        """Busca el archivo route_*.json mas reciente en la carpeta routes."""
        import glob
        routes_dir = os.path.expanduser(
            '~/Assesment_qcar_irs/src/roboracer_racing/routes'
        )
        if not os.path.isdir(routes_dir):
            self.get_logger().warn(f'No existe la carpeta {routes_dir}')
            return ''
        files = sorted(glob.glob(os.path.join(routes_dir, 'route_*.json')))
        if not files:
            self.get_logger().warn(f'No hay rutas en {routes_dir}')
            return ''
        return files[-1]   # ultimo por orden alfabetico = mas reciente por timestamp

    def load_route(self, filepath):
        """
        Carga waypoints desde JSON y los ALINEA AL ORIGEN del state_estimator.

        Por que es necesario:
          El qcar_state_estimator fija el origen (0,0,0) en la posicion ACTUAL
          del carro al arrancar. Las rutas grabadas tienen coordenadas
          ABSOLUTAS del momento en que se grabaron (ej. x=7.66, y=-1.91).
          Si no las trasladamos, el Pure Pursuit cree que tiene que viajar
          7.66 m en linea recta antes de "alcanzar" el primer waypoint.

        Solucion: trasladar y rotar la ruta para que el PRIMER waypoint
        quede en (0,0) con yaw=0. Asi se vuelve relativa al punto donde
        el carro arranca, y la geometria es consistente entre sesiones.
        """
        if not os.path.exists(filepath):
            self.get_logger().error(f'❌ Route file not found: {filepath}')
            return

        with open(filepath, 'r') as f:
            data = json.load(f)

        if not data.get('waypoints'):
            self.get_logger().error('❌ Ruta vacia')
            return

        raw_x   = [float(wp['x']) for wp in data['waypoints']]
        raw_y   = [float(wp['y']) for wp in data['waypoints']]
        raw_yaw = [float(wp.get('yaw', 0.0)) for wp in data['waypoints']]

        # Origen de la ruta = primer waypoint
        x0, y0, yaw0 = raw_x[0], raw_y[0], raw_yaw[0]

        # Trasladar y rotar para que el primer waypoint sea (0, 0, 0)
        cos_t = math.cos(-yaw0)
        sin_t = math.sin(-yaw0)
        self.cx, self.cy, self.cyaw = [], [], []
        for x_w, y_w, yaw_w in zip(raw_x, raw_y, raw_yaw):
            # Trasladar al origen
            dx = x_w - x0
            dy = y_w - y0
            # Rotar -yaw0 para alinear el heading inicial con el eje X
            x_local = cos_t * dx - sin_t * dy
            y_local = sin_t * dx + cos_t * dy
            yaw_local = normalize_angle(yaw_w - yaw0)
            self.cx.append(x_local)
            self.cy.append(y_local)
            self.cyaw.append(yaw_local)

        self.target_ind = 0
        self.route_active = True

        self.get_logger().info(
            f'✅ Loaded {len(self.cx)} waypoints from {os.path.basename(filepath)}'
        )
        self.get_logger().info(
            f'   Origen ruta original: ({x0:.2f}, {y0:.2f}, {math.degrees(yaw0):+.1f}°)'
        )
        self.get_logger().info(
            f'   Trasladado a (0, 0, 0). Ultimo waypoint relativo: '
            f'({self.cx[-1]:.2f}, {self.cy[-1]:.2f})'
        )

        # Publicar la ruta en RViz
        self.publish_path_viz()

    def publish_path_viz(self):
        """Publica la ruta como Path para visualización en RViz."""
        msg = Path()
        msg.header.frame_id = 'odom'   # mismo frame que /qcar/odom_fused
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

    def check_origin(self):
        """
        Verifica si el carro esta cerca del primer waypoint (origen de la ruta).
        Despues del fix de auto-alineacion, el primer waypoint ES (0,0,0).
        Si la odometria actual difiere mas que origin_tolerance_m, alerta.
        Retorna True si OK, False si lejos.
        """
        if not self.cx:
            return False
        tolerance = float(self.get_parameter('origin_tolerance_m').value)
        dist = math.hypot(self.current_x - self.cx[0],
                          self.current_y - self.cy[0])
        yaw_diff = abs(normalize_angle(self.current_yaw - self.cyaw[0]))

        if dist <= tolerance and yaw_diff < 0.35:    # ~20 grados
            self.get_logger().info(
                f'✅ CARRO EN ORIGEN OK | dist={dist*100:.1f}cm '
                f'yaw_diff={math.degrees(yaw_diff):+.1f}° → ARRANCANDO'
            )
            return True
        else:
            self.get_logger().warn(
                f'⚠️  CARRO LEJOS DEL ORIGEN | pose=({self.current_x:+.2f}, '
                f'{self.current_y:+.2f}, {math.degrees(self.current_yaw):+.1f}°) | '
                f'dist={dist*100:.1f}cm yaw_diff={math.degrees(yaw_diff):+.1f}°'
            )
            self.get_logger().warn(
                f'   → MOVER el carro al punto (0,0) o presionar R para resetear odom'
            )
            return False

    def reset_odom(self):
        """Manda comando de reset al state_estimator (origen = pose actual)."""
        self.reset_odom_pub.publish(Bool(data=True))
        self.get_logger().info('🔄 Reset de odometria solicitado (pose actual = nuevo origen)')

    def control_loop(self):
        """Loop principal de Pure Pursuit a 50Hz."""
        # Publicar el estado activo en cada ciclo (para que teleop sepa).
        # Activo = running Y ruta cargada Y al menos 2 waypoints.
        is_active = self.running and self.route_active and len(self.cx) >= 2
        self.active_pub.publish(Bool(data=is_active))

        if not is_active:
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

        # Aplicar steering_gain (compensa zona muerta servo o pequeñas desviaciones)
        steering_gain = float(self.get_parameter('steering_gain').value)
        delta = delta * steering_gain

        # Clamp steering al maximo fisico
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))

        # Reducir velocidad en curvas cerradas
        curve_factor = 1.0 - (abs(delta) / self.max_steer) * 0.4
        final_v = self.current_v * curve_factor

        # CAP DURO de velocidad — nunca superar v_max_safety
        v_max_safety = float(self.get_parameter('v_max_safety').value)
        final_v = float(np.clip(final_v, 0.0, v_max_safety))

        # MIN MOTOR SPEED: si comando > 0 pero menor a min_motor_speed,
        # forzar a min_motor_speed para cruzar zona muerta del motor.
        # Evita el caso "el control envia 0.04 pero el motor no se mueve".
        min_motor = float(self.get_parameter('min_motor_speed').value)
        if 0.001 < final_v < min_motor:
            final_v = min_motor

        # Aplicar signo del steering segun montaje del servo
        delta_cmd = self.steering_sign * delta

        # --- PUBLICAR COMANDO ---
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = float(final_v)
        cmd.vector.y = float(delta_cmd)
        self.cmd_pub.publish(cmd)

        # --- HUD EN RVIZ ---
        self.publish_hud(final_v, delta_cmd)

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


def _get_key(timeout=0.1):
    """Lee 1 tecla del stdin sin bloquear (modo raw)."""
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)


def _keyboard_loop(node):
    """ENTER/SPACE = arrancar, R = reset odometria, Q = parar limpio."""
    print('\n  Controles:', flush=True)
    print('    ENTER / SPACE  →  ARRANCAR', flush=True)
    print('    R              →  RESET odometria (pose actual = origen)', flush=True)
    print('    P              →  Mostrar POSE actual del carro', flush=True)
    print('    Q              →  PARAR / Salir', flush=True)
    print('', flush=True)
    try:
        while rclpy.ok():
            key = _get_key(timeout=0.2)
            if key is None:
                continue
            if key in ('\r', '\n', ' '):
                if not node.running:
                    # Verificar origen ANTES de arrancar
                    if node.check_origin():
                        node.running = True
                        print('\n  ▶  ARRANCANDO Pure Pursuit ruta...', flush=True)
                    else:
                        print(
                            '\n  ❌  NO ARRANCO — Carro lejos del origen.',
                            '\n      Opciones:',
                            '\n      1) Mover el carro fisicamente al punto (0,0)',
                            '\n      2) Presionar R para resetear odom (origen = pose actual)',
                            '\n      3) Forzar arranque: presionar SPACE de nuevo',
                            flush=True,
                        )
                else:
                    # Si presiona ENTER/SPACE estando ya activo, forzar arranque
                    # (ignora check_origin — para casos donde sabes lo que haces)
                    print('\n  ▶  CARRO YA EN MARCHA (segundo ENTER ignorado)', flush=True)

            elif key in ('r', 'R'):
                node.reset_odom()
                print('\n  🔄 Reset enviado. Espera 1s para que tome efecto.', flush=True)

            elif key in ('p', 'P'):
                pose_str = (
                    f'\n  📍 POSE ACTUAL: x={node.current_x:+.3f}m '
                    f'y={node.current_y:+.3f}m yaw={math.degrees(node.current_yaw):+.1f}°'
                )
                if node.cx:
                    dist = math.hypot(
                        node.current_x - node.cx[0],
                        node.current_y - node.cy[0],
                    )
                    pose_str += f'\n     Dist al wp[0]: {dist*100:.1f}cm'
                    pose_str += f'\n     Wp[0]: ({node.cx[0]:.2f}, {node.cy[0]:.2f})'
                    pose_str += f'\n     Wp[-1]: ({node.cx[-1]:.2f}, {node.cy[-1]:.2f})'
                print(pose_str, flush=True)

            elif key in ('q', 'Q', '\x03'):
                if node.running:
                    node.running = False
                    node.stop_vehicle()
                    print('\n  ■  PARADO. Q de nuevo para salir.', flush=True)
                else:
                    break
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()

    # Si stdin es tty (corre en terminal real), arrancar con teclado.
    # Si no (lanzado por launch sin tty), usa el flag auto_start.
    if sys.stdin.isatty():
        ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        ros_thread.start()
        try:
            _keyboard_loop(node)
        except KeyboardInterrupt:
            pass
    else:
        node.running = True
        node.get_logger().info('Sin TTY — arrancando automaticamente (modo launch)')
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass

    node.running = False
    node.stop_vehicle()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

if __name__ == '__main__':
    main()
