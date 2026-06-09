"""
Autonomous Lane Follower (Pure Pursuit Edition) — QCar Physical
==============================================================
Ported from PurePursuitNode architecture:
- Dynamic Lookahead based on speed.
- Acceleration Ramps and behavior styles (Racing/Conservative).
- Virtual waypoint generation from Lane Offset.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32, Bool
import time
import math
import numpy as np

class AutonomousLaneFollower(Node):
    def __init__(self):
        super().__init__('autonomous_lane_follower')

        # --- PARÁMETROS DE ESTILO (Inspirados en pure_pursuit_node.py) ---
        self.declare_parameter('behavior_style', 'BALANCED')
        self.declare_parameter('v_ref', 0.8)
        self.declare_parameter('track_width', 0.5) # Ancho estimado de carril (m)

        # CAP DURO DE VELOCIDAD: limite absoluto que se aplica DESPUES de todo
        # (estilo, curvas, LIDAR). Ningun comando publicado superara este valor.
        # Es una red de seguridad final: aunque algo en la cadena calcule
        # una velocidad mayor, esto la corta. Default conservador.
        self.declare_parameter('v_max_safety', 0.20)
        
        # Estilos: (aceleracion, escala_vel, base_lookahead)
        self.styles = {
            'CONSERVATIVE': (0.4, 0.5, 1.2),
            'BALANCED':     (0.7, 0.8, 0.9),
            'RACING':       (1.0, 1.3, 0.6),
        }

        style_name = self.get_parameter('behavior_style').value
        self.style = self.styles.get(style_name, self.styles['BALANCED'])
        self.v_ref = self.get_parameter('v_ref').value
        self.L = 0.256 # QCar wheelbase
        self.max_steer = 0.52 # ~30 deg

        # --- ESTADO ---
        self.offset = 0.0
        self.confidence = 0.0
        self.lidar_min = 9.9
        self.is_obstacle = False
        self.stop_sign = False
        self.current_v = 0.0
        self.active = False
        self.last_time = time.time()

        # --- SUBSCRIPCIONES ---
        self.create_subscription(Float32, '/lane/center_offset', self._offset_cb, 10)
        self.create_subscription(Float32, '/lane/confidence', self._conf_cb, 10)
        self.create_subscription(Bool, '/lane/stop_sign', self._stop_cb, 10)
        self.create_subscription(Float32, '/lidar/min_distance', self._lidar_cb, 10)
        self.create_subscription(Bool, '/lidar/obstacle', self._obs_cb, 10)
        self.create_subscription(Bool, '/lane/enable_auto', self._toggle_cb, 10)

        # --- PUBLICACIONES ---
        self.cmd_pub = self.create_publisher(Vector3Stamped, '/qcar/user_command', 10)
        self.active_pub = self.create_publisher(Bool, '/lane/auto_active', 10)

        # Loop de control a 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info(f'🏎️  PURE PURSUIT LANE FOLLOWER | Style: {style_name}')

    def _offset_cb(self, msg): self.offset = msg.data
    def _conf_cb(self, msg): self.confidence = msg.data
    def _stop_cb(self, msg): self.stop_sign = msg.data
    def _lidar_cb(self, msg): self.lidar_min = msg.data
    def _obs_cb(self, msg): self.is_obstacle = msg.data

    def _toggle_cb(self, msg):
        if msg.data and not self.active:
            self.active = True
            self.get_logger().info('🟢 MODO AUTONOMO ACTIVADO')
        elif not msg.data and self.active:
            self.active = False
            self.stop_car()
            self.get_logger().warn('🔴 MODO AUTONOMO DESACTIVADO')

    def control_loop(self):
        self.active_pub.publish(Bool(data=self.active))
        if not self.active: return

        # 1. Definir Velocidad Objetivo según Estilo
        accel_A, v_scale, l_base = self.style
        v_target = self.v_ref * v_scale

        # 2. Seguridad LIDAR (Freno de emergencia)
        if self.lidar_min < 0.6 or self.is_obstacle:
            v_target = 0.0
        elif self.lidar_min < 1.5:
            v_target *= 0.4 # Slow down

        if self.confidence < 0.35 or self.stop_sign:
            v_target = 0.0

        # 3. Rampa de Aceleración (Strucure from pure_pursuit_node.py)
        accel_rate = 0.02 * accel_A
        if self.current_v < v_target:
            self.current_v = min(self.current_v + accel_rate, v_target)
        elif self.current_v > v_target:
            self.current_v = max(self.current_v - 0.05, v_target)

        # 4. PURE PURSUIT LOGIC
        # Lookahead dinámico
        lookahead = (0.4 * l_base) + (self.current_v * 0.5)
        lookahead = max(lookahead, 0.35)

        # Generar "Waypoint Virtual"
        # El offset está normalizado [-1, 1]. track_width es la amplitud en metros.
        # Target Y (lateral) = -offset * (ancho_carril / 2)
        target_y = -self.offset * (self.get_parameter('track_width').value / 2.0)
        target_x = lookahead

        # Ángulo al punto (alpha)
        alpha = math.atan2(target_y, target_x)

        # Ley de Pure Pursuit: delta = atan2(2*L*sin(alpha), lookahead)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), lookahead)
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))

        # Reducir velocidad en curvas
        curve_factor = 1.0 - (abs(delta) / self.max_steer) * 0.35
        final_v = self.current_v * curve_factor

        # CAP DURO DE SEGURIDAD: nunca superar v_max_safety, pase lo que pase.
        # Se lee como parametro dinamico para tunearlo sin reiniciar el nodo.
        v_max_safety = float(self.get_parameter('v_max_safety').value)
        final_v = float(np.clip(final_v, 0.0, v_max_safety))

        # 5. Publicar Comando
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = float(final_v)
        cmd.vector.y = float(delta)
        self.cmd_pub.publish(cmd)

        if final_v == 0.0 and v_target == 0.0 and self.active:
            # Informar por qué está detenido sin desactivar el modo
            msg = "ESPERANDO LINEA/OBSTACULO"
            if self.is_obstacle or self.lidar_min < 0.6: msg = "OBSTACULO EN RUTA"
            elif self.confidence < 0.35: msg = "BUSCANDO CARRIL..."
            
            # Limitar logs para no saturar
            if int(time.time() * 2) % 10 == 0:
                self.get_logger().info(f'⏱️  Auto-Hold: {msg}', throttle_duration_sec=2.0)

    def stop_car(self):
        cmd = Vector3Stamped()
        cmd.vector.x = 0.0
        cmd.vector.y = 0.0
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousLaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

