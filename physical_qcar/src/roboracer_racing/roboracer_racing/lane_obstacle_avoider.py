"""
Lane Obstacle Avoider — nodo intermedio entre lane_detector y lane_follower_pp.

Decide si mantener el carril preferido o esquivar al opuesto cuando el lidar
detecta un obstaculo en la trayectoria. Reescribe /lane_target_point_m con un
sesgo lateral y republica en /lane_target_point_m_safe (que el follower debe
consumir via parametro target_point_topic).

preferred_lane se AUTODETECTA del primer carril estable que reporte el
lane_detector (no se setea a mano).
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Bool, String


# Cono frontal en radianes para considerar que el obstaculo esta "delante"
# (compatible con la salida de lidar_processor.closest_angle)
_FRONT_CONE_RAD = 0.6  # ~35 grados a cada lado del eje X


class LaneObstacleAvoider(Node):
    def __init__(self):
        super().__init__('lane_obstacle_avoider')

        # Topics in/out
        self.declare_parameter('target_in_topic',  '/lane_target_point_m')
        self.declare_parameter('target_out_topic', '/lane_target_point_m_safe')

        # Geometria del carril (en metros) — half-width del carril
        # Si el target_x viene en metros desde el detector, este half_width
        # debe coincidir con `lane_half_width_px` del detector traducido a m.
        # 0.18 m es un default razonable para track interior del QCar.
        self.declare_parameter('lane_half_width_m', 0.18)

        # Bias lateral cuando estoy en el carril que QUIERO ESTAR
        # 0.0 = centro del carril. Subirlo acerca al divisor amarillo.
        self.declare_parameter('bias_in_target_lane', 0.0)

        # "Fuerza" extra para empujarme al carril correcto cuando estoy fuera
        # de el. Se multiplica por half_width y se suma al target_x.
        self.declare_parameter('bias_to_force_lane', 0.6)

        # Trigger / recuperacion de evasion
        self.declare_parameter('evade_trigger_distance', 0.8)   # m
        self.declare_parameter('evade_recover_distance', 1.5)   # m
        self.declare_parameter('evade_min_hold_s', 1.5)         # s
        self.declare_parameter('lidar_timeout_s', 0.5)

        # Frames estables antes de auto-detectar el preferred_lane
        self.declare_parameter('lock_lane_after_frames', 10)

        # ── Cache ────────────────────────────────────────────────────────────
        self.target_in   = self.get_parameter('target_in_topic').value
        self.target_out  = self.get_parameter('target_out_topic').value
        self.hw          = float(self.get_parameter('lane_half_width_m').value)
        self.bias_in     = float(self.get_parameter('bias_in_target_lane').value)
        self.bias_force  = float(self.get_parameter('bias_to_force_lane').value)
        self.trig_d      = float(self.get_parameter('evade_trigger_distance').value)
        self.recov_d     = float(self.get_parameter('evade_recover_distance').value)
        self.min_hold    = float(self.get_parameter('evade_min_hold_s').value)
        self.lidar_tout  = float(self.get_parameter('lidar_timeout_s').value)
        self.lock_after  = int(self.get_parameter('lock_lane_after_frames').value)

        # ── Estado ───────────────────────────────────────────────────────────
        self.preferred_lane    = None      # se autodetecta
        self._candidate_lane   = None
        self._candidate_count  = 0

        self.detector_lane     = 'unknown'
        self.lidar_min         = float('inf')
        self.lidar_angle       = 0.0
        self.lidar_obstacle    = False
        self.last_lidar_t      = 0.0

        self.evading           = False
        self.evade_lane        = None
        self.evade_start_t     = 0.0

        # ── ROS ──────────────────────────────────────────────────────────────
        self.create_subscription(Float32MultiArray, self.target_in,
                                 self._target_cb, 10)
        self.create_subscription(String,  '/lane/current_lane',
                                 self._lane_cb, 10)
        self.create_subscription(Float32, '/lidar/min_distance',
                                 self._lidar_min_cb, 10)
        self.create_subscription(Float32, '/lidar/closest_angle',
                                 self._lidar_ang_cb, 10)
        self.create_subscription(Bool,    '/lidar/obstacle',
                                 self._lidar_obs_cb, 10)

        self.target_pub = self.create_publisher(
            Float32MultiArray, self.target_out, 10)
        self.state_pub  = self.create_publisher(
            String, '/avoider/state', 10)

        self.get_logger().info(
            f'lane_obstacle_avoider iniciado | in={self.target_in} '
            f'out={self.target_out} | preferred_lane=AUTO'
        )

    # ── Callbacks ───────────────────────────────────────────────────────────
    def _lane_cb(self, msg):
        self.detector_lane = msg.data

        # Autodeteccion del preferred_lane: el primer carril ESTABLE
        if self.preferred_lane is None:
            if msg.data in ('left', 'right'):
                if msg.data == self._candidate_lane:
                    self._candidate_count += 1
                else:
                    self._candidate_lane = msg.data
                    self._candidate_count = 1

                if self._candidate_count >= self.lock_after:
                    self.preferred_lane = self._candidate_lane
                    self.get_logger().info(
                        f'preferred_lane AUTODETECTADO = {self.preferred_lane}')

    def _lidar_min_cb(self, msg):
        self.lidar_min = float(msg.data)
        self.last_lidar_t = time.time()

    def _lidar_ang_cb(self, msg):
        self.lidar_angle = float(msg.data)
        self.last_lidar_t = time.time()

    def _lidar_obs_cb(self, msg):
        self.lidar_obstacle = bool(msg.data)
        self.last_lidar_t = time.time()

    # ── Logica principal ────────────────────────────────────────────────────
    def _target_cb(self, msg):
        data = msg.data
        if len(data) < 2:
            return
        target_x = float(data[0])
        target_y = float(data[1])

        # Sentinel del detector = sin deteccion → reenvia tal cual
        if target_y <= 0.0:
            out = Float32MultiArray()
            out.data = [target_x, target_y]
            self.target_pub.publish(out)
            self._publish_state('NO_DETECTION')
            return

        # Si aun no se ha bloqueado preferred_lane, pasa el target tal cual
        if self.preferred_lane is None:
            out = Float32MultiArray()
            out.data = [target_x, target_y]
            self.target_pub.publish(out)
            self._publish_state('LEARNING_LANE')
            return

        # ── Estado del lidar ───────────────────────────────────────────────
        lidar_fresh = (time.time() - self.last_lidar_t) <= self.lidar_tout
        obstacle_front = (
            lidar_fresh and
            self.lidar_obstacle and
            abs(self.lidar_angle) <= _FRONT_CONE_RAD and
            self.lidar_min < self.trig_d
        )

        # ── Decision del carril objetivo ───────────────────────────────────
        now = time.time()
        target_lane = self.preferred_lane
        state = 'CRUISING'

        if self.evading:
            # Si todavia hay obstaculo cerca o no ha pasado el min_hold,
            # mantengo el carril de evasion.
            hold_elapsed = (now - self.evade_start_t) >= self.min_hold
            still_close  = lidar_fresh and self.lidar_min < self.recov_d
            if (not hold_elapsed) or still_close:
                target_lane = self.evade_lane
                state = 'EVADING_HOLD'
            else:
                self.evading = False
                self.evade_lane = None
                target_lane = self.preferred_lane
                state = 'RECOVERING'
        elif obstacle_front and self.detector_lane != 'unknown':
            # Obstaculo en el carril actual → cambio al opuesto
            if self.detector_lane == self.preferred_lane:
                self.evading = True
                self.evade_lane = _opposite(self.detector_lane)
                self.evade_start_t = now
                target_lane = self.evade_lane
                state = 'EVADING_START'
            else:
                # Ya estoy en el opuesto al preferido — me quedo aqui
                target_lane = self.detector_lane
                state = 'EVADING_ALREADY'

        # ── Calcular bias segun donde quiero estar vs donde estoy ──────────
        # Convencion: target_x > 0 → carro debe ir a la DERECHA
        #             target_x < 0 → carro debe ir a la IZQUIERDA
        # Si estoy en el carril correcto: pequeño bias central
        # Si estoy en el opuesto: empujar hacia donde quiero ir
        if self.detector_lane == target_lane:
            bias = self.bias_in
            sign = 0.0
        else:
            # Empuja en la direccion del carril que QUIERO
            # target_lane='left' → quiero ir a la izquierda → target_x mas negativo
            # target_lane='right' → quiero ir a la derecha → target_x mas positivo
            sign = -1.0 if target_lane == 'left' else 1.0
            bias = self.bias_force

        new_target_x = target_x + sign * bias * self.hw

        # ── Publicar ───────────────────────────────────────────────────────
        out = Float32MultiArray()
        out.data = [float(new_target_x), float(target_y)]
        self.target_pub.publish(out)
        self._publish_state(state)

    def _publish_state(self, state):
        msg = String()
        msg.data = (
            f'{state}|pref={self.preferred_lane}|cur={self.detector_lane}|'
            f'lidar_min={self.lidar_min:.2f}|obs={self.lidar_obstacle}|'
            f'ang={self.lidar_angle:+.2f}'
        )
        self.state_pub.publish(msg)


def _opposite(lane):
    return 'left' if lane == 'right' else 'right'


def main(args=None):
    rclpy.init(args=args)
    node = LaneObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
