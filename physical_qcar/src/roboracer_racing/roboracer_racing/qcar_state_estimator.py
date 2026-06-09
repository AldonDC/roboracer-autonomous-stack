#!/usr/bin/env python3
"""
QCar State Estimator — Fusión de IMU + Odometría.

Estima la pose del QCar (x, y, yaw, v) fusionando:
  - Yaw absoluto del IMU BNO055 (/imu/data, orientation quaternion)
  - Velocidad lineal del tach del QCar (/qcar/velocity)
  - Dead reckoning (bicycle model integrado a 50 Hz)

Publica:
  /qcar/odom_fused        nav_msgs/Odometry        pose + twist estimados
  tf2: odom → base_link                              para visualización RViz

Subscribe:
  /imu/data               sensor_msgs/Imu           yaw + gyro_z
  /qcar/velocity          geometry_msgs/Vector3Stamped   v lineal
  /qcar/reset_odom        std_msgs/Bool             reset a (0,0,0)

Por qué este enfoque (no EKF formal):
  El BNO055 ya hace fusión interna entre accel/gyro/magnetómetro y entrega
  un quaternion absoluto muy bueno. Reintegrar gyro externamente sería
  redundante. El tach del QCar es preciso (encoder real). Para velocidades
  bajas y trayectorias cortas, dead reckoning + yaw absoluto del IMU es
  suficientemente preciso, mucho más simple, y más fácil de debuggear.
"""

import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from sensor_msgs.msg import Imu
from geometry_msgs.msg import (Vector3Stamped, TransformStamped, Quaternion,
                                PoseStamped)
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool

# tf2_ros es opcional: si no esta instalado, el nodo sigue funcionando
# publicando /qcar/odom_fused, solo no broadcasta TF. Para RViz, el TF
# es util pero no estrictamente necesario (RViz puede usar Odometry).
try:
    from tf2_ros import TransformBroadcaster
    _HAS_TF2 = True
except ImportError:
    TransformBroadcaster = None
    _HAS_TF2 = False


def quaternion_to_yaw(qx, qy, qz, qw):
    """Extrae el angulo de yaw (Z) de un quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw_rad):
    """Convierte un angulo de yaw (rad) a quaternion plano (roll=pitch=0)."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad * 0.5)
    q.w = math.cos(yaw_rad * 0.5)
    return q


def normalize_angle(a):
    """Normaliza angulo al rango [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class QCarStateEstimator(Node):
    def __init__(self):
        super().__init__('qcar_state_estimator')

        # ── Parametros ────────────────────────────────────────────────────────
        # Frame names
        self.declare_parameter('odom_frame',      'odom')
        self.declare_parameter('base_frame',      'base_link')
        # Rate del loop de prediccion (Hz)
        self.declare_parameter('publish_rate',    50.0)
        # Cuando arranca, el yaw inicial del IMU se toma como "yaw=0" para
        # tener un sistema de coordenadas consistente con la posicion (0,0)
        self.declare_parameter('zero_yaw_on_start', True)
        # Si tu IMU esta montada rotada respecto al chasis, ajusta este offset
        self.declare_parameter('imu_yaw_offset',  0.0)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        rate            = float(self.get_parameter('publish_rate').value)
        self.zero_on_start = bool(self.get_parameter('zero_yaw_on_start').value)
        self.imu_offset = float(self.get_parameter('imu_yaw_offset').value)

        # ── Estado interno ────────────────────────────────────────────────────
        self.x   = 0.0      # posicion X (m, odom frame)
        self.y   = 0.0      # posicion Y (m, odom frame)
        self.yaw = 0.0      # orientacion (rad, odom frame)
        self.v   = 0.0      # velocidad lineal (m/s)
        self.gyro_z = 0.0   # velocidad angular Z (rad/s)

        # Yaw "raw" mas reciente del IMU (sin offset)
        self.imu_yaw_raw = None
        # Yaw inicial para usar como cero
        self.imu_yaw_origin = None

        self.last_predict_time = None

        # ── QoS ───────────────────────────────────────────────────────────────
        # IMU: RELIABLE (matcheamos lo que publica imu_bno055)
        imu_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability  = QoSDurabilityPolicy.VOLATILE,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 10,
        )
        # Velocity del QCar: RELIABLE+TRANSIENT_LOCAL (matchea qcarnode)
        vel_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability  = QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 1,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(Imu,            '/imu/data',
                                 self._imu_cb,   imu_qos)
        self.create_subscription(Vector3Stamped, '/qcar/velocity',
                                 self._vel_cb,   vel_qos)
        self.create_subscription(Bool,           '/qcar/reset_odom',
                                 self._reset_cb, 10)

        # ── Publishers ────────────────────────────────────────────────────────
        self.odom_pub = self.create_publisher(Odometry, '/qcar/odom_fused', 10)
        # Path para visualizacion en RViz (no necesita TF — funciona standalone)
        self.path_pub = self.create_publisher(Path, '/qcar/trajectory', 10)
        if _HAS_TF2:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None
            self.get_logger().warn(
                'tf2_ros NO instalado — solo se publicara /qcar/odom_fused + '
                '/qcar/trajectory. Para RViz visual completo: '
                'sudo apt install ros-dashing-tf2-ros'
            )

        # ── Buffer de trayectoria (ultimas N poses) ──────────────────────────
        # Limitado para no consumir memoria infinita. ~10 s de buffer a 50Hz.
        self._path_buffer = deque(maxlen=500)
        # Solo agregar punto al path cada cierto desplazamiento (evita ruido
        # de stationary noise inundando el path)
        self._min_path_step_m = 0.02   # 2 cm
        self._last_path_x     = 0.0
        self._last_path_y     = 0.0

        # ── Timer de prediccion + publicacion ─────────────────────────────────
        self.timer = self.create_timer(1.0 / rate, self._predict_and_publish)

        self.get_logger().info(
            f'QCar State Estimator OK | rate={rate:.0f} Hz | '
            f'frames: {self.odom_frame} → {self.base_frame}'
        )
        if self.zero_on_start:
            self.get_logger().info('Esperando primera lectura de IMU para fijar yaw=0...')

    # ──────────────────────────────────────────────────────────────────────────
    def _imu_cb(self, msg):
        """Recibe IMU: actualiza yaw absoluto y gyro_z."""
        yaw_raw = quaternion_to_yaw(
            msg.orientation.x, msg.orientation.y,
            msg.orientation.z, msg.orientation.w
        )
        self.imu_yaw_raw = yaw_raw
        self.gyro_z      = float(msg.angular_velocity.z)

        # Fijar yaw_origin la primera vez (zero on start)
        if self.zero_on_start and self.imu_yaw_origin is None:
            self.imu_yaw_origin = yaw_raw
            self.get_logger().info(
                f'Yaw inicial fijado en {math.degrees(yaw_raw):+.2f}° '
                f'(este angulo sera yaw=0 en odom_frame)'
            )

        # Actualizar yaw (con offset y origin si aplica)
        origin = self.imu_yaw_origin if self.zero_on_start else 0.0
        self.yaw = normalize_angle(yaw_raw - origin + self.imu_offset)

    # ──────────────────────────────────────────────────────────────────────────
    def _vel_cb(self, msg):
        """Recibe velocidad del QCar tach."""
        # Velocidad lineal magnitud (puede venir como vector x,y proyectado)
        self.v = math.hypot(float(msg.vector.x), float(msg.vector.y))

    # ──────────────────────────────────────────────────────────────────────────
    def _reset_cb(self, msg):
        """Reset de odometria a (0, 0, yaw_actual) y limpia trayectoria."""
        if msg.data:
            self.x = 0.0
            self.y = 0.0
            if self.zero_on_start and self.imu_yaw_raw is not None:
                self.imu_yaw_origin = self.imu_yaw_raw
                self.yaw = 0.0
            self._path_buffer.clear()
            self._last_path_x = 0.0
            self._last_path_y = 0.0
            self.get_logger().info('🔄 Odometria + trayectoria reseteadas')

    # ──────────────────────────────────────────────────────────────────────────
    def _predict_and_publish(self):
        """
        Loop principal: integra posicion usando v del tach + yaw del IMU.

        Modelo bicycle simplificado (dead reckoning):
          x_new = x + v · cos(yaw) · dt
          y_new = y + v · sin(yaw) · dt
          yaw   = yaw del IMU directamente (fusion ya hecha por BNO055)
        """
        now = self.get_clock().now()
        now_s = now.nanoseconds * 1e-9

        # Esperar a tener primera lectura del IMU
        if self.imu_yaw_raw is None:
            return

        if self.last_predict_time is not None:
            dt = now_s - self.last_predict_time
            if dt > 0.0 and dt < 1.0:   # ignora dt extremos (warmup, gap)
                # Integracion del bicycle model
                self.x += self.v * math.cos(self.yaw) * dt
                self.y += self.v * math.sin(self.yaw) * dt

        self.last_predict_time = now_s

        # ── Publicar Odometry ────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame

        odom.pose.pose.position.x  = self.x
        odom.pose.pose.position.y  = self.y
        odom.pose.pose.position.z  = 0.0
        odom.pose.pose.orientation = yaw_to_quaternion(self.yaw)

        # Twist en el frame base_link
        odom.twist.twist.linear.x  = self.v          # forward
        odom.twist.twist.angular.z = self.gyro_z     # giro

        self.odom_pub.publish(odom)

        # ── Broadcast TF odom → base_link (solo si tf2_ros disponible) ───────
        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp    = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id  = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation      = yaw_to_quaternion(self.yaw)
            self.tf_broadcaster.sendTransform(t)

        # ── Publicar Path (trayectoria acumulada) ────────────────────────────
        # Solo agregar punto si se desplazo significativamente desde el ultimo
        dx = self.x - self._last_path_x
        dy = self.y - self._last_path_y
        if math.hypot(dx, dy) >= self._min_path_step_m:
            ps = PoseStamped()
            ps.header.stamp    = now.to_msg()
            ps.header.frame_id = self.odom_frame
            ps.pose.position.x = self.x
            ps.pose.position.y = self.y
            ps.pose.position.z = 0.0
            ps.pose.orientation = yaw_to_quaternion(self.yaw)
            self._path_buffer.append(ps)
            self._last_path_x = self.x
            self._last_path_y = self.y

        # Publicar Path cada ciclo (RViz necesita el snapshot completo)
        path_msg = Path()
        path_msg.header.stamp    = now.to_msg()
        path_msg.header.frame_id = self.odom_frame
        path_msg.poses           = list(self._path_buffer)
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = QCarStateEstimator()
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
