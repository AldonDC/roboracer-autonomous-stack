"""
Gap Follower — Disparity Extender vectorizado para QCar
========================================================

Nodo de PROPUESTA, no comanda el carro directamente. Solo computa el steering
que el algoritmo "Disparity Extender" sugiere a partir del LIDAR, y publica:

    /gap/steer_cmd   (Float32, rad)  — steering propuesto, ya con steering_sign QCar
    /gap/confidence  (Float32, 0..1) — qué tan "limpio" es el gap encontrado
    /gap/distance    (Float32, m)    — distancia al target dentro del gap

El consumidor es lane_follower_pp.py: cuando `fusion_enable=True` fusiona
steer_pp (vision) con steer_gap (lidar) ponderado por la confianza de cada
modalidad. Si nadie suscribe a /gap/*, este nodo no hace nada visible.

Calibracion QCar replicada de lidar_processor:
  - flip_scan = True
  - angle_offset = -1.5708 rad (-90 deg)
  - min_valid_range = 0.15 m (descarta chasis)

Algoritmo:
  1. Filtrar rangos validos y recortar al cono frontal (default 90 deg).
  2. Detectar disparidades = saltos consecutivos > disparity_threshold.
  3. En cada disparidad, "extender" la zona peligrosa: el rayo mas lejano se
     trunca al rango del rayo cercano para una mancha angular de car_half_width
     proyectada a esa distancia.
  4. Buscar el gap (corrida de rayos cuyo rango >= safe_distance) mas profundo.
  5. Apuntar al rayo medio del gap (angulo central).
  6. Aplicar steering_sign QCar y clamp.

Vectorizado con NumPy, sin loops sobre rayos.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class GapFollower(Node):
    def __init__(self):
        super().__init__('gap_follower')

        # ── Parametros (calibracion QCar y algoritmo) ────────────────────────
        self.declare_parameter('scan_topic',          '/qcar/scan')
        self.declare_parameter('max_range',           5.0)
        self.declare_parameter('min_valid_range',     0.15)
        self.declare_parameter('flip_scan',           True)
        self.declare_parameter('angle_offset',        -1.5708)   # rad
        self.declare_parameter('front_cone_deg',      90.0)
        # Half-width efectivo del carro para inflar disparidades.
        # Ancho real ~0.19 m → half = 0.095. Subimos a 0.12 por margen.
        self.declare_parameter('car_half_width',      0.12)
        # Salto entre rayos consecutivos para considerar "disparidad".
        self.declare_parameter('disparity_threshold', 0.30)
        # Distancia objetivo del gap: rayos por debajo se descartan.
        self.declare_parameter('safe_distance',       0.80)
        # Steering del QCar
        self.declare_parameter('max_steering',        0.30)
        self.declare_parameter('steering_sign',       -1.0)

        # Cache (se leen en cada scan_cb para permitir param set en runtime)
        self._scan_topic = self.get_parameter('scan_topic').value

        # ── ROS interfaces ───────────────────────────────────────────────────
        # Mismo QoS que lidar_processor para matchear publisher RPLidar.
        qos_be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        self.create_subscription(LaserScan, self._scan_topic,
                                 self._scan_cb, qos_be)

        self.pub_steer = self.create_publisher(Float32, '/gap/steer_cmd',  10)
        self.pub_conf  = self.create_publisher(Float32, '/gap/confidence', 10)
        self.pub_dist  = self.create_publisher(Float32, '/gap/distance',   10)

        self.get_logger().info(
            f'GapFollower up, listening on {self._scan_topic}'
        )

    # ─────────────────────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return

        # Lectura en vivo de parametros
        max_range  = float(self.get_parameter('max_range').value)
        min_valid  = float(self.get_parameter('min_valid_range').value)
        flip       = bool(self.get_parameter('flip_scan').value)
        ang_off    = float(self.get_parameter('angle_offset').value)
        cone_rad   = math.radians(float(self.get_parameter('front_cone_deg').value))
        car_half_w = float(self.get_parameter('car_half_width').value)
        disp_th    = float(self.get_parameter('disparity_threshold').value)
        safe_d     = float(self.get_parameter('safe_distance').value)
        max_steer  = float(self.get_parameter('max_steering').value)
        steer_sign = float(self.get_parameter('steering_sign').value)

        # Angulos base + calibracion QCar (flip + offset, igual que lidar_processor)
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        if flip:
            angles = -angles
        angles = angles + ang_off
        angles = (angles + np.pi) % (2 * np.pi) - np.pi

        # Mascara: rayos validos dentro del cono frontal
        valid = np.isfinite(ranges) & (ranges > min_valid) & (ranges < max_range)
        front = np.abs(angles) < (cone_rad * 0.5)
        m = valid & front
        if not np.any(m):
            self._publish(0.0, 0.0, 0.0)
            return

        r = ranges[m].copy()
        a = angles[m]
        if r.size < 5:
            self._publish(0.0, 0.0, 0.0)
            return

        # Reordenar por angulo ascendente (de derecha a izquierda en frame robot)
        order = np.argsort(a)
        r = r[order]
        a = a[order]

        # ── Disparity Extender vectorizado ───────────────────────────────────
        # 1) Detectar disparidades: |r[i+1] - r[i]| > threshold
        dr = np.abs(np.diff(r))
        disp_idx = np.where(dr > disp_th)[0]   # indices de "salto"

        # 2) Inflar: en cada disparidad, truncar el lado mas lejano al rango
        #    del lado mas cercano, sobre un ancho angular suficiente para
        #    cubrir car_half_width a esa distancia.
        if disp_idx.size > 0:
            # Estimar paso angular promedio (igual para todo el array si
            # angle_increment es constante, que es lo tipico de RPLidar).
            da = float(np.mean(np.diff(a))) if a.size > 1 else 0.001
            if da <= 0.0:
                da = 0.001
            for i in disp_idx:
                near_r = min(r[i], r[i + 1])
                # numero de rayos a inflar segun arcsin
                if near_r > 1e-3:
                    bubble_rad = math.asin(min(car_half_width / near_r, 1.0))
                    n_bubble = int(math.ceil(bubble_rad / da))
                else:
                    n_bubble = 0
                if r[i] > r[i + 1]:
                    # lado izquierdo (i) es mas lejano: inflar hacia atras
                    lo = max(0, i - n_bubble)
                    r[lo:i + 1] = np.minimum(r[lo:i + 1], near_r)
                else:
                    hi = min(r.size, i + 1 + n_bubble + 1)
                    r[i + 1:hi] = np.minimum(r[i + 1:hi], near_r)

        # 3) Encontrar el gap mas profundo: rayos con r >= safe_d
        gap_mask = r >= safe_d
        if not np.any(gap_mask):
            # No hay gap "seguro" — apuntar al rayo mas lejano disponible
            best_i = int(np.argmax(r))
            target_a = float(a[best_i])
            target_d = float(r[best_i])
            steer = self._steer_from_angle(target_a, steer_sign, max_steer)
            # Confianza muy baja en este modo (no hay gap real)
            conf = 0.1 * min(target_d / max_range, 1.0)
            self._publish(steer, conf, target_d)
            return

        # 4) Detectar corridas de True en gap_mask (segmentos contiguos de gap)
        # Trick vectorial: usar diff sobre mascara para encontrar bordes
        edges = np.diff(np.concatenate(([0], gap_mask.astype(np.int8), [0])))
        starts = np.where(edges == 1)[0]
        ends = np.where(edges == -1)[0]    # exclusive
        # Elegir el segmento con mayor profundidad promedio (no solo el mas
        # ancho — preferimos un gap profundo aunque sea angularmente angosto)
        best_score = -1.0
        best_seg = None
        for s, e in zip(starts, ends):
            depth_avg = float(np.mean(r[s:e]))
            width_n = e - s
            score = depth_avg * math.sqrt(max(width_n, 1))
            if score > best_score:
                best_score = score
                best_seg = (s, e, depth_avg, width_n)

        if best_seg is None:
            self._publish(0.0, 0.0, 0.0)
            return

        s, e, depth_avg, width_n = best_seg
        # Apuntar al medio del segmento
        mid = (s + e - 1) // 2
        target_a = float(a[mid])
        target_d = float(r[mid])

        # 5) Convertir angulo objetivo a steering QCar
        steer = self._steer_from_angle(target_a, steer_sign, max_steer)

        # 6) Confianza: profundidad_norm * ancho_norm
        depth_ratio = min(depth_avg / max_range, 1.0)
        # Convertir ancho de rayos a metros aproximados (usando angulo total
        # del segmento * distancia promedio)
        da = float(np.mean(np.diff(a))) if a.size > 1 else 0.001
        gap_width_m = depth_avg * da * width_n
        gap_width_norm = min(gap_width_m / 0.5, 1.0)
        conf = float(depth_ratio * gap_width_norm)

        self._publish(steer, conf, target_d)

    # ─────────────────────────────────────────────────────────────────────────
    def _steer_from_angle(self, target_a, steer_sign, max_steer):
        """Convierte angulo objetivo del gap (frame robot, 0=adelante,
        positivo=derecha si fuera del flip) en steering del QCar.

        En el frame del scan procesado: angles 0=adelante, +=derecha.
        Pure-Pursuit-style mapping: clamp del propio angulo del target.
        """
        steer = steer_sign * float(target_a)
        if steer > max_steer:
            steer = max_steer
        elif steer < -max_steer:
            steer = -max_steer
        return float(steer)

    def _publish(self, steer, conf, dist):
        self.pub_steer.publish(Float32(data=float(steer)))
        self.pub_conf.publish(Float32(data=float(conf)))
        self.pub_dist.publish(Float32(data=float(dist)))


def main(args=None):
    rclpy.init(args=args)
    node = GapFollower()
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
