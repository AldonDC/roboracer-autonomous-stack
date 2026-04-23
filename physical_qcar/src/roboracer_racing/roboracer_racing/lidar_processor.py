"""
LIDAR Processor — QCar fisico
==============================
Suscribe a /qcar/scan (sensor_msgs/LaserScan) y publica:
  - /lidar/image_debug  (CompressedImage) : render top-down en JPEG
  - /lidar/min_distance (Float32)         : distancia minima frontal (m)
  - /lidar/closest_angle(Float32)         : angulo del obstaculo mas cercano (rad)
  - /lidar/obstacle     (Bool)            : True si hay obstaculo < umbral
"""
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan, CompressedImage
from std_msgs.msg import Float32, Bool


IMG_SIZE = 400          # lienzo cuadrado (px)
MAX_RANGE_M = 5.0       # rango maximo visible (m)
FRONT_CONE_DEG = 60.0   # cono frontal para "min distance"
OBSTACLE_THRESH = 0.6   # m para flag de obstaculo


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')

        qos_be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            history=QoSHistoryPolicy.KEEP_LAST, depth=2)

        self.declare_parameter('scan_topic', '/qcar/scan')
        self.declare_parameter('max_range', MAX_RANGE_M)
        self.declare_parameter('front_cone_deg', FRONT_CONE_DEG)
        self.declare_parameter('obstacle_thresh', OBSTACLE_THRESH)

        scan_topic = self.get_parameter('scan_topic').value
        self.max_range = float(self.get_parameter('max_range').value)
        self.front_cone = math.radians(float(self.get_parameter('front_cone_deg').value))
        self.obs_th = float(self.get_parameter('obstacle_thresh').value)

        self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos_be)

        self.pub_img = self.create_publisher(CompressedImage, '/lidar/image_debug', qos_be)
        self.pub_min = self.create_publisher(Float32, '/lidar/min_distance', 10)
        self.pub_ang = self.create_publisher(Float32, '/lidar/closest_angle', 10)
        self.pub_obs = self.create_publisher(Bool, '/lidar/obstacle', 10)

        self.get_logger().info(f'LIDAR Processor escuchando {scan_topic}')

    def _scan_cb(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return

        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)

        # ── Minima distancia en cono frontal ──────────────────────────────────
        front_mask = valid & (np.abs(angles) < self.front_cone / 2.0)
        if np.any(front_mask):
            idx = np.argmin(np.where(front_mask, ranges, np.inf))
            d_min = float(ranges[idx])
            a_min = float(angles[idx])
        else:
            d_min = float('inf')
            a_min = 0.0

        self.pub_min.publish(Float32(data=(d_min if np.isfinite(d_min) else -1.0)))
        self.pub_ang.publish(Float32(data=a_min))
        self.pub_obs.publish(Bool(data=bool(np.isfinite(d_min) and d_min < self.obs_th)))

        # ── Render top-down ───────────────────────────────────────────────────
        img = self._render(ranges, angles, valid, d_min, a_min)

        # Encode JPEG
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.pub_img.publish(out)

    def _render(self, ranges, angles, valid, d_min, a_min):
        S = IMG_SIZE
        cx = cy = S // 2
        img = np.full((S, S, 3), 18, dtype=np.uint8)  # fondo oscuro

        # Anillos de distancia (1, 2, 3, ... m)
        px_per_m = (S * 0.45) / self.max_range
        for r_m in range(1, int(self.max_range) + 1):
            r_px = int(r_m * px_per_m)
            cv2.circle(img, (cx, cy), r_px, (60, 60, 60), 1, cv2.LINE_AA)
            cv2.putText(img, f'{r_m}m', (cx + r_px + 2, cy - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (120, 120, 120), 1, cv2.LINE_AA)

        # Ejes
        cv2.line(img, (cx, 0), (cx, S), (45, 45, 45), 1)
        cv2.line(img, (0, cy), (S, cy), (45, 45, 45), 1)

        # Cono frontal
        half = self.front_cone / 2.0
        r_cone = int(self.max_range * px_per_m)
        p1 = (int(cx + r_cone * math.sin(-half)), int(cy - r_cone * math.cos(-half)))
        p2 = (int(cx + r_cone * math.sin(+half)), int(cy - r_cone * math.cos(+half)))
        overlay = img.copy()
        cv2.fillPoly(overlay, [np.array([(cx, cy), p1, p2], dtype=np.int32)],
                     (0, 60, 70))
        cv2.addWeighted(overlay, 0.25, img, 0.75, 0, img)

        # Puntos LIDAR
        r = ranges[valid]
        a = angles[valid]
        # Convencion: angulo 0 hacia adelante (eje +X del carro) → arriba de la imagen.
        # y hacia "adelante" es -Y pixel. Rotacion: x = r*sin(a), y = -r*cos(a)
        xs = (cx + r * np.sin(a) * px_per_m).astype(np.int32)
        ys = (cy - r * np.cos(a) * px_per_m).astype(np.int32)
        m = (xs >= 0) & (xs < S) & (ys >= 0) & (ys < S)
        xs = xs[m]; ys = ys[m]; rr = r[m]

        # Color por cercania: cerca = rojo, lejos = cyan
        if rr.size:
            t = np.clip(rr / self.max_range, 0.0, 1.0)
            col_b = (t * 230 + 20).astype(np.uint8)       # azul sube con distancia
            col_g = (t * 200 + 40).astype(np.uint8)
            col_r = ((1.0 - t) * 230 + 25).astype(np.uint8)  # rojo sube con cercania
            for x, y, b, g, rch in zip(xs, ys, col_b, col_g, col_r):
                img[y, x] = (int(b), int(g), int(rch))
            # Engrosar
            img = cv2.dilate(img, np.ones((2, 2), np.uint8))

        # Carro (triangulo)
        car = np.array([(cx, cy - 10), (cx - 7, cy + 8), (cx + 7, cy + 8)], dtype=np.int32)
        cv2.fillPoly(img, [car], (0, 230, 204))

        # Marca obstaculo mas cercano
        if np.isfinite(d_min) and d_min < self.max_range:
            ox = int(cx + d_min * math.sin(a_min) * px_per_m)
            oy = int(cy - d_min * math.cos(a_min) * px_per_m)
            col = (0, 68, 255) if d_min < self.obs_th else (50, 179, 227)
            cv2.circle(img, (ox, oy), 6, col, 2, cv2.LINE_AA)
            cv2.line(img, (cx, cy), (ox, oy), col, 1, cv2.LINE_AA)

        # HUD
        txt = f'MIN: {d_min:.2f} m' if np.isfinite(d_min) else 'MIN: ---'
        cv2.putText(img, txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(img, f'ANG: {math.degrees(a_min):+.1f}°',
                    (8, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (230, 230, 230), 1, cv2.LINE_AA)

        return img


def main():
    rclpy.init()
    node = LidarProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
