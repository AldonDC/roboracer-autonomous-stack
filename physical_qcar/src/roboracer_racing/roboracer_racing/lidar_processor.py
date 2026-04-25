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
        self.declare_parameter('angle_offset', -1.5708)  # -90 deg to align after flip
        self.declare_parameter('flip_scan', True)      # Mirrored Lidar fix

        scan_topic = self.get_parameter('scan_topic').value
        self.max_range = float(self.get_parameter('max_range').value)
        self.front_cone = math.radians(float(self.get_parameter('front_cone_deg').value))
        self.obs_th = float(self.get_parameter('obstacle_thresh').value)
        self.angle_offset = float(self.get_parameter('angle_offset').value)
        self.flip_scan = bool(self.get_parameter('flip_scan').value)

        self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos_be)

        self.pub_img = self.create_publisher(CompressedImage, '/lidar/image_debug', qos_be)
        self.pub_min = self.create_publisher(Float32, '/lidar/min_distance', 10)
        self.pub_ang = self.create_publisher(Float32, '/lidar/closest_angle', 10)
        self.pub_obs = self.create_publisher(Bool, '/lidar/obstacle', 10)

        self.get_logger().info(f'LIDAR Processor (RViz Style) escuchando {scan_topic} (Flipped: {self.flip_scan})')

    def _scan_cb(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return

        # Calcular angulos base
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment
        
        # Invertir eje horizontal si el scan esta espejado (comun en montajes invertidos)
        if self.flip_scan:
            angles = -angles
            
        # Aplicar offset de orientacion y normalizar a [-pi, pi]
        angles = angles + self.angle_offset
        angles = (angles + np.pi) % (2 * np.pi) - np.pi

        # Filtrar puntos muy cercanos (chasis del QCar) y validos
        valid = np.isfinite(ranges) & (ranges > 0.35) & (ranges < self.max_range)

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

        # ── Render top-down (RViz Style) ──────────────────────────────────────
        img = self._render_rviz(ranges, angles, valid, d_min, a_min)

        # Encode JPEG
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.pub_img.publish(out)

    def _render_rviz(self, ranges, angles, valid, d_min, a_min):
        S = IMG_SIZE
        cx = cy = S // 2
        # Color fondo RViz (gris oscuro tecnico)
        img = np.full((S, S, 3), (45, 45, 45), dtype=np.uint8)

        px_per_m = (S * 0.45) / self.max_range

        # ── GRID CUADRADO (Estilo RViz) ───────────────────────────────────────
        grid_step_m = 1.0
        grid_color = (60, 60, 60)
        for i in range(-int(self.max_range), int(self.max_range) + 1):
            x = int(cx + i * grid_step_m * px_per_m)
            if 0 <= x < S: cv2.line(img, (x, 0), (x, S), grid_color, 1)
            y = int(cy + i * grid_step_m * px_per_m)
            if 0 <= y < S: cv2.line(img, (0, y), (S, y), grid_color, 1)

        # Anillos circulares tenues
        for r_m in range(1, int(self.max_range) + 1):
            r_px = int(r_m * px_per_m)
            cv2.circle(img, (cx, cy), r_px, (75, 75, 75), 1, cv2.LINE_AA)
            cv2.putText(img, f'{r_m}m', (cx + r_px + 2, cy - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (110, 110, 110), 1, cv2.LINE_AA)

        # Ejes principales (X-Y)
        cv2.line(img, (cx, 0), (cx, S), (100, 100, 100), 1)
        cv2.line(img, (0, cy), (S, cy), (100, 100, 100), 1)

        # ── CONO DE SEGURIDAD ─────────────────────────────────────────────────
        half = self.front_cone / 2.0
        r_cone = int(self.max_range * px_per_m)
        p1 = (int(cx + r_cone * math.sin(-half)), int(cy - r_cone * math.cos(-half)))
        p2 = (int(cx + r_cone * math.sin(+half)), int(cy - r_cone * math.cos(+half)))
        
        overlay = img.copy()
        pts = np.array([(cx, cy), p1, p2], dtype=np.int32)
        cv2.fillPoly(overlay, [pts], (0, 200, 255)) 
        cv2.addWeighted(overlay, 0.15, img, 0.85, 0, img)
        cv2.line(img, (cx, cy), p1, (0, 180, 220), 1, cv2.LINE_AA)
        cv2.line(img, (cx, cy), p2, (0, 180, 220), 1, cv2.LINE_AA)

        # ── PUNTOS LIDAR (Squares like RViz) ──────────────────────────────────
        r = ranges[valid]
        a = angles[valid]
        xs = (cx + r * np.sin(a) * px_per_m).astype(np.int32)
        ys = (cy - r * np.cos(a) * px_per_m).astype(np.int32)
        m = (xs >= 2) & (xs < S-2) & (ys >= 2) & (ys < S-2)
        xs = xs[m]; ys = ys[m]; rr = r[m]

        if rr.size:
            t = np.clip(rr / self.max_range, 0.0, 1.0)
            for x, y, dist_t in zip(xs, ys, t):
                if dist_t < 0.3:   c = (0, 0, 255)    # Rojo
                elif dist_t < 0.6: c = (0, 255, 255)  # Amarillo
                elif dist_t < 0.8: c = (0, 255, 0)    # Verde
                else:              c = (255, 200, 0)  # Cyan/Azul
                cv2.rectangle(img, (x-1, y-1), (x+1, y+1), c, -1)

        # ── CARRO / ROBOT ─────────────────────────────────────────────────────
        rw, rh = 14, 22
        cv2.rectangle(img, (cx-rw//2, cy-rh//2), (cx+rw//2, cy+rh//2), (220, 220, 220), -1)
        # Flecha de orientacion apuntando al FRENTE (Up)
        cv2.arrowedLine(img, (cx, cy + 5), (cx, cy - 8), (50, 50, 50), 2, tipLength=0.4)

        # ── OBSTACULO MAS CERCANO ─────────────────────────────────────────────
        if np.isfinite(d_min) and d_min < self.max_range:
            ox = int(cx + d_min * math.sin(a_min) * px_per_m)
            oy = int(cy - d_min * math.cos(a_min) * px_per_m)
            is_obs = d_min < self.obs_th
            color = (0, 0, 255) if is_obs else (0, 255, 0)
            cv2.drawMarker(img, (ox, oy), color, cv2.MARKER_TILTED_CROSS, 10, 2)
            cv2.line(img, (cx, cy), (ox, oy), color, 1, cv2.LINE_4)
            cv2.putText(img, f"{d_min:.2f}m", (ox + 8, oy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

        # HUD Technical
        cv2.putText(img, "TOP-DOWN LIDAR VIEW", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(img, f"FIXED FRAME: base_link", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

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
