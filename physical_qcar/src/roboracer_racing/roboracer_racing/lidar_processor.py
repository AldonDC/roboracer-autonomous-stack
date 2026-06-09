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
# Cono frontal a 90 deg (±45°): cobertura amplia para detectar QCars u
# objetos con offset lateral. Ajustar en vivo con
#   ros2 param set /lidar_processor front_cone_deg <valor>
FRONT_CONE_DEG = 90.0   # cono frontal para "min distance"
OBSTACLE_THRESH = 0.45  # m para flag de obstaculo y para color rojo en HUD
# Distancia minima de range valido. Bajado de 0.35 a 0.15 para detectar
# objetos cercanos (otro QCar, caja). El chasis del QCar empieza alrededor
# de los 12 cm desde el centro del lidar — 0.15 lo descarta.
MIN_VALID_RANGE = 0.15
# Percentil usado como min_distance (en vez de argmin puro). Asi un rayo
# espurio aislado no dispara el freno. 5° percentil = ignora el 5% mas
# cercano (probable outlier) y toma como min el siguiente.
MIN_PERCENTILE = 5.0


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
        self.declare_parameter('min_valid_range', MIN_VALID_RANGE)
        self.declare_parameter('min_percentile', MIN_PERCENTILE)
        # Cono central (front): cuña frontal de ±(front_center_deg/2).
        # Default front_center=30 → cono frontal de ±15°.
        self.declare_parameter('front_center_deg', 30.0)
        # Borde EXTERIOR de los sectores laterales (las cuñas NARANJAS del overlay).
        # Cada lado cubre desde front_center_deg/2 hasta side_outer_deg.
        # Subir = detecta objetos mas hacia el costado/atras. (antes 85° fijo)
        self.declare_parameter('side_outer_deg', 85.0)
        # Cono TRASERO (back). Cobertura simetrica al frontal pero detras.
        # Sirve para detectar objetos pegados a la parte trasera (otro QCar,
        # caja, persona). Solo este sector cuenta — el resto del 360 se ignora.
        self.declare_parameter('back_cone_deg', 60.0)
        # ── CIRCULOS DE SEGURIDAD (omnidireccionales en capas) ──────────────
        # Dos burbujas concentricas alrededor del coche:
        #  - safety_circle_slow_radius (exterior, 35 cm): SLOW — reduce
        #    velocidad cuando un objeto se acerca. Aviso temprano.
        #  - safety_circle_radius (interior, 30 cm): STOP duro — frena
        #    en seco como red de seguridad final. Validado AL ULTIMO.
        # Ambos NO respetan is_wall (son absolutos).
        # El follower tiene parametros equivalentes con los mismos defaults.
        self.declare_parameter('safety_circle_radius',      0.30)
        self.declare_parameter('safety_circle_slow_radius', 0.35)
        self.declare_parameter('safety_circle_radius_side', 0.18)
        # ── Deteccion de PARED envolvente (vallas alrededor de la pista) ─
        # Si N sectores ven simultaneamente algo a distancia parecida, es
        # una pared, no un obstaculo real. Se publica /lidar/is_wall.
        self.declare_parameter('wall_trigger_distance',   1.50)   # m
        self.declare_parameter('wall_sectors_required',   2)      # 2 o 3
        self.declare_parameter('wall_distance_tolerance', 0.40)   # m
        self.declare_parameter('safety_width',            0.36)   # m (ancho del pasillo frontal de seguridad)
        self.declare_parameter('safety_side_max',         1.00)   # m (rango lateral maximo de interes)

        # ── MODO PISTA CERRADA (assume_walled_track) ────────────────────────
        # Para pistas rodeadas por vallas antichoques (pista cuadrada):
        # cuando True, el lidar asume que TODO lo que ve es la pared, salvo
        # que aparezca algo REALMENTE cerca (< near_obstacle_threshold).
        # is_wall se reporta como True por default; solo False si hay un
        # objeto real bajo el umbral cercano.
        # Esto evita que asimetrias geometricas (curvas, esquinas) confundan
        # al detector standard de 'N sectores a distancia parecida'.
        # Default True: la pista de pruebas tiene vallas. Si lo necesitas
        # apagar (otra pista o pista abierta): ros2 param set ... False
        self.declare_parameter('assume_walled_track',     True)   # antes False
        # Bajado a 0.40 (antes 0.55) porque las vallas laterales de pista
        # cuadrada estan tipicamente a 0.40-0.50 m. Con 0.55 daban falso
        # positivo de "objeto real" y rompian la deteccion de wall.
        self.declare_parameter('near_obstacle_threshold', 0.40)   # antes 0.55

        scan_topic = self.get_parameter('scan_topic').value
        self.max_range = float(self.get_parameter('max_range').value)
        # NOTA: front_cone, min_valid_range, min_percentile y obstacle_thresh
        # se leen EN VIVO en _scan_cb para que `ros2 param set` haga efecto
        # sin reiniciar el nodo.
        self.angle_offset = float(self.get_parameter('angle_offset').value)
        self.flip_scan = bool(self.get_parameter('flip_scan').value)

        self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos_be)

        self.pub_img = self.create_publisher(CompressedImage, '/lidar/image_debug', qos_be)
        self.pub_min = self.create_publisher(Float32, '/lidar/min_distance', 10)
        self.pub_ang = self.create_publisher(Float32, '/lidar/closest_angle', 10)
        self.pub_obs = self.create_publisher(Bool, '/lidar/obstacle', 10)
        # Sectores: front (cono central) + left + right (lados) + back (trasero)
        self.pub_front = self.create_publisher(Float32, '/lidar/min_distance_front', 10)
        self.pub_left  = self.create_publisher(Float32, '/lidar/min_distance_left',  10)
        self.pub_right = self.create_publisher(Float32, '/lidar/min_distance_right', 10)
        self.pub_back  = self.create_publisher(Float32, '/lidar/min_distance_back',  10)
        # Circulo de seguridad 360° (omnidirecional)
        self.pub_360   = self.create_publisher(Float32, '/lidar/min_distance_360',   10)
        # Flag de pared envolvente (vallas)
        self.pub_wall  = self.create_publisher(Bool, '/lidar/is_wall', 10)
        # Distancia minima en cono central estrecho de 10 grados (±5°)
        self.pub_center_10deg = self.create_publisher(Float32, '/lidar/min_distance_center_10deg', 10)

        # Estado previo de pared para aplicar histéresis
        self.last_is_wall = True
        self.wall_filter_counter = 0

        self.get_logger().info(f'LIDAR Processor (RViz Style) escuchando {scan_topic} (Flipped: {self.flip_scan})')

    def _scan_cb(self, msg: LaserScan):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        n = ranges.size
        if n == 0:
            return

        # Lectura en vivo de los parametros tunables
        min_valid  = float(self.get_parameter('min_valid_range').value)
        obs_th     = float(self.get_parameter('obstacle_thresh').value)
        pct        = float(self.get_parameter('min_percentile').value)
        safety_w   = float(self.get_parameter('safety_width').value)
        side_max   = float(self.get_parameter('safety_side_max').value)
        half_w     = safety_w / 2.0

        # Calcular angulos base
        angles = msg.angle_min + np.arange(n, dtype=np.float32) * msg.angle_increment

        # Invertir eje horizontal si el scan esta espejado (comun en montajes invertidos)
        if self.flip_scan:
            angles = -angles

        # Aplicar offset de orientacion y normalizar a [-pi, pi]
        angles = angles + self.angle_offset
        angles = (angles + np.pi) % (2 * np.pi) - np.pi

        # Filtrar: validos, no infinitos, fuera del chasis y dentro de max_range
        valid = np.isfinite(ranges) & (ranges > min_valid) & (ranges < self.max_range)

        # Proyectar a coordenadas cartesianas (en metros en el frame del vehiculo)
        xs = ranges * np.cos(angles)  # Adelante (+) / Atras (-)
        ys = ranges * np.sin(angles)  # Izquierda (+) / Derecha (-) en ROS standard

        # Leer parámetros angulares para definición de sectores radiales (cuñas)
        cone_half_deg = float(self.get_parameter('front_cone_deg').value) / 2.0
        center_half_deg = float(self.get_parameter('front_center_deg').value) / 2.0
        back_cone_deg_v = float(self.get_parameter('back_cone_deg').value)
        center_half_deg = min(center_half_deg, cone_half_deg)

        cone_half = math.radians(cone_half_deg)
        center_half = math.radians(center_half_deg)
        back_half = math.radians(back_cone_deg_v) / 2.0
        side_outer = math.radians(float(self.get_parameter('side_outer_deg').value))

        # Sector Frontal (cuña central delantera)
        front_mask = valid & (angles >= -center_half) & (angles <= center_half)

        # Sector Izquierdo (cuña lateral izquierda)
        left_mask = valid & (angles > center_half) & (angles <= side_outer)

        # Sector Derecho (cuña lateral derecha)
        right_mask = valid & (angles >= -side_outer) & (angles < -center_half)

        # Sector Trasero (cuña trasera con wrap-around en ±pi)
        back_mask = valid & ((angles >= np.pi - back_half) | (angles <= -np.pi + back_half))

        # Sector Central Estrecho de 10 grados (±5°)
        center_10deg_mask = valid & (angles >= -math.radians(5.0)) & (angles <= math.radians(5.0))

        def _get_sector_min(mask):
            if not np.any(mask):
                return float('inf')
            vals = ranges[mask]
            if vals.size <= 3:
                return float(vals.min())
            return float(np.percentile(vals, pct))

        d_front = _get_sector_min(front_mask)
        d_left  = _get_sector_min(left_mask)
        d_right = _get_sector_min(right_mask)
        d_back  = _get_sector_min(back_mask)
        d_center_10deg = _get_sector_min(center_10deg_mask)

        # ── CIRCULO DE SEGURIDAD ELÍPTICO (360° adaptativo) ──────────────────
        # Distancia mínima a cualquier punto válido alrededor del coche
        # usando una elipse para proteger más el frente y menos los laterales.
        safety_r = float(self.get_parameter('safety_circle_radius').value)
        safety_r_side = float(self.get_parameter('safety_circle_radius_side').value)
        if safety_r_side <= 0.01:
            safety_r_side = 0.18
        ellipse_scale = safety_r / safety_r_side

        if np.any(valid):
            # Distancia elíptica equivalente: escala la componente Y (lateral)
            d_360 = float(np.sqrt(xs[valid]**2 + (ellipse_scale * ys[valid])**2).min())
        else:
            d_360 = float('inf')

        # Para el HUD y diagnostico general (punto más cercano en el frente):
        if np.any(front_mask):
            front_ranges = ranges[front_mask]
            idx_in_front = np.argmin(front_ranges)
            d_min = float(front_ranges[idx_in_front])
            a_min = float(angles[front_mask][idx_in_front])
        else:
            d_min = float('inf')
            a_min = 0.0

        # ── Deteccion de pared envolvente (Híbrida y Robusta) ────────────────
        wall_d   = float(self.get_parameter('wall_trigger_distance').value)
        wall_n   = int(self.get_parameter('wall_sectors_required').value)
        wall_tol = float(self.get_parameter('wall_distance_tolerance').value)
        walled_track = bool(self.get_parameter('assume_walled_track').value)
        near_thr     = float(self.get_parameter('near_obstacle_threshold').value)

        # Lista de distancias "viendo algo" (finitas y dentro del rango de pared)
        seeing = []
        for d in (d_front, d_left, d_right, d_back):
            if np.isfinite(d) and d < wall_d:
                seeing.append(d)

        # 1. Uniformidad de sectores (standard): si varios sectores ven algo a distancia similar.
        is_wall_std = False
        if len(seeing) >= wall_n:
            if (max(seeing) - min(seeing)) <= wall_tol:
                is_wall_std = True

        # 2. Heurística lateral: si el obstáculo más cercano está en los lados y el frente está despejado.
        is_wall_lateral = False
        min_dist = min(seeing) if seeing else float('inf')
        
        if walled_track:
            if min_dist >= near_thr:
                # Si todo está lejos, es la valla/pared
                is_wall_lateral = True
            elif (not np.isfinite(d_front) or d_front > near_thr) and (d_left < near_thr or d_right < near_thr):
                # Si el frente está libre pero hay paredes cerca en los lados, es vallas laterales en curva
                is_wall_lateral = True
                
        is_wall_raw = is_wall_std or is_wall_lateral

        # Filtro de histéresis temporal: para considerar que perdimos la pared,
        # requerimos 5 scans consecutivos (~100 ms a 50Hz) con False.
        if is_wall_raw:
            self.last_is_wall = True
            self.wall_filter_counter = 0
        else:
            self.wall_filter_counter += 1
            if self.wall_filter_counter >= 5:
                self.last_is_wall = False
                self.wall_filter_counter = 0

        is_wall = self.last_is_wall

        self.pub_min.publish(Float32(data=(d_min if np.isfinite(d_min) else -1.0)))
        self.pub_ang.publish(Float32(data=a_min))
        self.pub_obs.publish(Bool(data=bool(np.isfinite(d_min) and d_min < obs_th)))
        self.pub_front.publish(Float32(data=(d_front if np.isfinite(d_front) else -1.0)))
        self.pub_left.publish (Float32(data=(d_left  if np.isfinite(d_left)  else -1.0)))
        self.pub_right.publish(Float32(data=(d_right if np.isfinite(d_right) else -1.0)))
        self.pub_back.publish (Float32(data=(d_back  if np.isfinite(d_back)  else -1.0)))
        self.pub_360.publish  (Float32(data=(d_360   if np.isfinite(d_360)   else -1.0)))
        self.pub_wall.publish(Bool(data=is_wall))
        self.pub_center_10deg.publish(Float32(data=(d_center_10deg if np.isfinite(d_center_10deg) else -1.0)))

        # ── Render top-down (RViz Style) ──────────────────────────────────────
        img = self._render_rviz(ranges, angles, valid, d_min, a_min,
                                 d_front, d_left, d_right, is_wall, d_back,
                                 d_360, d_center_10deg)

        # Encode JPEG
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            return
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self.pub_img.publish(out)

    def _render_rviz(self, ranges, angles, valid, d_min, a_min,
                     d_front=float('inf'), d_left=float('inf'),
                     d_right=float('inf'), is_wall=False,
                     d_back=float('inf'), d_360=float('inf'),
                     d_center_10deg=float('inf')):
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

        # ── 4 SECTORES DE SEGURIDAD (ANGULAR / cuñas radiales) ──────────────
        # Cuñas tipo "pie slice" que salen del centro del coche con ángulos
        # configurables. Mas natural que rectangulos cartesianos.
        # Mantenemos safety_w y side_max declarados para retro-compatibilidad
        # del HUD pero NO los usamos para el dibujo.
        safety_w = float(self.get_parameter('safety_width').value)
        side_max = float(self.get_parameter('safety_side_max').value)

        cone_half_deg   = float(self.get_parameter('front_cone_deg').value) / 2.0
        center_half_deg = float(self.get_parameter('front_center_deg').value) / 2.0
        back_cone_deg_v = float(self.get_parameter('back_cone_deg').value)
        center_half_deg = min(center_half_deg, cone_half_deg)

        cone_half   = math.radians(cone_half_deg)
        center_half = math.radians(center_half_deg)
        back_half   = math.radians(back_cone_deg_v) / 2.0
        side_outer  = math.radians(float(self.get_parameter('side_outer_deg').value))
        r_cone = int(self.max_range * px_per_m)

        # Helper para construir poligono de un sector entre dos angulos
        # Convencion de dibujo no-espejada: angle 0 = arriba (frente), positivo = izquierda.
        def _sector_poly(a_lo, a_hi, n=16):
            pts = [(cx, cy)]
            for i in range(n + 1):
                a = a_lo + (a_hi - a_lo) * (i / n)
                pts.append((int(cx - r_cone * math.sin(a)),
                            int(cy - r_cone * math.cos(a))))
            return np.array(pts, dtype=np.int32)

        overlay = img.copy()
        # Sector FRONTAL (cuña centrada al frente) - Cyan
        cv2.fillPoly(overlay, [_sector_poly(-center_half, +center_half)],
                     (0, 200, 255))
        # Sector IZQUIERDO (entre center_half y side_outer hacia la izq) - Naranja
        cv2.fillPoly(overlay, [_sector_poly(+center_half, +side_outer)],
                     (50, 130, 230))
        # Sector DERECHO - Naranja
        cv2.fillPoly(overlay, [_sector_poly(-side_outer, -center_half)],
                     (50, 130, 230))
        # Sector TRASERO (cuña centrada atras, alrededor de ±pi) - Magenta
        # Dibujamos dos mitades para evitar el wrap-around de angulos.
        cv2.fillPoly(overlay, [_sector_poly(math.pi - back_half, math.pi)],
                     (180, 60, 200))
        cv2.fillPoly(overlay, [_sector_poly(-math.pi, -math.pi + back_half)],
                     (180, 60, 200))

        # Sector Central Estrecho de 10 grados (±5°) - Amarillo
        center_10deg_rad = math.radians(5.0)
        cv2.fillPoly(overlay, [_sector_poly(-center_10deg_rad, +center_10deg_rad)],
                     (0, 255, 255))

        cv2.addWeighted(overlay, 0.18, img, 0.82, 0, img)

        # Bordes radiales (lineas del centro al borde)
        for a in (-side_outer, -center_half, +center_half, +side_outer):
            p = (int(cx - r_cone * math.sin(a)),
                 int(cy - r_cone * math.cos(a)))
            color = (0, 180, 220) if abs(a) <= center_half + 1e-3 else (50, 130, 200)
            cv2.line(img, (cx, cy), p, color, 1, cv2.LINE_AA)
        for a in (math.pi - back_half, -math.pi + back_half):
            p = (int(cx - r_cone * math.sin(a)),
                 int(cy - r_cone * math.cos(a)))
            cv2.line(img, (cx, cy), p, (180, 60, 200), 1, cv2.LINE_AA)
        for a in (-center_10deg_rad, +center_10deg_rad):
            p = (int(cx - r_cone * math.sin(a)),
                 int(cy - r_cone * math.cos(a)))
            cv2.line(img, (cx, cy), p, (0, 255, 255), 1, cv2.LINE_AA)

        # ── PUNTOS LIDAR (Squares like RViz) ──────────────────────────────────
        r = ranges[valid]
        a = angles[valid]
        xs = (cx - r * np.sin(a) * px_per_m).astype(np.int32)
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
        # ── BURBUJAS ELÍPTICAS DE SEGURIDAD (omnidireccionales en capas) ──────
        # EXTERIOR (slow): reduce velocidad al detectar.
        # INTERIOR (stop): frena en seco como red de seguridad final.
        safety_r      = float(self.get_parameter('safety_circle_radius').value)
        safety_slow_r = float(self.get_parameter('safety_circle_slow_radius').value)
        safety_r_side = float(self.get_parameter('safety_circle_radius_side').value)
        if safety_r_side <= 0.01:
            safety_r_side = 0.18

        ellipse_scale = safety_r / safety_r_side
        safety_slow_r_side = safety_slow_r / ellipse_scale

        axes_stop = (int(safety_r_side * px_per_m), int(safety_r * px_per_m))
        axes_slow = (int(safety_slow_r_side * px_per_m), int(safety_slow_r * px_per_m))

        safety_invaded     = np.isfinite(d_360) and d_360 < safety_r
        safety_slow_invaded = np.isfinite(d_360) and d_360 < safety_slow_r

        # Elipse exterior — amarilla si libre, naranja si invadido
        # (pero no llego al interior). Si llego al interior, lo dibujamos rojo.
        if safety_invaded:
            slow_color = (0, 80, 255)   # rojo-naranja
        elif safety_slow_invaded:
            slow_color = (0, 165, 255)  # naranja
        else:
            slow_color = (0, 220, 220)  # amarillo
        cv2.ellipse(img, (cx, cy), axes_slow, 0, 0, 360, slow_color, 1, cv2.LINE_AA)

        # Elipse interior — blanca si libre, roja si invadido
        safety_color = (0, 0, 255) if safety_invaded else (255, 255, 255)
        cv2.ellipse(img, (cx, cy), axes_stop, 0, 0, 360, safety_color, 1, cv2.LINE_AA)
        
        # El radio vertical se usa para posicionar el texto
        safety_r_px = int(safety_r * px_per_m)
        safety_slow_r_px = int(safety_slow_r * px_per_m)
        if safety_invaded:
            cv2.putText(img, "SAFETY STOP",
                        (cx - 42, cy + safety_r_px + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1, cv2.LINE_AA)
        elif safety_slow_invaded:
            cv2.putText(img, "SLOW",
                        (cx - 18, cy + safety_slow_r_px + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 165, 255), 1, cv2.LINE_AA)

        rw, rh = 14, 22
        cv2.rectangle(img, (cx-rw//2, cy-rh//2), (cx+rw//2, cy+rh//2), (220, 220, 220), -1)
        # Flecha de orientacion apuntando al FRENTE (Up)
        cv2.arrowedLine(img, (cx, cy + 5), (cx, cy - 8), (50, 50, 50), 2, tipLength=0.4)

        # ── OBSTACULO MAS CERCANO ─────────────────────────────────────────────
        if np.isfinite(d_min) and d_min < self.max_range:
            ox = int(cx - d_min * math.sin(a_min) * px_per_m)
            oy = int(cy - d_min * math.cos(a_min) * px_per_m)
            is_obs = d_min < float(self.get_parameter('obstacle_thresh').value)
            color = (0, 0, 255) if is_obs else (0, 255, 0)
            cv2.drawMarker(img, (ox, oy), color, cv2.MARKER_TILTED_CROSS, 10, 2)
            cv2.line(img, (cx, cy), (ox, oy), color, 1, cv2.LINE_4)
            cv2.putText(img, f"{d_min:.2f}m", (ox + 8, oy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)

        # HUD Technical
        cv2.putText(img, "TOP-DOWN LIDAR VIEW (CARTESIAN)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        cv2.putText(img, f"FIXED FRAME: base_link", (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)

        # Etiquetas con conos y distancias por sector (esquina superior derecha)
        obs_th     = float(self.get_parameter('obstacle_thresh').value)

        cv2.putText(img, f"WIDTH: {safety_w:.2f}m  SIDE: {side_max:.2f}m",
                    (S - 200, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                    (0, 200, 255), 1, cv2.LINE_AA)
        cv2.putText(img, f"OBS_TH: {obs_th:.2f} m",
                    (S - 200, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                    (180, 180, 180), 1, cv2.LINE_AA)

        # Distancias por sector con color (rojo si < obs_th)
        def _fmt(d):
            return f"{d:.2f}m" if np.isfinite(d) else "inf"
        def _col(d):
            return (0, 0, 255) if (np.isfinite(d) and d < obs_th) else (255, 255, 255)

        cv2.putText(img, f"L:{_fmt(d_left)}  F:{_fmt(d_front)}  R:{_fmt(d_right)}",
                    (S - 200, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                    _col(min(d_left, d_front, d_right)), 1, cv2.LINE_AA)

        # Sector trasero (color magenta para coincidir con el cono dibujado)
        back_col = (0, 0, 255) if (np.isfinite(d_back) and d_back < obs_th) else (180, 60, 200)
        cv2.putText(img, f"B:{_fmt(d_back)}",
                    (S - 200, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                    back_col, 1, cv2.LINE_AA)

        # Circulos de seguridad 360° (rojo=STOP, naranja=SLOW, blanco=libre)
        if np.isfinite(d_360) and d_360 < safety_r:
            s360_col = (0, 0, 255)
        elif np.isfinite(d_360) and d_360 < safety_slow_r:
            s360_col = (0, 165, 255)
        else:
            s360_col = (220, 220, 220)
        cv2.putText(img, f"360:{_fmt(d_360)}  (slow={safety_slow_r:.2f} stop={safety_r:.2f})",
                    (S - 240, 88), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                    s360_col, 1, cv2.LINE_AA)

        # Indicador de pared (vallas)
        if is_wall:
            cv2.putText(img, "WALL DETECTED (ignoring)",
                        (S - 200, 106), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                        (0, 200, 255), 1, cv2.LINE_AA)

        # Indicador de 10 grados
        c10deg_col = (0, 255, 255) if (np.isfinite(d_center_10deg) and d_center_10deg < obs_th) else (200, 200, 200)
        cv2.putText(img, f"C (10deg): {_fmt(d_center_10deg)}",
                    (S - 200, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.40,
                    c10deg_col, 1, cv2.LINE_AA)

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
