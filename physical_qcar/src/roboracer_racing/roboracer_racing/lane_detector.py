import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool, Float32MultiArray, String
import cv2
import numpy as np
import threading
import time
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSHistoryPolicy, QoSDurabilityPolicy)

IMG_W, IMG_H = 320, 240

# Umbral de confianza bajo el cual se emite stop_sign
_CONF_STOP_THRESHOLD = 0.12
# Ciclos sin deteccion antes de resetear el estado EMA
_CONF_RESET_THRESHOLD = 0.05

# ── Constantes para LaneSideEstimator ─────────────────────────────────────────
# Probabilidad minima para emitir una decision (debajo → 'unknown')
# Umbral bajo: con una sola linea amarilla en pista, casi siempre carril derecho
_LANE_MIN_CONFIDENCE  = 0.12
# Alpha EMA rapido (muchas senales disponibles)
_LANE_EMA_FAST        = 0.25
# Alpha EMA lento (pocas/ninguna senal)
_LANE_EMA_SLOW        = 0.12
# Zona muerta alrededor del centro (px): reducida para pista con una sola linea
_LANE_DEAD_ZONE_PX    = 12
# Pesos relativos de cada senal en la fusion
# Senal amarilla es la principal; offset es apoyo debil
_W_YELLOW             = 0.85
_W_OFFSET             = 0.15
# Frames de gracia sin linea amarilla antes de iniciar decaimiento de confianza
_YELLOW_GRACE_FRAMES  = 8
# Escala sigmoid para senal amarilla (fraccion de IMG_W)
_YELLOW_SIGMOID_SCALE = 0.10


# ═══════════════════════════════════════════════════════════════════════════════
#  Utilidades de linea: representacion [x_bot, y_bot, x_top, y_top]
# ═══════════════════════════════════════════════════════════════════════════════

def shift_line_x(line, dx):
    return [line[0] + dx, line[1], line[2] + dx, line[3]]


def average_lines(line_a, line_b):
    return [
        int((line_a[0] + line_b[0]) / 2), line_a[1],
        int((line_a[2] + line_b[2]) / 2), line_a[3],
    ]


def ema_line(prev, new, alpha):
    """EMA componente a componente. Si prev es None devuelve new (y viceversa)."""
    if prev is None:
        return new
    if new is None:
        return prev
    return [int(alpha * new[i] + (1 - alpha) * prev[i]) for i in range(4)]


def line_top_point(line):
    return (line[2], line[3])


def line_bot_point(line):
    return (line[0], line[1])


def interpolate_line_x_at_y(line, y_target):
    """
    Interpola la coordenada X de una linea [x_bot, y_bot, x_top, y_top]
    en una altura Y dada. Devuelve None si la linea es horizontal.
    """
    x1, y1, x2, y2 = line
    if y1 == y2:
        return None
    t = (y_target - y1) / (y2 - y1)
    return x1 + t * (x2 - x1)


# ═══════════════════════════════════════════════════════════════════════════════
#  Modelo de carril — genera center_line SIEMPRE que haya al menos una linea
# ═══════════════════════════════════════════════════════════════════════════════

def compute_lane_model(yellow_line, half_width_px, img_w=IMG_W):
    """
    Modelo de carril para pista con UNA SOLA linea amarilla divisora central.

    Geometria de la pista:
      - 1 sola linea amarilla = divisor central entre carriles
      - Carro en CARRIL DERECHO  → amarilla aparece a la IZQUIERDA de la imagen
      - Carro en CARRIL IZQUIERDO → amarilla aparece a la DERECHA  de la imagen

    El target_pixel para el seguidor es la amarilla desplazada `half_width_px`
    hacia el lado opuesto donde aparece (mantener el carro centrado en su carril).

    Retorna dict: yellow_line, center_line, target_pixel, lane_side, mode.
    """
    result = dict(yellow_line=None, center_line=None, target_pixel=None,
                  lane_side=None, mode='NONE')

    if yellow_line is None:
        return result

    img_cx = img_w / 2.0
    yellow_x = float(yellow_line[0])   # x_bot — punto mas cercano al carro

    if yellow_x < img_cx:
        # Amarilla a la izquierda → carro en carril DERECHO → target a la derecha
        dx = int(half_width_px)
        result['lane_side'] = 'right'
    else:
        # Amarilla a la derecha → carro en carril IZQUIERDO → target a la izquierda
        dx = -int(half_width_px)
        result['lane_side'] = 'left'

    center_line = shift_line_x(yellow_line, dx)
    result['yellow_line']  = yellow_line
    result['center_line']  = center_line
    result['target_pixel'] = list(line_top_point(center_line))
    result['mode']         = 'YELLOW'
    return result


# ═══════════════════════════════════════════════════════════════════════════════
#  Estimador de lado de carril con fusion de senales
# ═══════════════════════════════════════════════════════════════════════════════

class LaneSideEstimator:
    """
    Estimador de carril para pista con UNA linea amarilla divisora central.

    Logica:
      - Amarilla a la IZQUIERDA de la imagen → carro en CARRIL DERECHO
      - Amarilla a la DERECHA   de la imagen → carro en CARRIL IZQUIERDO

    Funde la senal principal (posicion de la amarilla) con el offset normalizado
    como apoyo debil. EMA sobre la probabilidad para estabilidad.
    """

    def __init__(self):
        self._prob_right       = 0.5
        self._confidence       = 0.0
        self._frames_no_yellow = 0
        self._last_decision    = 'unknown'

    def update(self, yellow_line, current_offset, frame_conf, img_w=IMG_W):
        """
        Parametros
        ----------
        yellow_line    : [x_bot, y_bot, x_top, y_top] | None
        current_offset : float [-1, 1] — offset lateral suavizado
        frame_conf     : float [0, 1]  — confianza del frame
        img_w          : int           — ancho de imagen en px

        Retorna (decision, prob_right, confidence)
        """
        img_cx = img_w / 2.0
        votes  = []

        # ── Senal A: posicion de la amarilla en la imagen ────────────────────
        if yellow_line is not None:
            self._frames_no_yellow = 0
            y_mid = (yellow_line[1] + yellow_line[3]) / 2.0
            x_at_mid = interpolate_line_x_at_y(yellow_line, y_mid)
            if x_at_mid is None:
                x_at_mid = (yellow_line[0] + yellow_line[2]) / 2.0
            delta = x_at_mid - img_cx

            if abs(delta) >= _LANE_DEAD_ZONE_PX:
                scale = max(img_w * _YELLOW_SIGMOID_SCALE, 1.0)
                # delta < 0 → amarilla a la izq → carril DERECHO → prob_a > 0.5
                prob_a = 1.0 / (1.0 + np.exp(delta / scale))
            else:
                prob_a = 0.5

            w_a = _W_YELLOW * float(np.clip(frame_conf * 2.0, 0.0, 1.0))
            votes.append((prob_a, w_a))
        else:
            self._frames_no_yellow += 1

        # ── Senal B: offset normalizado (apoyo debil) ────────────────────────
        # offset > 0 → target a la derecha → carro tiende a izquierda → LEFT
        # offset < 0 → target a la izquierda → carro tiende a derecha → RIGHT
        if abs(current_offset) > 0.05 and yellow_line is not None:
            prob_b = float(np.clip((-current_offset + 1.0) / 2.0, 0.05, 0.95))
            w_b = _W_OFFSET * float(np.clip(frame_conf, 0.0, 0.8))
            votes.append((prob_b, w_b))

        # ── Fusion ponderada ─────────────────────────────────────────────────
        if votes:
            total_w = sum(w for _, w in votes)
            fused_prob = sum(p * w for p, w in votes) / total_w if total_w > 1e-6 else 0.5
        else:
            fused_prob = 0.5

        # ── EMA sobre prob_right ─────────────────────────────────────────────
        has_signal = yellow_line is not None and frame_conf > 0.30
        alpha = _LANE_EMA_FAST if has_signal else _LANE_EMA_SLOW
        self._prob_right = alpha * fused_prob + (1.0 - alpha) * self._prob_right
        self._prob_right = float(np.clip(self._prob_right, 0.0, 1.0))

        # ── Confianza con decay por ausencia de amarilla ─────────────────────
        decision_strength = abs(self._prob_right - 0.5) * 2.0
        no_yellow_decay = np.exp(
            -max(0, self._frames_no_yellow - _YELLOW_GRACE_FRAMES) / 10.0)
        self._confidence = float(
            decision_strength * no_yellow_decay * min(frame_conf * 1.5, 1.0))

        # ── Decision final ───────────────────────────────────────────────────
        if not votes or self._confidence < _LANE_MIN_CONFIDENCE:
            decision = 'unknown'
        elif self._prob_right > 0.5:
            decision = 'right'
        else:
            decision = 'left'

        self._last_decision = decision
        return decision, float(self._prob_right), float(self._confidence)

    def reset(self):
        """Reinicia el estado interno (llamar cuando se pierde tracking completo)."""
        self._prob_right       = 0.5
        self._confidence       = 0.0
        self._frames_no_yellow = 0
        self._last_decision    = 'unknown'

    @property
    def last_decision(self):
        return self._last_decision

    @property
    def prob_right(self):
        return self._prob_right

    @property
    def confidence(self):
        return self._confidence


# ═══════════════════════════════════════════════════════════════════════════════
#  Nodo ROS 2
# ═══════════════════════════════════════════════════════════════════════════════

class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        # ── Parametros ────────────────────────────────────────────────────────
        self.declare_parameter('image_topic',   '/qcar/csi_front')
        self.declare_parameter('publish_debug',  True)

        # Distancia (px en imagen) que se desplaza el polinomio amarillo
        # para definir el center_line del carril. A mayor valor → target
        # mas alejado de la linea → mas margen para que el carro este
        # centrado en su carril. Si es muy chico, el target queda casi
        # sobre la linea y el carro la sigue muy de cerca.
        self.declare_parameter('lane_half_width_px',        90.0)
        self.declare_parameter('lane_half_width_ema_alpha',  0.15)
        self.declare_parameter('center_line_ema_alpha',      0.25)
        self.declare_parameter('lane_lateral_bias',          0.0)

        self.declare_parameter('hough_threshold',            12)
        self.declare_parameter('min_line_length',            25)
        self.declare_parameter('max_line_gap',               20)
        self.declare_parameter('debug_draw_hough_candidates', False)
        self.declare_parameter('fallback_max_cycles',        15)
        # Procesar 1 de cada N frames recibidos. 1 = procesar todos (default).
        # Subir a 2 si el procesamiento no alcanza al frame-rate de camara.
        self.declare_parameter('process_every_n_frames',      1)

        self.declare_parameter('camera_to_rear_axle_forward_m', 0.323)
        self.declare_parameter('camera_to_rear_axle_lateral_m',  0.0)
        self.declare_parameter('bev_pixels_per_meter', 1000.0)
        self.declare_parameter('bev_dst_points_m', [
            0.0, 0.0,  0.0, 0.103908,
            0.309683, 0.103908,  0.309683, 0.0,
        ])
        # ROI: trapecio MUY AMPLIO (casi rectangular).
        # Top extendido lateralmente para captar la linea cuando se pega
        # a un borde de la imagen (curvas pronunciadas o carro descentrado).
        self.declare_parameter('roi_polygon_points_px', [
            10.0, 235.0,   # inferior-izquierdo: base muy amplia
            45.0, 125.0,   # superior-izquierdo: muy amplio
           275.0, 125.0,   # superior-derecho: muy amplio
           310.0, 235.0,   # inferior-derecho: base muy amplia
        ])

        # ── Camara intrinseca ─────────────────────────────────────────────────
        hfov, vfov = 160.0, 120.0
        fx = (IMG_W / 2) / np.tan(np.radians(hfov / 2))
        fy = (IMG_H / 2) / np.tan(np.radians(vfov / 2))
        self.K = np.array([[fx, 0, IMG_W / 2],
                           [0, fy, IMG_H / 2],
                           [0,  0,         1]], dtype=np.float64)
        self.dist = np.array([-0.264598808, 0.0156281135,
                               0.0000652954, 0.0053984313,
                               0.0822019378], dtype=np.float64)
        self.newK, _ = cv2.getOptimalNewCameraMatrix(
            self.K, self.dist, (IMG_W, IMG_H), 1, (IMG_W, IMG_H))

        # ── BEV ───────────────────────────────────────────────────────────────
        self.bev_ppm = float(self.get_parameter('bev_pixels_per_meter').value)
        bev_flat = self.get_parameter('bev_dst_points_m').value
        self.bev_dst = np.array(bev_flat, dtype=np.float32).reshape(4, 2)
        roi_flat = self.get_parameter('roi_polygon_points_px').value
        self.roi_px = np.array(roi_flat, dtype=np.float32).reshape(4, 2)

        bw = float(np.max(self.bev_dst[:, 0]) - np.min(self.bev_dst[:, 0]))
        bh = float(np.max(self.bev_dst[:, 1]) - np.min(self.bev_dst[:, 1]))
        self.bev_size = (max(1, int(np.ceil(bw * self.bev_ppm))),
                         max(1, int(np.ceil(bh * self.bev_ppm))))

        # Homografia cacheada — el ROI no cambia entre frames
        self._h_matrix = self._build_homography()
        self._h_inv    = np.linalg.inv(self._h_matrix)   # BEV → image

        # Mascara ROI cacheada — usada para excluir todo lo de afuera
        # del trapecio en CADA mascara (color, sobel, etc.) desde el inicio.
        self._roi_mask = np.zeros((IMG_H, IMG_W), dtype=np.uint8)
        cv2.fillPoly(self._roi_mask,
                     self.roi_px.astype(np.int32).reshape(1, 4, 2), 255)

        # ── Rangos de color ───────────────────────────────────────────────────
        # PISTA: dos lineas amarillas paralelas. No hay linea blanca real en el suelo.
        # Las baldosas blancas del borde se filtran excluyendolas del ROI y
        # aumentando los requisitos de saturacion para "blanco valido".

        # Amarillo HSV (post-CLAHE) — PERMISIVO:
        # H: 15-45 (rango amarillo amplio, incluye dorado)
        # S: 35+ (acepta pintura muy desgastada)
        # V: 30+ (acepta amarillo bajo sombra fuerte)
        self.yel_hsv_lo = np.array([15,  35,  30], dtype=np.uint8)
        self.yel_hsv_hi = np.array([45, 255, 255], dtype=np.uint8)
        # Amarillo HLS — PERMISIVO:
        self.yel_hls_lo = np.array([15,  35,  40], dtype=np.uint8)
        self.yel_hls_hi = np.array([45, 220, 255], dtype=np.uint8)
        # Blanco HLS: umbral de saturacion ALTO (S < 25) para evitar baldosas grises
        # y exigir luminancia muy alta (L > 180). Las baldosas tipicamente tienen
        # L entre 120-160 y S > 30, por lo que quedan excluidas.
        self.wht_hls_lo = np.array([  0, 180,   0], dtype=np.uint8)
        self.wht_hls_hi = np.array([180, 255,  25], dtype=np.uint8)

        # CLAHE instanciado una vez
        self._clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))

        # Kernels morfologicos precalculados
        self._k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # Parametros Hough cacheados (Python 3.6: no hay add_on_set_parameters_callback)
        self._hough_thr = int(self.get_parameter('hough_threshold').value)
        self._hough_mll = int(self.get_parameter('min_line_length').value)
        self._hough_mlg = int(self.get_parameter('max_line_gap').value)

        # ── Estado EMA ────────────────────────────────────────────────────────
        self.last_yellow_line  = None
        self.last_target_pixel = None
        # Continuidad temporal: polinomio del frame anterior. Sirve como
        # "prior espacial" para el sliding windows del siguiente frame,
        # asi no salta a otro lado cuando hay ruido cerca de la linea real.
        self._last_poly = None

        self.dyn_half_w    = float(self.get_parameter('lane_half_width_px').value)
        self.last_offset   = 0.0
        self.last_conf     = 0.0
        self.current_lane  = 'unknown'
        self.waiting_cycles = 0
        self._raw_bgr = None
        self._debug_frame_count = 0   # publica debug cada 2 frames
        self._frame_count = 0         # contador para frame-skip de procesamiento

        # ── Worker thread para procesamiento CV ──────────────────────────────
        # El callback de ROS NUNCA debe bloquearse haciendo CV pesado:
        # eso atora el executor y mata el flujo de la camara y otros nodos.
        # En su lugar, el callback solo guarda el ultimo mensaje, y un hilo
        # dedicado consume el mas reciente y lo procesa a su propio ritmo.
        self._latest_msg     = None
        self._msg_lock       = threading.Lock()
        self._stop_worker    = False
        self._worker_thread  = threading.Thread(
            target=self._processing_loop, daemon=True, name='lane_cv_worker')

        # ── Estimador de carril (NUEVO) ───────────────────────────────────────
        self._lane_estimator = LaneSideEstimator()

        # ── ROS ───────────────────────────────────────────────────────────────
        # QoS depth=1, BEST_EFFORT: siempre procesar SOLO el frame mas reciente.
        # Evita que se acumulen frames viejos en la cola si el procesamiento
        # se retrasa — el carro reacciona al "ahora", no al "hace 300ms".
        cam_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability  = QoSDurabilityPolicy.VOLATILE,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 1,
        )
        self.sub = self.create_subscription(
            CompressedImage,
            self.get_parameter('image_topic').value,
            self._compressed_cb,
            cam_qos)

        self.offset_pub    = self.create_publisher(Float32,           '/lane/center_offset',    10)
        self.conf_pub      = self.create_publisher(Float32,           '/lane/confidence',        10)
        self.dbg_pub       = self.create_publisher(CompressedImage,   '/lane/image_debug',       10)
        self.stop_pub      = self.create_publisher(Bool,              '/lane/stop_sign',         10)
        self.target_pub    = self.create_publisher(Float32MultiArray, '/lane_target_point_m',    10)
        self.lane_pub      = self.create_publisher(String,            '/lane/current_lane',      10)
        # NUEVO: probabilidad continua para diagnostico/logging
        self.lane_prob_pub = self.create_publisher(Float32,           '/lane/right_probability', 10)
        # NUEVO: confianza interna del estimador de carril
        self.lane_conf_pub = self.create_publisher(Float32,           '/lane/side_confidence',   10)
        # FASE 2 (Contribucion C): curvatura del carril en 1/m y coeficientes.
        # /lane/curvature: kappa = |2A| / (1 + (2A*y_lookahead + B)^2)^(3/2)
        #                  convertida a 1/m multiplicando por bev_pixels_per_meter.
        # /lane/poly_coeffs: [A, B, C, y_lookahead_px, ppm] para downstreams
        #                    que quieran recalcular curvatura en otra y o
        #                    proyectar puntos arbitrarios sobre la curva.
        # Aditivo: si nadie los consume, no afecta nada del pipeline.
        self.curv_pub      = self.create_publisher(Float32,           '/lane/curvature',         10)
        self.poly_pub      = self.create_publisher(Float32MultiArray, '/lane/poly_coeffs',       10)

        # Arrancar el worker AHORA — todas las dependencias estan listas
        self._worker_thread.start()

        self.get_logger().info('Lane Detector Pro v7 — worker thread + BEV sliding windows')

    # ──────────────────────────────────────────────────────────────────────────
    #  Callback principal
    # ──────────────────────────────────────────────────────────────────────────
    def _compressed_cb(self, msg):
        """
        Callback LIGERO: solo guarda el mensaje mas reciente.
        El procesamiento CV pesado se hace en el worker thread aparte para
        que el executor de ROS quede libre y no se ahogue la camara.
        """
        with self._msg_lock:
            self._latest_msg = msg

    def _processing_loop(self):
        """
        Loop del worker thread: consume el ultimo CompressedImage disponible
        y procesa el frame. Si no hay frame nuevo, duerme brevemente.
        """
        while not self._stop_worker:
            with self._msg_lock:
                msg = self._latest_msg
                self._latest_msg = None
            if msg is None:
                time.sleep(0.003)   # 3ms — sin frame nuevo
                continue

            # Frame-skip configurable
            self._frame_count += 1
            skip_n = max(1, int(self.get_parameter('process_every_n_frames').value))
            if self._frame_count % skip_n != 0:
                continue

            try:
                buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                bgr = cv2.resize(bgr, (IMG_W, IMG_H), interpolation=cv2.INTER_AREA)
                if self.get_parameter('publish_debug').value:
                    self._raw_bgr = bgr.copy()
                bgr = cv2.undistort(bgr, self.K, self.dist, None, self.newK)
                self._process_frame(bgr, msg.header)
            except Exception as e:
                self.get_logger().error(f'CV worker error: {e}')

    # ──────────────────────────────────────────────────────────────────────────
    #  Segmentacion de color
    # ──────────────────────────────────────────────────────────────────────────
    def _segment_colors(self, bgr):
        """
        CLAHE en canal V. Segmenta amarillo (HSV+HLS), blanco (HLS),
        y calcula mascara Sobel-X sobre canal L (refuerza bordes verticales).

        Retorna (yellow_mask, white_mask, sobel_mask, bgr_eq).
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        v_eq   = self._clahe.apply(v)
        hsv_eq = cv2.merge([h, s, v_eq])
        bgr_eq = cv2.cvtColor(hsv_eq, cv2.COLOR_HSV2BGR)
        hls_eq = cv2.cvtColor(bgr_eq, cv2.COLOR_BGR2HLS)

        # ── (1) Mascara amarillo HSV+HLS (la que ya teniamos) ────────────────
        m_y_hsv = cv2.inRange(hsv_eq, self.yel_hsv_lo, self.yel_hsv_hi)
        m_y_hls = cv2.inRange(hls_eq, self.yel_hls_lo, self.yel_hls_hi)
        yellow  = cv2.bitwise_or(m_y_hsv, m_y_hls)
        white   = cv2.inRange(hls_eq, self.wht_hls_lo, self.wht_hls_hi)

        # ── (2) Mascara amarillo LAB + BRILLO ADAPTATIVO con Otsu ───────────
        # Estrategia: pintura amarilla tiene tono cromatico (b alto) Y brillo
        # MEDIO (no muy oscuro como el asfalto, ni sobre-expuesto como reflejos).
        # Otsu calcula automaticamente el "punto medio" entre fondo y objetos:
        # usamos ese valor como referencia adaptativa al frame actual.
        lab = cv2.cvtColor(bgr_eq, cv2.COLOR_BGR2LAB)
        l_lab = lab[:, :, 0]
        b_lab = lab[:, :, 2]

        # Tono cromatico amarillo (b > 135: permisivo, captura amarillos debiles)
        yellow_lab = cv2.inRange(b_lab, 135, 215)

        # Otsu sobre L da el umbral T que separa fondo oscuro de objetos claros
        T_otsu, _ = cv2.threshold(l_lab, 0, 255,
                                  cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Brillo valido: rango AMPLIO. Acepta desde brillo del fondo hasta
        # casi sobre-expuesto. Mas tolerante a variaciones de iluminacion.
        L_min = int(T_otsu * 0.85)              # un poco abajo del fondo (sombra)
        L_max = int(min(T_otsu * 1.35, 220))    # bien arriba (pintura clara)
        in_brightness_range = cv2.inRange(l_lab, L_min, L_max)

        # Combinar: amarillo cromatico AND brillo en rango medio adaptativo
        yellow_lab = cv2.bitwise_and(yellow_lab, in_brightness_range)

        # Fusionar amarillo HSV/HLS con amarillo LAB (refuerzo mutuo)
        yellow = cv2.bitwise_or(yellow, yellow_lab)

        # Morfologia: limpiar ruido puntual y cerrar huecos pequenos
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN,  self._k3)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, self._k3)
        yellow = cv2.dilate(yellow, self._k5, iterations=1)
        white  = cv2.morphologyEx(white,  cv2.MORPH_OPEN,  self._k3)
        white  = cv2.morphologyEx(white,  cv2.MORPH_CLOSE, self._k3)
        white  = cv2.dilate(white,  self._k5, iterations=1)

        # ── Mascaras de luminosidad/gradiente como REFUERZO del color ───────
        # Antes competian con el amarillo (OR). Ahora REFUERZAN: solo cuentan
        # donde tambien hay color amarillo. Esto evita que luces blancas
        # o reflejos cuenten como linea.
        l_ch = hls_eq[:, :, 1]

        # Sobel-X (gradiente horizontal = borde vertical = linea)
        sobel_x = cv2.Sobel(l_ch, cv2.CV_32F, 1, 0, ksize=3)
        sobel_x = np.abs(sobel_x)
        smax = float(sobel_x.max()) if sobel_x.max() > 0 else 1.0
        sobel_x = np.clip(sobel_x * 255.0 / smax, 0, 255).astype(np.uint8)
        sobel_mask = cv2.inRange(sobel_x, 40, 255)

        # Otsu sobre L: deteccion bimodal automatica
        gray = cv2.cvtColor(bgr_eq, cv2.COLOR_BGR2GRAY)
        _, otsu_mask = cv2.threshold(gray, 0, 255,
                                     cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # Adaptive local: tolerante a sombras
        adaptive_mask = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, blockSize=15, C=-5)

        # Top-Hat morfologico: detecta zonas CLARAS rodeadas de OSCURO.
        # Perfecto para pintura desgastada (casi blanca) sobre asfalto negro,
        # independiente del color exacto. La pintura "casi blanca entre carriles
        # negros" pasa este filtro aunque ya no sea cromaticamente amarilla.
        kernel_th = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 3))
        top_hat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel_th)
        tophat_mask = cv2.inRange(top_hat, 25, 255)

        # Combinar los 4 detectores de luminosidad/gradiente
        edge_raw = cv2.bitwise_or(sobel_mask, otsu_mask)
        edge_raw = cv2.bitwise_or(edge_raw, tophat_mask)
        edge_raw = cv2.bitwise_or(edge_raw, adaptive_mask)

        # CLAVE: intersectar con `yellow` para que la mayoria de detectores
        # (Sobel/Otsu/Adaptive) solo cuenten donde hay color amarillo.
        # PERO el Top-Hat pasa libre — su diseno (claro entre oscuro) ya
        # filtra muy bien la pintura, asi captamos amarillo desgastado/casi
        # blanco que el filtro de color no atrapa.
        edge_color = cv2.bitwise_and(
            cv2.bitwise_or(sobel_mask, cv2.bitwise_or(otsu_mask, adaptive_mask)),
            yellow)
        edge_mask  = cv2.bitwise_or(edge_color, tophat_mask)
        edge_mask  = cv2.morphologyEx(edge_mask, cv2.MORPH_OPEN, self._k3)

        # ── ROI FINAL: SOLO procesar lo que esta dentro del trapecio ────────
        # Aplicado al final para que todas las detecciones (color, edges, etc.)
        # queden recortadas al area visible del carril. Esto:
        #  - Elimina visualmente los blobs fuera del ROI
        #  - Evita falsos positivos de luces/objetos perifericos
        #  - Acelera procesamiento posterior (menos pixeles a procesar)
        yellow    = cv2.bitwise_and(yellow,    self._roi_mask)
        white     = cv2.bitwise_and(white,     self._roi_mask)
        edge_mask = cv2.bitwise_and(edge_mask, self._roi_mask)

        return yellow, white, edge_mask, bgr_eq

    # ──────────────────────────────────────────────────────────────────────────
    #  Deteccion Hough + extrapolacion lineal
    # ──────────────────────────────────────────────────────────────────────────
    def _detect_lines(self, yellow_mask, sobel_mask, build_viz=True):
        """
        Pipeline BEV-first con sliding windows y polinomio cuadratico.

        Parametro build_viz: si False, no dibuja ventanas/polinomio en bev_viz
        (ahorra CPU cuando no vamos a publicar debug en este frame).
        """
        # 1. Combinar yellow + edge_mask (ambos YA vienen recortados al ROI
        # desde _segment_colors, asi que NO necesitamos re-aplicar el ROI aqui)
        masked = cv2.bitwise_or(yellow_mask, sobel_mask)
        edges  = cv2.Canny(masked, 35, 90)

        # 2. Warp a BEV
        bev = cv2.warpPerspective(masked, self._h_matrix, self.bev_size,
                                  flags=cv2.INTER_NEAREST)
        _, bev = cv2.threshold(bev, 30, 255, cv2.THRESH_BINARY)
        H_bev, W_bev = bev.shape

        # Buffer de viz para debug (color) — solo si build_viz=True
        bev_viz = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR) if build_viz else None

        # 3. Filtrar BEV por COMPONENTES CONECTADOS verticalmente largos.
        # Umbral PERMISIVO (22%) para no perder fragmentos cortos de linea
        # cuando solo se ve un tramo de pintura.
        n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
            bev, connectivity=8)
        min_blob_height = max(int(H_bev * 0.22), 8)
        bev_filtered = np.zeros_like(bev)
        for i in range(1, n_labels):   # skip background (label 0)
            if stats[i, cv2.CC_STAT_HEIGHT] >= min_blob_height:
                bev_filtered[labels == i] = 255

        # Si el filtrado borro todo, no hay linea valida en este frame
        if int(bev_filtered.max()) == 0:
            return None, None, bev_viz, masked, edges

        # Histograma sobre BEV filtrado, mitad cercana al carro.
        # PONDERADO: las filas mas cercanas al carro pesan mas (2x → 1x).
        # Asi si hay un blob de ruido lejos en un lado pero la linea real
        # esta cerca en el otro lado, el cercano gana en el argmax.
        near_half = bev_filtered[:H_bev // 2, :].astype(np.float32)
        weights = np.linspace(2.0, 1.0, near_half.shape[0]).reshape(-1, 1)
        histogram = np.sum(near_half * weights, axis=0)
        if int(histogram.max()) < 200:
            return None, None, bev_viz, masked, edges

        # CONTINUIDAD TEMPORAL: si tenemos polinomio del frame anterior,
        # restringimos la busqueda del base_x a ±100px de donde estaba
        # antes. Esto evita que el sliding windows salte a otro pico
        # si aparece un blob grande del lado contrario. Si no hay
        # poly previo (primer frame o reset), usamos el argmax global.
        if self._last_poly is not None:
            prior_x = int(np.polyval(self._last_poly, 0))  # x en y=0 (base BEV)
            search_lo = max(0, prior_x - 100)
            search_hi = min(W_bev, prior_x + 100)
            masked_hist = np.zeros_like(histogram)
            masked_hist[search_lo:search_hi] = histogram[search_lo:search_hi]
            # Si hay pico valido cerca del prior, usarlo
            if int(masked_hist.max()) > 200:
                base_x = int(np.argmax(masked_hist))
            else:
                # No hay pico cerca → reset y busqueda global
                self._last_poly = None
                base_x = int(np.argmax(histogram))
        else:
            base_x = int(np.argmax(histogram))

        # Trabajar con bev_filtered de aqui en adelante (no con bev original)
        bev = bev_filtered

        # 4. Sliding windows hacia el horizonte
        n_windows = 9
        window_h  = max(1, H_bev // n_windows)
        margin    = max(12, W_bev // 12)
        minpix    = 20

        nonzero_y, nonzero_x = bev.nonzero()
        if len(nonzero_y) < 50:
            return None, None, bev_viz, masked, edges

        current_x = base_x
        collected = []

        for win in range(n_windows):
            y_lo = win * window_h
            y_hi = (win + 1) * window_h
            x_lo = max(0, current_x - margin)
            x_hi = min(W_bev, current_x + margin)

            # Dibujar ventana para debug (solo si vamos a publicar viz)
            if build_viz:
                cv2.rectangle(bev_viz, (x_lo, y_lo), (x_hi, y_hi), (0, 255, 0), 1)

            in_win = ((nonzero_y >= y_lo) & (nonzero_y < y_hi) &
                      (nonzero_x >= x_lo) & (nonzero_x < x_hi))
            idx = np.where(in_win)[0]
            collected.append(idx)

            if len(idx) > minpix:
                current_x = int(np.mean(nonzero_x[idx]))

        all_idx = np.concatenate(collected) if collected else np.array([], dtype=np.int64)
        if len(all_idx) < 50:
            return None, None, bev_viz, masked, edges

        y_pts = nonzero_y[all_idx].astype(np.float32)
        x_pts = nonzero_x[all_idx].astype(np.float32)

        # 5. Polinomio cuadratico: x = A*y² + B*y + C
        try:
            poly = np.polyfit(y_pts, x_pts, 2)
        except (np.linalg.LinAlgError, ValueError):
            return None, None, bev_viz, masked, edges

        # Guardar polinomio para usarlo como prior en el siguiente frame
        # (continuidad temporal — evita saltos a otros blobs)
        self._last_poly = poly

        # Dibujar curva del polinomio en bev_viz (solo si vamos a publicar)
        if build_viz:
            y_samples = np.linspace(0, H_bev - 1, 30).astype(np.float32)
            x_samples = np.polyval(poly, y_samples)
            pts_viz = np.array([[int(x), int(y)] for x, y in zip(x_samples, y_samples)
                                if 0 <= int(x) < W_bev], dtype=np.int32)
            if len(pts_viz) >= 2:
                cv2.polylines(bev_viz, [pts_viz], False, (0, 230, 230), 2)

        # 6. Proyectar 2 puntos extremos del polinomio a image space
        y_near_bev = 0.0
        y_far_bev  = float(H_bev - 1)
        x_near_bev = float(np.polyval(poly, y_near_bev))
        x_far_bev  = float(np.polyval(poly, y_far_bev))

        bev_pts = np.array([[[x_near_bev, y_near_bev],
                             [x_far_bev,  y_far_bev]]], dtype=np.float32)
        img_pts = cv2.perspectiveTransform(bev_pts, self._h_inv)[0]

        x_bot, y_bot = img_pts[0]
        x_top, y_top = img_pts[1]
        yellow_line = [int(x_bot), int(y_bot), int(x_top), int(y_top)]

        return yellow_line, poly, bev_viz, masked, edges

    def _fits_to_line_weighted(self, fits):
        """
        Promedia lista de (m, b, x_mid, weight) ponderada por soporte de color.
        Genera linea extrapolada entre el fondo de la imagen y el punto look-ahead.
        """
        if not fits:
            return None
        weights = np.array([f[3] for f in fits], dtype=np.float32)
        total_w = weights.sum()
        if total_w < 1e-6:
            return None
        ms = float(np.sum([f[0] * f[3] for f in fits]) / total_w)
        bs = float(np.sum([f[1] * f[3] for f in fits]) / total_w)
        if abs(ms) < 1e-6:
            return None
        y1 = IMG_H
        y2 = int(IMG_H * 0.52)
        x1 = int((y1 - bs) / ms)
        x2 = int((y2 - bs) / ms)
        if not (-(IMG_W // 4) < x1 < IMG_W + (IMG_W // 4) and
                -(IMG_W // 4) < x2 < IMG_W + (IMG_W // 4)):
            return None
        return [x1, y1, x2, y2]

    def _fits_to_line(self, fits):
        """Promedia lista de (m, b, x_mid) sin ponderacion. Compatibilidad."""
        if not fits:
            return None
        ms = np.mean([f[0] for f in fits])
        bs = np.mean([f[1] for f in fits])
        if abs(ms) < 1e-6:
            return None
        y1 = IMG_H
        y2 = int(IMG_H * 0.52)
        x1 = int((y1 - bs) / ms)
        x2 = int((y2 - bs) / ms)
        if not (-(IMG_W // 4) < x1 < IMG_W + (IMG_W // 4) and
                -(IMG_W // 4) < x2 < IMG_W + (IMG_W // 4)):
            return None
        return [x1, y1, x2, y2]

    # ──────────────────────────────────────────────────────────────────────────
    #  EMA de lineas y ancho de carril
    # ──────────────────────────────────────────────────────────────────────────
    def _smooth_lines(self, yellow_raw):
        """EMA sobre la unica linea real detectada (amarilla central)."""
        a_line = 0.40
        self.last_yellow_line = ema_line(self.last_yellow_line, yellow_raw, a_line)
        return self.last_yellow_line

    # ──────────────────────────────────────────────────────────────────────────
    #  Deteccion de carril — NUEVA implementacion con LaneSideEstimator
    # ──────────────────────────────────────────────────────────────────────────
    def _detect_lane_side(self, yellow_line, offset, frame_conf):
        """Delega al LaneSideEstimator y publica metricas de diagnostico."""
        decision, prob_right, side_conf = self._lane_estimator.update(
            yellow_line    = yellow_line,
            current_offset = offset,
            frame_conf     = frame_conf,
        )
        self.lane_prob_pub.publish(Float32(data=prob_right))
        self.lane_conf_pub.publish(Float32(data=side_conf))
        return decision

    # ──────────────────────────────────────────────────────────────────────────
    #  Homografia y conversion pixel → metros
    # ──────────────────────────────────────────────────────────────────────────
    def _build_homography(self):
        """Llamada una sola vez en __init__. El ROI es constante."""
        src = self.roi_px.astype(np.float32)
        dst = self.bev_dst * self.bev_ppm
        return cv2.getPerspectiveTransform(src, dst)

    def _pixel_to_rear_axle_m(self, px, py):
        """Usa la homografia cacheada."""
        pt    = np.array([[[float(px), float(py)]]], dtype=np.float32)
        bev   = cv2.perspectiveTransform(pt, self._h_matrix)[0][0]
        bev_m = bev / self.bev_ppm

        cx = 0.5 * (np.min(self.bev_dst[:, 0]) + np.max(self.bev_dst[:, 0]))
        ny = 0.5 * (self.bev_dst[0][1] + self.bev_dst[3][1])
        fy = 0.5 * (self.bev_dst[1][1] + self.bev_dst[2][1])

        lateral = float(bev_m[0] - cx)
        forward = float(np.clip(abs(bev_m[1] - ny), 0.0, abs(fy - ny)))

        x_r = lateral + float(self.get_parameter('camera_to_rear_axle_lateral_m').value)
        y_r = forward + float(self.get_parameter('camera_to_rear_axle_forward_m').value)
        return x_r, y_r

    # ──────────────────────────────────────────────────────────────────────────
    #  Frame principal
    # ──────────────────────────────────────────────────────────────────────────
    def _process_frame(self, bgr, header):
        # Determinar si vamos a publicar debug en ESTE frame
        # (asi evitamos construir bev_viz cuando no sirve)
        will_publish_debug = (
            self.get_parameter('publish_debug').value and
            ((self._debug_frame_count + 1) % 2 == 0)
        )

        # 1. Segmentacion de color + Sobel-X
        yellow_mask, white_mask, sobel_mask, bgr_eq = self._segment_colors(bgr)

        # 2. Deteccion BEV-first: sliding windows + polinomio cuadratico
        yellow_raw, yellow_poly, bev_viz, masked, edges = \
            self._detect_lines(yellow_mask, sobel_mask, build_viz=will_publish_debug)

        # 3. EMA sobre la amarilla
        self._smooth_lines(yellow_raw)

        # 4. Modelo de carril con amarilla EMA + half_width fijo
        model_sm = compute_lane_model(self.last_yellow_line, self.dyn_half_w, IMG_W)
        mode = model_sm['mode']

        # Si no hay historial EMA todavia, usar deteccion raw directamente
        if mode == 'NONE':
            model_sm = compute_lane_model(yellow_raw, self.dyn_half_w, IMG_W)
            mode = model_sm['mode']

        # 4b. Override curve-aware del target_pixel usando el polinomio en BEV
        #     (mas preciso que extrapolar la linea recta en image space).
        if yellow_poly is not None and model_sm['lane_side'] is not None:
            H_bev = self.bev_size[1]
            y_lookahead = float(H_bev - 5)   # cerca del horizonte = look-ahead largo
            x_yellow_bev = float(np.polyval(yellow_poly, y_lookahead))
            dx_bev = self.dyn_half_w if model_sm['lane_side'] == 'right' else -self.dyn_half_w
            target_bev = np.array([[[x_yellow_bev + dx_bev, y_lookahead]]],
                                  dtype=np.float32)
            target_img = cv2.perspectiveTransform(target_bev, self._h_inv)[0][0]
            tx, ty = int(target_img[0]), int(target_img[1])
            if 0 <= tx < IMG_W and 0 <= ty < IMG_H:
                model_sm['target_pixel'] = [tx, ty]

            # FASE 2 (Contribucion C): publicar curvatura y coeficientes del
            # polinomio yellow_poly. Se publican aqui dentro porque solo son
            # validos cuando hay yellow_poly. Si no hay, no publicamos —
            # downstream debe asumir timeout y usar fallback (curvatura 0).
            A = float(yellow_poly[0])  # coef de y^2
            B = float(yellow_poly[1])  # coef de y
            C = float(yellow_poly[2])  # constante
            # Derivada x'(y) = 2A*y + B, evaluada en y_lookahead
            dx_dy = 2.0 * A * y_lookahead + B
            # Curvatura signed-free: kappa_px = |2A| / (1 + (dx/dy)^2)^(3/2)
            kappa_px = abs(2.0 * A) / ((1.0 + dx_dy * dx_dy) ** 1.5)
            # Conversion a 1/m: multiplicar por bev_pixels_per_meter (ppm).
            # ppm = px/m => kappa [1/px] * ppm [px/m] = kappa [1/m].
            kappa_m = kappa_px * float(self.bev_ppm)
            self.curv_pub.publish(Float32(data=float(kappa_m)))
            poly_msg = Float32MultiArray()
            poly_msg.data = [A, B, C, y_lookahead, float(self.bev_ppm)]
            self.poly_pub.publish(poly_msg)

        # 6. Target pixel con bias lateral
        fallback_max = int(self.get_parameter('fallback_max_cycles').value)
        bias = float(np.clip(
            self.get_parameter('lane_lateral_bias').value, -0.9, 0.9))
        target_pixel = None

        if model_sm['target_pixel'] is not None:
            tp = model_sm['target_pixel']
            bx = int(tp[0] - bias * self.dyn_half_w)
            target_pixel = [int(np.clip(bx, 0, IMG_W - 1)), tp[1]]
            self.last_target_pixel = target_pixel
            self.waiting_cycles = 0
        else:
            self.waiting_cycles += 1
            if self.waiting_cycles <= fallback_max and self.last_target_pixel is not None:
                target_pixel = self.last_target_pixel
                mode = 'FALLBACK'

        # 7. Offset normalizado [-1, 1] y confianza [0, 1]
        n_px = int(np.sum(masked > 0))
        if target_pixel is not None:
            raw_off   = (target_pixel[0] - IMG_W / 2.0) / (IMG_W / 2.0)
            offset    = float(np.clip(raw_off, -1.0, 1.0))
            conf_base = float(np.clip(n_px / 1500.0, 0.0, 1.0))   # menos pixels esperados (1 sola linea)
            if mode == 'YELLOW':
                conf_base = min(conf_base * 1.35, 1.0)
            elif mode in ('NONE', 'FALLBACK'):
                conf_base *= 0.3
            self.last_offset = 0.55 * self.last_offset + 0.45 * offset
            self.last_conf   = 0.8 * self.last_conf   + 0.2 * conf_base
        else:
            self.last_conf *= 0.88
            if self.last_conf < _CONF_RESET_THRESHOLD:
                self.last_yellow_line  = None
                self.last_target_pixel = None
                self._last_poly        = None
                self._lane_estimator.reset()

        self.offset_pub.publish(Float32(data=float(self.last_offset)))
        self.conf_pub.publish(Float32(data=float(self.last_conf)))

        # Publicar stop_sign cuando la confianza cae bajo umbral de seguridad
        low_conf = self.last_conf < _CONF_STOP_THRESHOLD and target_pixel is None
        self.stop_pub.publish(Bool(data=low_conf))

        # 8. Decision de carril basada solo en la posicion de la amarilla
        self.current_lane = self._detect_lane_side(
            yellow_line = self.last_yellow_line,
            offset      = self.last_offset,
            frame_conf  = self.last_conf,
        )
        self.lane_pub.publish(String(data=self.current_lane))

        # 9. Target en metros usando homografia cacheada
        tgt_msg = Float32MultiArray()
        if target_pixel is not None:
            x_m, y_m = self._pixel_to_rear_axle_m(target_pixel[0], target_pixel[1])
            tgt_msg.data = [x_m, y_m]
        else:
            tgt_msg.data = [-1.0, -1.0]
        self.target_pub.publish(tgt_msg)

        # 10. Debug visual — publicar cada 2 frames para reducir carga WiFi
        self._debug_frame_count += 1
        if self.get_parameter('publish_debug').value and (self._debug_frame_count % 2 == 0):
            poligon = self.roi_px.astype(np.int32).reshape(1, 4, 2)
            self._publish_debug(
                bgr, yellow_mask, white_mask, masked, edges, bev_viz,
                model_sm, target_pixel, mode, poligon, header, n_px)

    # ──────────────────────────────────────────────────────────────────────────
    #  Debug visual profesional
    # ──────────────────────────────────────────────────────────────────────────
    def _publish_debug(self, bgr, yellow_mask, white_mask, masked, edges,
                       bev_viz, model, target_pixel, mode,
                       poligon, header, n_px):

        # ── Panel principal: imagen de camara con overlays ─────────────────────
        draw = (self._raw_bgr.copy() if self._raw_bgr is not None
                else bgr.copy())
        draw = cv2.resize(draw, (IMG_W, IMG_H))   # garantizar tamaño correcto

        # ── Overlay azul de pixeles detectados (yellow + sobel ∩ ROI) ─────────
        # Muestra exactamente que pixeles esta clasificando el algoritmo como linea.
        if masked is not None and masked.size > 0:
            blue_layer = np.zeros_like(draw)
            blue_layer[masked > 0] = (255, 120, 0)   # azul brillante en BGR
            draw = cv2.addWeighted(blue_layer, 0.55, draw, 1.0, 0)

        yl = model.get('yellow_line')
        cl = model.get('center_line')

        # Linea amarilla detectada (real): amarillo solido
        if yl is not None:
            is_real = (mode == 'YELLOW')
            self._draw_line(draw, yl,
                            (0, 230, 230) if is_real else (0, 200, 120),
                            3 if is_real else 2,
                            dashed=(not is_real))

        # Center line inferido (target del carril): verde brillante
        if cl is not None:
            self._draw_line(draw, cl, (0, 240, 0), 4)

        # ROI poligonal
        cv2.polylines(draw, poligon, True, (0, 220, 220), 1)

        # Target point: circulo con anillo
        if target_pixel is not None:
            tx = int(np.clip(target_pixel[0], 0, IMG_W - 1))
            ty = int(np.clip(target_pixel[1], 0, IMG_H - 1))
            cv2.circle(draw, (tx, ty), 7,  (0, 220, 255), -1)
            cv2.circle(draw, (tx, ty), 11, (0, 160, 200),  2)

        # HUD en bloque negro semitransparente en la parte superior
        mode_colors = {
            'YELLOW':   (0, 230, 230),
            'FALLBACK': (0, 140, 255),
            'NONE':     (60, 60, 230),
        }
        mc = mode_colors.get(mode, (120, 120, 120))
        prob_r = self._lane_estimator.prob_right
        side_c = self._lane_estimator.confidence

        # Fondo semitransparente para el HUD (evita que el texto se pierda sobre asfalto)
        overlay = draw.copy()
        cv2.rectangle(overlay, (0, 0), (IMG_W, 112), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.45, draw, 0.55, 0, draw)

        hud = [
            (f'MODE : {mode}',                    mc),
            (f'OFFSET: {self.last_offset:+.3f}',  (255, 255, 255)),
            (f'CONF  : {self.last_conf:.2f}',      _conf_col(self.last_conf)),
            (f'HW    : {self.dyn_half_w:.0f} px',  (170, 170, 170)),
            (f'P(R)  : {prob_r:.2f}  SC:{side_c:.2f}', (200, 200, 60)),
            (f'WAIT  : {self.waiting_cycles}',     (150, 150, 150)),
        ]
        for i, (txt, col) in enumerate(hud):
            y = 14 + i * 16
            cv2.putText(draw, txt, (4, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.40, (0, 0, 0), 2, cv2.LINE_AA)
            cv2.putText(draw, txt, (4, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.40, col, 1, cv2.LINE_AA)

        # Etiqueta de carril grande en la parte inferior
        if self.current_lane == 'right':
            lt, lc = 'CARRIL: DERECHO >>',  (0, 165, 255)
        elif self.current_lane == 'left':
            lt, lc = '<< CARRIL: IZQUIERDO', (255, 180, 0)
        else:
            lt, lc = 'CARRIL: DESCONOCIDO', (120, 120, 120)
        cv2.putText(draw, lt, (4, IMG_H - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(draw, lt, (4, IMG_H - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, lc, 2, cv2.LINE_AA)

        # ── Panel mascara: visualizacion de la segmentacion de color ──────────
        mask_panel = np.zeros((IMG_H, IMG_W, 3), dtype=np.uint8)
        mask_panel[edges > 0] = (45, 45, 45)
        mask_panel[yellow_mask > 0] = (0, 200, 255)   # naranja = amarillo
        mask_panel[white_mask  > 0] = (210, 210, 210) # gris = blanco
        # Lineas extrapoladas encima de la mascara
        if yl is not None:
            cv2.line(mask_panel, line_bot_point(yl), line_top_point(yl), (0, 230, 230), 2)
        if cl is not None:
            cv2.line(mask_panel, line_bot_point(cl), line_top_point(cl), (0, 230, 0), 2)
        # ROI en la mascara
        cv2.polylines(mask_panel, poligon, True, (0, 200, 200), 1)
        cv2.putText(mask_panel, f'PX:{n_px}', (3, 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

        # ── Panel BEV ─────────────────────────────────────────────────────────
        # bev_viz ya trae las sliding windows y la curva polinomial dibujadas
        if bev_viz is not None and bev_viz.size > 0:
            bev_panel = cv2.resize(bev_viz, (IMG_W // 2, IMG_H))
        else:
            bev_panel = np.zeros((IMG_H, IMG_W // 2, 3), dtype=np.uint8)

        # ── Panel stats ───────────────────────────────────────────────────────
        stats_w = 150
        stats = np.zeros((IMG_H, stats_w, 3), dtype=np.uint8)
        entries = [
            ('--- DETECCION ---', (160, 160, 160)),
            (f'MODE:{mode}',       mc),
            (f'OFF:{self.last_offset:+.3f}',  (255, 255, 255)),
            (f'CONF:{self.last_conf:.2f}',     _conf_col(self.last_conf)),
            (f'HW:{self.dyn_half_w:.0f}px',   (160, 160, 160)),
            ('--- CARRIL ---',    (160, 160, 160)),
            (f'LANE:{self.current_lane.upper()}', _lane_col(self.current_lane)),
            (f'P(R):{prob_r:.2f}',  (200, 200, 60)),
            (f'SC  :{side_c:.2f}',  _conf_col(side_c)),
            ('--- SISTEMA ---',  (160, 160, 160)),
            (f'WAIT:{self.waiting_cycles}', (150, 150, 150)),
            (f'PX  :{n_px}',        (130, 130, 130)),
        ]
        for i, (txt, col) in enumerate(entries):
            cv2.putText(stats, txt, (4, 16 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, col, 1, cv2.LINE_AA)

        # ── Composicion final: [camara | mascara | BEV] ──────────────────────
        # Stats omitido del stream WiFi para reducir ancho de banda (~150px menos)
        mask_show = cv2.resize(mask_panel, (IMG_W // 2, IMG_H))
        final = np.hstack([draw, mask_show, bev_panel])

        try:
            _, jpg = cv2.imencode('.jpg', final, [cv2.IMWRITE_JPEG_QUALITY, 60])
            out        = CompressedImage()
            out.header = header
            out.format = 'jpeg'
            out.data   = jpg.tobytes()
            self.dbg_pub.publish(out)
        except Exception:
            pass

    @staticmethod
    def _draw_line(img, line, color, thickness, dashed=False):
        x1, y1, x2, y2 = line
        if not dashed:
            cv2.line(img, (x1, y1), (x2, y2), color, thickness)
            return
        pts = np.linspace(0, 1, 30)
        for k in range(0, len(pts) - 1, 2):
            p1 = (int(x1 + pts[k]     * (x2 - x1)),
                  int(y1 + pts[k]     * (y2 - y1)))
            p2 = (int(x1 + pts[k + 1] * (x2 - x1)),
                  int(y1 + pts[k + 1] * (y2 - y1)))
            cv2.line(img, p1, p2, color, thickness)


# ── Helpers de color para HUD ─────────────────────────────────────────────────
def _conf_col(c):
    if c > 0.5:  return (0, 230, 0)
    if c > 0.2:  return (0, 200, 200)
    return (0, 60, 240)

def _lane_col(lane):
    if lane == 'right': return (0, 165, 255)
    if lane == 'left':  return (255, 180, 0)
    return (120, 120, 120)


# ═══════════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el worker thread limpiamente antes de cerrar el nodo
        node._stop_worker = True
        if node._worker_thread.is_alive():
            node._worker_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()