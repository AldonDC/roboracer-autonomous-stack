"""
Lane Detector v2 — Fusión Amarillo (centro) + Blanco (borde derecho).

Semántica (Opción D):
  * Línea AMARILLA = divisoria central de la carretera (borde izq. de MI carril).
  * Línea BLANCA   = borde derecho de MI carril (curb/línea externa).
  * Centro del carril = promedio entre ambas líneas cuando hay detección
    completa, o estimación por ancho fijo cuando solo una está presente.

Pipeline:
  1. BGR -> HSV.  Dos máscaras en paralelo (amarilla, blanca).
  2. ROI trapezoidal (mitad inferior de la imagen).
  3. Canny + HoughLinesP sobre cada máscara.
  4. Fit lineal de la mejor línea por lado (amarillo → izq, blanco → der).
  5. Cálculo de centro de carril y offset normalizado [-1, +1].

Publica:
  /lane/center_offset  (std_msgs/Float32)  — offset normalizado [-1, 1].
  /lane/confidence     (std_msgs/Float32)  — 0..1 según qué líneas ve.
  /lane/image_debug    (sensor_msgs/Image) — imagen anotada.

Suscribe:
  /qcar_sim/csi_front/image_raw (sensor_msgs/Image).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
import cv2
import numpy as np
import math


def imgmsg_to_bgr(msg):
    """Decoder mínimo ROS Image -> BGR numpy.  Evita cv_bridge (ABI NumPy 1.x vs 2.x)."""
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.width, 3)
        if enc == 'rgb8':
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img
    if enc == 'mono8':
        img = buf.reshape(msg.height, msg.width)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if enc in ('bgra8', 'rgba8'):
        img = buf.reshape(msg.height, msg.width, 4)
        code = cv2.COLOR_BGRA2BGR if enc == 'bgra8' else cv2.COLOR_RGBA2BGR
        return cv2.cvtColor(img, code)
    raise ValueError(f'Encoding no soportado: {msg.encoding}')


def bgr_to_imgmsg(img, stamp_header):
    """Empaqueta BGR numpy -> sensor_msgs/Image (bgr8)."""
    msg = Image()
    msg.header = stamp_header
    msg.height, msg.width = img.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = img.tobytes()
    return msg


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        # Tópicos y flags
        self.declare_parameter('image_topic', '/qcar_sim/csi_front/image_raw')
        self.declare_parameter('publish_debug', True)

        # HSV AMARILLO (línea central)
        self.declare_parameter('h_min', 18)
        self.declare_parameter('h_max', 38)
        self.declare_parameter('s_min', 80)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 80)
        self.declare_parameter('v_max', 255)

        # HSV BLANCO (borde derecho): baja saturación + valor medio-alto.
        # Valores permisivos porque el curb de la pista es gris-claro, no blanco puro.
        self.declare_parameter('wh_s_max', 80)
        self.declare_parameter('wh_v_min', 140)

        # ROI y geometría del carril
        self.declare_parameter('roi_top_frac', 0.55)
        # Trapecio: fracciones x del ROI en top (far) y bottom (near).
        self.declare_parameter('roi_top_left_frac', 0.20)
        self.declare_parameter('roi_top_right_frac', 0.80)
        self.declare_parameter('roi_bot_left_frac', 0.02)
        self.declare_parameter('roi_bot_right_frac', 0.98)
        # Ancho medio de carril (half-width) como fracción del ancho de imagen.
        self.declare_parameter('lane_half_width_frac', 0.28)

        # ── Detección de señal STOP (multi-cue) ──────────────────────────────
        # Área (px) — filtro de distancia. Señales lejanas dan blobs pequeños:
        # subir min_area para ignorarlas y disparar solo a media/cerca.
        self.declare_parameter('stop_min_area', 1200)
        self.declare_parameter('stop_max_area', 60000)
        # Confianza mínima por frame para sumar un voto.
        self.declare_parameter('stop_min_confidence', 0.50)
        # Nº de frames consecutivos con voto positivo para confirmar.
        self.declare_parameter('stop_vote_needed', 3)
        # Fracción de altura desde arriba donde se busca la señal (recorta suelo).
        self.declare_parameter('stop_search_top_frac', 0.02)
        self.declare_parameter('stop_search_bot_frac', 0.80)

        self.publish_debug = self.get_parameter('publish_debug').value

        image_topic = self.get_parameter('image_topic').value
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 10)

        self.offset_pub = self.create_publisher(Float32, '/lane/center_offset', 10)
        self.conf_pub = self.create_publisher(Float32, '/lane/confidence', 10)
        self.dbg_pub = self.create_publisher(Image, '/lane/image_debug', 10)
        self.stop_pub = self.create_publisher(Bool, '/lane/stop_sign', 10)

        self.last_offset = 0.0
        self.last_conf = 0.0
        self.stop_vote_counter = 0        # Filtro temporal para STOP
        self.last_stop_conf = 0.0         # Última confianza (0..1) del frame
        self.last_stop_bbox = None        # Última bbox (x,y,w,h) o None

        self.get_logger().info(
            f'👁️  LANE DETECTOR v2 — Amarillo(centro) + Blanco(borde) — sub: {image_topic}')

    # ── Mascaras ──────────────────────────────────────────────────────────────
    def _yellow_mask(self, hsv):
        lo = np.array([
            self.get_parameter('h_min').value,
            self.get_parameter('s_min').value,
            self.get_parameter('v_min').value,
        ], dtype=np.uint8)
        hi = np.array([
            self.get_parameter('h_max').value,
            self.get_parameter('s_max').value,
            self.get_parameter('v_max').value,
        ], dtype=np.uint8)
        m = cv2.inRange(hsv, lo, hi)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        return m

    def _white_mask(self, hsv):
        # Blanco = cualquier H, S baja, V alta.
        lo = np.array([0, 0, self.get_parameter('wh_v_min').value], dtype=np.uint8)
        hi = np.array([179, self.get_parameter('wh_s_max').value, 255], dtype=np.uint8)
        m = cv2.inRange(hsv, lo, hi)
        k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        return m

    # ── Máscara ROJA robusta (multi-rango + CLAHE) ──────────────────────────
    def _red_mask(self, hsv):
        """Detecta rojo con 3 bandas HSV + ecualización adaptativa del valor.

        Por qué 3 bandas: el rojo cruza el círculo de Hue (0 y 180). Además
        añadimos una tercera banda con saturación relajada para capturar
        rojos descoloridos por sombra o sobre-exposición (iluminación variable).
        CLAHE sobre el canal V compensa cambios de brillo sin afectar color,
        así la señal sigue detectándose en sol fuerte, sombra o contraluz.
        """
        h_ch, s_ch, v_ch = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        v_eq = clahe.apply(v_ch)
        hsv_eq = cv2.merge([h_ch, s_ch, v_eq])

        # Banda 1: rojo "clásico" cerca de H=0
        lo1, hi1 = np.array([0, 70, 55]), np.array([12, 255, 255])
        # Banda 2: rojo "clásico" cerca de H=180 (wrap-around)
        lo2, hi2 = np.array([165, 70, 55]), np.array([179, 255, 255])
        # Banda 3: rojo desaturado / sombra — permite S baja con V medio-alto
        lo3, hi3 = np.array([0, 35, 110]), np.array([15, 255, 255])
        lo4, hi4 = np.array([160, 35, 110]), np.array([179, 255, 255])

        m = cv2.inRange(hsv_eq, lo1, hi1)
        m = cv2.bitwise_or(m, cv2.inRange(hsv_eq, lo2, hi2))
        m = cv2.bitwise_or(m, cv2.inRange(hsv_eq, lo3, hi3))
        m = cv2.bitwise_or(m, cv2.inRange(hsv_eq, lo4, hi4))

        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
        return m

    # ── Detección de señal STOP (multi-cue) ──────────────────────────────────
    def _detect_stop_sign(self, bgr, hsv):
        """Retorna (confianza [0..1], bbox|None).

        Combina 4 señales visuales (todas invariantes a orientación):
          • COLOR:   máscara roja robusta (CLAHE + 3 bandas HSV).
          • FORMA:   octágono vía approxPolyDP (6..10 vértices) + compacidad.
          • TEXTO:   ratio de píxeles blancos dentro del blob rojo (letras
                     STOP + borde → ~8..45%). Funciona con la señal al revés.
          • POS:     sesgo por mitad superior de la imagen.
        La confianza final es un mix ponderado, robusta a iluminación,
        rotación (incluyendo señal invertida) y oclusión parcial.
        """
        h_img, w_img = bgr.shape[:2]
        mask = self._red_mask(hsv)

        # Recorte vertical: ignora cielo y suelo (mejor SNR y más rápido).
        top_cut = int(h_img * float(self.get_parameter('stop_search_top_frac').value))
        bot_cut = int(h_img * float(self.get_parameter('stop_search_bot_frac').value))
        if top_cut > 0:
            mask[:top_cut, :] = 0
        if bot_cut < h_img:
            mask[bot_cut:, :] = 0

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        min_area = float(self.get_parameter('stop_min_area').value)
        max_area = float(self.get_parameter('stop_max_area').value)

        best_score = 0.0
        best_bbox = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue
            x, y, w_c, h_c = cv2.boundingRect(cnt)
            if w_c < 10 or h_c < 10:
                continue

            aspect = float(w_c) / float(h_c)
            if aspect < 0.55 or aspect > 1.85:   # octágono ≈ cuadrado
                continue

            bbox_area = float(w_c * h_c)
            solidity = area / bbox_area          # octágono ≈ 0.83
            if solidity < 0.45:
                continue

            # Forma: nº vértices del polígono simplificado.
            peri = cv2.arcLength(cnt, True)
            if peri < 1e-3:
                continue
            approx = cv2.approxPolyDP(cnt, 0.035 * peri, True)
            n_vert = len(approx)
            if 6 <= n_vert <= 10:
                shape_score = 1.0
            elif 4 <= n_vert <= 12:
                shape_score = 0.6
            else:
                shape_score = 0.25

            # Compacidad 4πA/P² — octágono ≈ 0.95, cuadrado ≈ 0.79
            compactness = 4.0 * math.pi * area / (peri * peri + 1e-6)
            if compactness < 0.45:
                continue

            # Texto blanco dentro de la región roja (invariante a rotación).
            roi_hsv = hsv[y:y + h_c, x:x + w_c]
            if roi_hsv.size == 0:
                continue
            white_mask = cv2.inRange(roi_hsv,
                                     np.array([0, 0, 170], dtype=np.uint8),
                                     np.array([179, 75, 255], dtype=np.uint8))
            roi_pixels = float(max(1, w_c * h_c))
            white_ratio = float(np.count_nonzero(white_mask)) / roi_pixels
            if 0.02 <= white_ratio <= 0.65:
                # Pico en ~20%; lineal hacia los bordes del intervalo.
                text_score = 1.0 - abs(white_ratio - 0.20) / 0.45
                text_score = float(max(0.3, min(1.0, text_score)))
            else:
                # Sin texto claro: puede ser una señal lejana → no descartar.
                text_score = 0.15

            # Posición: recompensa mitad superior (señales cuelgan en alto).
            cy = y + h_c * 0.5
            if cy < h_img * 0.55:
                pos_score = 1.0
            else:
                pos_score = max(0.4, 1.0 - (cy - h_img * 0.55) / (h_img * 0.30))

            # Prior de tamaño: más grande → más probable confirmada.
            size_score = float(np.clip(
                (area - min_area) / max(1.0, max_area - min_area), 0.0, 1.0))
            size_score = 0.3 + 0.7 * size_score

            score = (0.35 * shape_score
                     + 0.25 * text_score
                     + 0.15 * pos_score
                     + 0.15 * size_score
                     + 0.10 * min(1.0, compactness * 1.05))

            if score > best_score:
                best_score = score
                best_bbox = (x, y, w_c, h_c)

        return float(best_score), best_bbox

    def _apply_roi(self, mask):
        h, w = mask.shape
        top = int(h * self.get_parameter('roi_top_frac').value)
        tl = self.get_parameter('roi_top_left_frac').value
        tr = self.get_parameter('roi_top_right_frac').value
        bl = self.get_parameter('roi_bot_left_frac').value
        br = self.get_parameter('roi_bot_right_frac').value
        poly = np.array([[
            (int(bl * w), h),
            (int(tl * w), top),
            (int(tr * w), top),
            (int(br * w), h),
        ]], dtype=np.int32)
        roi = np.zeros_like(mask)
        cv2.fillPoly(roi, poly, 255)
        return cv2.bitwise_and(mask, roi), poly

    # ── Fit ───────────────────────────────────────────────────────────────────
    @staticmethod
    def _fit_line(points, img_h):
        """Ajuste x = m*y + b.  Devuelve (x_bottom, y_bottom, x_top, y_top) o None."""
        if len(points) < 2:
            return None
        pts = np.array(points)
        xs = pts[:, 0].astype(np.float32)
        ys = pts[:, 1].astype(np.float32)
        if np.ptp(ys) < 5.0:
            return None
        m, b = np.polyfit(ys, xs, 1)
        return (int(m * img_h + b), int(img_h),
                int(m * (img_h * 0.6) + b), int(img_h * 0.6))

    @staticmethod
    def _hough(mask_roi):
        edges = cv2.Canny(mask_roi, 60, 180)
        return cv2.HoughLinesP(
            edges, rho=1, theta=np.pi / 180,
            threshold=25, minLineLength=20, maxLineGap=40,
        )

    @staticmethod
    def _collect_segments(lines, side, img_cx):
        """side = 'left' | 'right' | 'any'. Devuelve lista de (x,y)."""
        pts = []
        if lines is None:
            return pts
        for ln in lines:
            x1, y1, x2, y2 = ln[0]
            dy = y2 - y1
            if abs(dy) < 6:
                continue  # líneas horizontales → basura
            mid_x = (x1 + x2) / 2.0
            if side == 'left' and mid_x >= img_cx:
                continue
            if side == 'right' and mid_x <= img_cx:
                continue
            pts.extend([(x1, y1), (x2, y2)])
        return pts

    # ── Callback principal ────────────────────────────────────────────────────
    def image_cb(self, msg):
        try:
            bgr = imgmsg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f'decode error: {e}', throttle_duration_sec=2.0)
            return

        h, w, _ = bgr.shape
        img_cx = w / 2.0
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Mascaras y ROI
        y_mask = self._yellow_mask(hsv)
        w_mask = self._white_mask(hsv)
        y_roi, poly = self._apply_roi(y_mask)
        w_roi, _ = self._apply_roi(w_mask)

        # ── Detección STOP (multi-cue: color + forma + texto + posición) ────
        stop_conf, stop_bbox = self._detect_stop_sign(bgr, hsv)
        min_conf = float(self.get_parameter('stop_min_confidence').value)
        vote_needed = int(self.get_parameter('stop_vote_needed').value)

        # Voto temporal ponderado por confianza: alta confianza acelera el
        # consenso; baja confianza decae suave (no desaparece con 1 frame malo).
        if stop_conf >= min_conf:
            delta_votes = 2 if stop_conf >= 0.65 else 1
            self.stop_vote_counter = min(self.stop_vote_counter + delta_votes, 10)
        else:
            self.stop_vote_counter = max(self.stop_vote_counter - 1, 0)

        stop_confirmed = (self.stop_vote_counter >= vote_needed)
        candidate_found = stop_conf >= min_conf
        self.last_stop_conf = stop_conf
        self.last_stop_bbox = stop_bbox

        # Amarillo → línea izquierda (divisoria central)
        y_lines = self._hough(y_roi)
        y_pts = self._collect_segments(y_lines, 'left', img_cx)
        yellow_fit = self._fit_line(y_pts, h)

        # Blanco → línea derecha (borde exterior)
        w_lines = self._hough(w_roi)
        w_pts = self._collect_segments(w_lines, 'right', img_cx)
        white_fit = self._fit_line(w_pts, h)

        half_w_px = float(self.get_parameter('lane_half_width_frac').value) * w

        # ── Modelo de centro de carril ────────────────────────────────────────
        lane_cx = None
        conf = 0.0
        mode = 'NONE'
        if yellow_fit and white_fit:
            lane_cx = (yellow_fit[0] + white_fit[0]) / 2.0
            conf = 1.0
            mode = 'FULL'
        elif yellow_fit:
            # Amarillo = borde izq de MI carril → centro está media anchura a la der.
            lane_cx = yellow_fit[0] + half_w_px
            conf = 0.70
            mode = 'YELLOW-ONLY'
        elif white_fit:
            # Blanco = borde der → centro está media anchura a la izq.
            lane_cx = white_fit[0] - half_w_px
            conf = 0.55
            mode = 'WHITE-ONLY'

        # ── Offset + suavizado ────────────────────────────────────────────────
        if lane_cx is not None:
            raw_offset = (lane_cx - img_cx) / (w / 2.0)
            raw_offset = float(np.clip(raw_offset, -1.0, 1.0))
            offset_norm = 0.7 * self.last_offset + 0.3 * raw_offset
        else:
            offset_norm = 0.85 * self.last_offset

        self.last_offset = offset_norm
        self.last_conf = 0.7 * self.last_conf + 0.3 * conf

        # Publicar
        self.offset_pub.publish(Float32(data=float(offset_norm)))
        self.conf_pub.publish(Float32(data=float(self.last_conf)))
        self.stop_pub.publish(Bool(data=bool(stop_confirmed)))

        # ── Debug overlay ─────────────────────────────────────────────────────
        if self.publish_debug:
            dbg = bgr.copy()
            # Pinta amarillo detectado en amarillo translúcido, blanco en blanco
            overlay = dbg.copy()
            overlay[y_roi > 0] = (0, 255, 255)
            overlay[w_roi > 0] = (255, 255, 255)
            dbg = cv2.addWeighted(overlay, 0.35, dbg, 0.65, 0)
            cv2.polylines(dbg, poly, True, (255, 120, 0), 2)

            # Líneas detectadas
            if yellow_fit:
                cv2.line(dbg, (yellow_fit[0], yellow_fit[1]),
                         (yellow_fit[2], yellow_fit[3]), (0, 230, 0), 4)
                cv2.putText(dbg, 'Y', (yellow_fit[0] - 22, yellow_fit[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            if white_fit:
                cv2.line(dbg, (white_fit[0], white_fit[1]),
                         (white_fit[2], white_fit[3]), (220, 220, 220), 4)
                cv2.putText(dbg, 'W', (white_fit[0] + 8, white_fit[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # Centro de imagen — guía vertical larga (referencia fija)
            cx_i = int(img_cx)
            cv2.line(dbg, (cx_i, h), (cx_i, int(h * 0.55)),
                     (180, 180, 180), 1, cv2.LINE_AA)
            cv2.putText(dbg, 'IMG', (cx_i + 4, int(h * 0.55) + 14),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

            # Centro de carril detectado — guía vertical de color
            if lane_cx is not None:
                lx = int(lane_cx)
                cv2.line(dbg, (lx, h), (lx, int(h * 0.62)),
                         (0, 0, 255), 2, cv2.LINE_AA)
                # Cross grande en la base
                cv2.drawMarker(dbg, (lx, h - 18), (0, 0, 255),
                               markerType=cv2.MARKER_CROSS,
                               markerSize=22, thickness=3)
                # Flecha de error (del centro imagen al centro carril)
                if abs(lx - cx_i) > 3:
                    cv2.arrowedLine(dbg, (cx_i, h - 45), (lx, h - 45),
                                    (0, 120, 255), 3, tipLength=0.25)
            
            # ── Alerta de señal STOP (bbox + confianza + badge) ─────────────
            if self.last_stop_bbox is not None:
                bx, by, bw, bh = self.last_stop_bbox
                box_color = (0, 0, 255) if stop_confirmed else (
                    0, 165, 255) if candidate_found else (0, 220, 220)
                cv2.rectangle(dbg, (bx, by), (bx + bw, by + bh), box_color, 2)
                label = f'STOP? {self.last_stop_conf:.2f} v={self.stop_vote_counter}'
                cv2.putText(dbg, label, (bx, max(14, by - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, box_color, 2)
            if stop_confirmed:
                cv2.rectangle(dbg, (w - 180, 20), (w - 20, 82),
                              (0, 0, 255), -1)
                cv2.putText(dbg, 'STOP', (w - 160, 65),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 4)
                cv2.drawMarker(dbg, (w - 100, 120), (0, 0, 255),
                               cv2.MARKER_TILTED_CROSS, 30, 5)
            elif candidate_found:
                cv2.circle(dbg, (w - 90, 50), 10, (0, 165, 255), -1)

            # ── HUD superior — cuadro de estado centrado ─────────────────────
            mode_color = {
                'FULL':         (0, 255,   0),
                'YELLOW-ONLY':  (0, 220, 255),
                'WHITE-ONLY':   (200, 200, 200),
                'NONE':         (80,  80, 255),
            }.get(mode, (0, 255, 255))

            line1 = f'{mode}'
            line2 = f'off={offset_norm:+.2f}   conf={self.last_conf:.2f}'
            # Mide tamaño para centrar
            (tw1, th1), _ = cv2.getTextSize(line1, cv2.FONT_HERSHEY_SIMPLEX,
                                            0.8, 2)
            (tw2, th2), _ = cv2.getTextSize(line2, cv2.FONT_HERSHEY_SIMPLEX,
                                            0.65, 2)
            box_w = max(tw1, tw2) + 24
            box_h = th1 + th2 + 28
            box_x = cx_i - box_w // 2
            box_y = 10
            cv2.rectangle(dbg, (box_x, box_y),
                          (box_x + box_w, box_y + box_h), (0, 0, 0), -1)
            cv2.rectangle(dbg, (box_x, box_y),
                          (box_x + box_w, box_y + box_h), mode_color, 2)
            cv2.putText(dbg, line1,
                        (cx_i - tw1 // 2, box_y + th1 + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
            cv2.putText(dbg, line2,
                        (cx_i - tw2 // 2, box_y + th1 + th2 + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

            try:
                self.dbg_pub.publish(bgr_to_imgmsg(dbg, msg.header))
            except Exception as e:
                self.get_logger().warn(f'debug publish error: {e}',
                                       throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
