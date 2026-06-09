"""
Yellow Line Tracker — nodo independiente enfocado SOLO en la linea amarilla
divisora central. No reemplaza al lane_detector; corre en paralelo.

Pipeline simple:
  1. Recibe frame de /qcar/csi_front
  2. ROI central (recorte parametrizable, no ROI trapezoidal del detector)
  3. Mascara amarilla HSV permisiva
  4. Centroid del blob mas grande dentro del ROI → punto objetivo
  5. Publica:
       /yellow_line/image_debug   (CompressedImage)
       /yellow_line/target_point_m (Float32MultiArray [x_m, y_m])
       /yellow_line/found          (Bool)

Pensado para usarse:
  - como segundo "ojo" visualizable en el dashboard (al lado de la frontal)
  - opcionalmente, como fuente alterna del follower
    (target_point_topic:=/yellow_line/target_point_m)
"""

import threading
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, Float32MultiArray


IMG_W, IMG_H = 320, 240


class YellowLineTracker(Node):
    def __init__(self):
        super().__init__('yellow_line_tracker')

        # ── Parametros ────────────────────────────────────────────────────────
        self.declare_parameter('image_topic',  '/qcar/csi_front')
        self.declare_parameter('publish_debug', True)

        # Rangos HSV del amarillo (refuerzo). Defaults un poco mas permisivos
        # porque ahora el filtro principal es LAB+Otsu adaptativo.
        self.declare_parameter('yellow_hsv_h_min', 15)
        self.declare_parameter('yellow_hsv_h_max', 50)
        self.declare_parameter('yellow_hsv_s_min', 30)
        self.declare_parameter('yellow_hsv_v_min', 30)

        # ── Filtro adaptativo LAB + Otsu ─────────────────────────────────────
        # En LAB, canal `b` mide cuan amarillo es un pixel (indep. del brillo).
        # Otsu sobre L da el umbral T entre fondo y objetos. Aceptamos brillo
        # en torno a T (ni muy oscuro ni saturado).
        self.declare_parameter('lab_b_min',          150)   # calibrado en pista (luz mixta)
        self.declare_parameter('lab_b_max',          215)
        self.declare_parameter('brightness_low_k',   0.65)  # mas tolerante a sombras
        self.declare_parameter('brightness_high_k',  1.80)  # mas tolerante a brillo alto
        self.declare_parameter('brightness_high_cap', 245)  # tope absoluto de L_max

        # CLAHE: ecualiza contraste local del canal V antes de segmentar.
        # Aumenta el "amarillo" cuando esta en sombra sin saturar el resto.
        self.declare_parameter('clahe_clip_limit',     2.5)
        self.declare_parameter('clahe_tile_grid',      8)

        # Morfologia (dilatacion para unir fragmentos cortos)
        self.declare_parameter('morph_close_iters',    1)
        self.declare_parameter('morph_dilate_iters',   1)

        # ROI central: porcentaje del ancho/alto que se conserva.
        # roi_h_top/bottom : porcentaje del alto donde empieza/acaba el ROI
        # roi_w_margin     : porcentaje del ancho que se RECORTA en cada lado
        self.declare_parameter('roi_h_top_pct',    0.55)   # calibrado en pista
        self.declare_parameter('roi_h_bottom_pct', 0.95)
        self.declare_parameter('roi_w_margin_pct', 0.01)   # ROI mas ancho (casi todo el frame)

        # Filtro de tamano del blob: ignorar contornos muy pequenos
        self.declare_parameter('min_blob_area_px', 80)

        # Conversion pixel → metros (mismos defaults que lane_detector
        # para que sea compatible con lane_follower_pp).
        self.declare_parameter('camera_to_rear_axle_forward_m', 0.323)
        self.declare_parameter('bev_pixels_per_meter', 1000.0)
        self.declare_parameter('bev_dst_points_m', [
            0.0, 0.0,  0.0, 0.103908,
            0.309683, 0.103908,  0.309683, 0.0,
        ])
        # ROI de homografia (mismo del lane_detector). Si se cambia uno,
        # cambiar el otro para mantener calibracion coherente.
        self.declare_parameter('roi_polygon_points_px', [
            10.0, 235.0,   45.0, 125.0,
           275.0, 125.0,  310.0, 235.0,
        ])

        # ── Cache ────────────────────────────────────────────────────────────
        self.image_topic = self.get_parameter('image_topic').value

        # Homografia (igual que lane_detector) → metros desde el eje trasero
        self.bev_ppm = float(self.get_parameter('bev_pixels_per_meter').value)
        bev_flat = self.get_parameter('bev_dst_points_m').value
        self.bev_dst = np.array(bev_flat, dtype=np.float32).reshape(4, 2)
        roi_flat = self.get_parameter('roi_polygon_points_px').value
        self.roi_px = np.array(roi_flat, dtype=np.float32).reshape(4, 2)
        self._h_matrix = cv2.getPerspectiveTransform(
            self.roi_px.astype(np.float32),
            self.bev_dst * self.bev_ppm,
        )

        # Kernel morfologico
        self._k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        self._k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        # CLAHE instanciado una vez (para preprocesado adaptativo a la luz)
        tile = int(self.get_parameter('clahe_tile_grid').value)
        self._clahe = cv2.createCLAHE(
            clipLimit  = float(self.get_parameter('clahe_clip_limit').value),
            tileGridSize = (tile, tile),
        )

        # ── ROS ──────────────────────────────────────────────────────────────
        cam_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability  = QoSDurabilityPolicy.VOLATILE,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 1,
        )
        self.sub = self.create_subscription(
            CompressedImage, self.image_topic, self._cb, cam_qos)

        self.dbg_pub    = self.create_publisher(CompressedImage,
                                                '/yellow_line/image_debug', 10)
        self.target_pub = self.create_publisher(Float32MultiArray,
                                                '/yellow_line/target_point_m', 10)
        self.found_pub  = self.create_publisher(Bool,
                                                '/yellow_line/found', 10)

        # ── Worker thread CV (no bloquear el executor ROS) ──────────────────
        # El callback ROS NUNCA hace imdecode/CV; solo guarda los bytes
        # crudos del frame mas reciente. Un hilo dedicado los procesa a su
        # ritmo. Mismo patron que lane_detector.
        self._latest_msg    = None
        self._msg_lock      = threading.Lock()
        self._stop_worker   = False
        self._debug_count   = 0     # contador para frame skip del debug
        self._worker_thread = threading.Thread(
            target=self._processing_loop, daemon=True, name='yellow_cv_worker')
        self._worker_thread.start()

        self.get_logger().info(
            f'yellow_line_tracker iniciado | topic={self.image_topic}')

    # ─────────────────────────────────────────────────────────────────────────
    def _cb(self, msg):
        """Callback ROS: SOLO guarda el msg mas reciente (no procesa CV).
        El worker thread se encarga del trabajo pesado."""
        with self._msg_lock:
            self._latest_msg = msg

    def _processing_loop(self):
        """Hilo dedicado: consume el frame mas reciente y lo procesa.
        Si llegan varios mensajes mientras procesa uno, descarta los
        intermedios — el carro reacciona al "ahora", no al "hace 200ms"."""
        while not self._stop_worker:
            with self._msg_lock:
                msg, self._latest_msg = self._latest_msg, None
            if msg is None:
                # Sin frame nuevo — espera corta y reintenta
                if not rclpy.ok():
                    break
                time.sleep(0.002)   # ~2 ms entre polls
                continue
            try:
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                bgr = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                if bgr.shape[:2] != (IMG_H, IMG_W):
                    bgr = cv2.resize(bgr, (IMG_W, IMG_H),
                                     interpolation=cv2.INTER_AREA)
                self._process_frame(bgr, msg.header)
            except Exception as e:
                self.get_logger().error(f'CV worker error: {e}')

    # ─────────────────────────────────────────────────────────────────────────
    def _process_frame(self, bgr, header):
        # ── 1. ROI central rectangular ─────────────────────────────────────
        y0 = int(IMG_H * float(self.get_parameter('roi_h_top_pct').value))
        y1 = int(IMG_H * float(self.get_parameter('roi_h_bottom_pct').value))
        mx = int(IMG_W * float(self.get_parameter('roi_w_margin_pct').value))
        x0, x1 = mx, IMG_W - mx

        roi = bgr[y0:y1, x0:x1].copy()

        # ── 2. Preprocesado adaptativo: CLAHE en V ─────────────────────────
        # Sube el contraste local: si la linea esta en sombra, su amarillo
        # se realza sin saturar el resto del frame.
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        h_ch, s_ch, v_ch = cv2.split(hsv)
        v_eq   = self._clahe.apply(v_ch)
        hsv_eq = cv2.merge([h_ch, s_ch, v_eq])
        bgr_eq = cv2.cvtColor(hsv_eq, cv2.COLOR_HSV2BGR)

        # ── 2a. Mascara amarilla HSV (refuerzo, no principal) ──────────────
        h_min = int(self.get_parameter('yellow_hsv_h_min').value)
        h_max = int(self.get_parameter('yellow_hsv_h_max').value)
        s_min = int(self.get_parameter('yellow_hsv_s_min').value)
        v_min = int(self.get_parameter('yellow_hsv_v_min').value)
        mask_hsv = cv2.inRange(
            hsv_eq,
            np.array([h_min, s_min, v_min], dtype=np.uint8),
            np.array([h_max, 255,   255  ], dtype=np.uint8),
        )

        # ── 2b. Mascara amarilla LAB + brillo adaptativo (Otsu) ────────────
        lab   = cv2.cvtColor(bgr_eq, cv2.COLOR_BGR2LAB)
        l_lab = lab[:, :, 0]
        b_lab = lab[:, :, 2]

        # Cromaticidad amarillo: en LAB, b alto = amarillo (indep. brillo)
        b_min = int(self.get_parameter('lab_b_min').value)
        b_max = int(self.get_parameter('lab_b_max').value)
        mask_lab_chroma = cv2.inRange(b_lab, b_min, b_max)

        # Brillo valido: alrededor del umbral T de Otsu (fondo vs objeto)
        T_otsu, _ = cv2.threshold(l_lab, 0, 255,
                                  cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        k_low  = float(self.get_parameter('brightness_low_k').value)
        k_high = float(self.get_parameter('brightness_high_k').value)
        cap    = int(self.get_parameter('brightness_high_cap').value)
        L_min  = int(T_otsu * k_low)
        L_max  = int(min(T_otsu * k_high, cap))
        mask_brightness = cv2.inRange(l_lab, L_min, L_max)

        mask_lab = cv2.bitwise_and(mask_lab_chroma, mask_brightness)

        # ── 2c. Fusion HSV (refuerzo) + LAB (principal) ────────────────────
        mask = cv2.bitwise_or(mask_hsv, mask_lab)

        # ── 2d. Morfologia: limpia ruido y une fragmentos cortos ───────────
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._k3)
        n_close  = int(self.get_parameter('morph_close_iters').value)
        n_dilate = int(self.get_parameter('morph_dilate_iters').value)
        if n_close > 0:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._k5,
                                    iterations=n_close)
        if n_dilate > 0:
            mask = cv2.dilate(mask, self._k3, iterations=n_dilate)

        # ── 3. Centroid del blob mas grande ────────────────────────────────
        # findContours: OpenCV 3.x devuelve 3 valores, OpenCV 4.x devuelve 2.
        # Tomar siempre `cnts` como el penultimo del retorno cubre ambos casos.
        fc = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                              cv2.CHAIN_APPROX_SIMPLE)
        cnts = fc[-2]
        target_px = None
        self._candidates_debug = []
        best_cnt = None
        
        if cnts:
            max_score = -1.0
            min_area_thresh = int(self.get_parameter('min_blob_area_px').value)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < min_area_thresh:
                    continue
                
                # Obtener el rectángulo rotado mínimo para calcular elongación (aspect ratio)
                rect = cv2.minAreaRect(c)
                w, h = rect[1]
                length = max(w, h)
                width = min(w, h)
                elongation = length / (width + 1e-5)
                
                # Una caja es compacta/cuadrada (elongación ~1.0 a 1.3).
                # Exigimos elongación >= 1.1 para ser considerado candidato a línea (permite curvas)
                is_valid_line = elongation >= 1.1
                
                # Guardar para el dibujo debug
                box = cv2.boxPoints(rect)
                box = np.int0(box)
                box[:, 0] += x0
                box[:, 1] += y0
                self._candidates_debug.append({
                    'box': box,
                    'elongation': elongation,
                    'area': area,
                    'is_valid_line': is_valid_line
                })
                
                if not is_valid_line:
                    continue
                
                # Score combinado: prioriza áreas con mayor elongación
                score = area * (elongation ** 1.5)
                if score > max_score:
                    max_score = score
                    best_cnt = c

            if best_cnt is not None:
                M = cv2.moments(best_cnt)
                if M['m00'] > 0:
                    cx_roi = int(M['m10'] / M['m00'])
                    cy_roi = int(M['m01'] / M['m00'])
                    target_px = (cx_roi + x0, cy_roi + y0)

        found = target_px is not None

        # DEBUG LOGGER YELLOW TRACKER
        now_log = time.time()
        if not hasattr(self, '_last_debug_log_t'):
            self._last_debug_log_t = 0.0
        if now_log - self._last_debug_log_t > 0.5:
            self._last_debug_log_t = now_log
            num_cnts = len(cnts) if cnts else 0
            max_area = cv2.contourArea(max(cnts, key=cv2.contourArea)) if cnts else 0.0
            min_area_thresh = int(self.get_parameter('min_blob_area_px').value)
            try:
                with open('/home/nvidia/roboracer_debug.log', 'a') as f_log:
                    f_log.write(
                        f"[{now_log:.3f}] [TRACKER] found={found} "
                        f"cnts={num_cnts} max_area={max_area:.1f} "
                        f"thresh={min_area_thresh} "
                        f"img_shape={bgr.shape}\n"
                    )
            except Exception:
                pass

        self.found_pub.publish(Bool(data=found))

        # ── 4. Publicar target en metros (formato compatible con follower) ─
        tgt_msg = Float32MultiArray()
        if found:
            x_m, y_m = self._pixel_to_rear_axle_m(target_px[0], target_px[1])
            tgt_msg.data = [float(x_m), float(y_m)]
        else:
            tgt_msg.data = [-1.0, -1.0]
        self.target_pub.publish(tgt_msg)

        # ── 5. Imagen debug — solo cada 2 frames (15 FPS) ───────────────────
        # El target_point y /yellow_line/found se publican siempre (arriba),
        # pero el debug es lo que consume mas CPU (encode JPEG) y mas WiFi.
        self._debug_count += 1
        if (bool(self.get_parameter('publish_debug').value) and
                (self._debug_count % 2 == 0)):
            self._publish_debug(bgr, mask, x0, y0, x1, y1, target_px)

    # ─────────────────────────────────────────────────────────────────────────
    def _publish_debug(self, bgr, mask, x0, y0, x1, y1, target_px):
        viz = bgr.copy()

        # Sombrear fuera del ROI para que se vea el area de busqueda
        overlay = viz.copy()
        cv2.rectangle(overlay, (0, 0), (IMG_W, y0), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, y1), (IMG_W, IMG_H), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, y0), (x0, y1), (0, 0, 0), -1)
        cv2.rectangle(overlay, (x1, y0), (IMG_W, y1), (0, 0, 0), -1)
        viz = cv2.addWeighted(overlay, 0.35, viz, 0.65, 0)

        # Recuadro del ROI
        cv2.rectangle(viz, (x0, y0), (x1, y1), (0, 255, 0), 1)

        # Pintar la mascara amarilla translucida sobre el ROI
        mask_color = np.zeros_like(viz)
        mask_color[y0:y1, x0:x1][mask > 0] = (0, 255, 255)
        viz = cv2.addWeighted(viz, 1.0, mask_color, 0.55, 0)

        # Dibujar todos los candidatos y su elongación (aspect ratio)
        for cand in getattr(self, '_candidates_debug', []):
            is_winner = False
            if target_px is not None:
                # Comprobar si el target_px está dentro de este contorno (o cerca del centro de la caja)
                cx = int(np.mean(cand['box'][:, 0]))
                cy = int(np.mean(cand['box'][:, 1]))
                if abs(cx - target_px[0]) < 8 and abs(cy - target_px[1]) < 8:
                    is_winner = True
            
            # Color: verde brillante si ganó, azul si es línea válida pero no ganó, rojo si no es línea (caja)
            if is_winner:
                color = (0, 255, 0)
                thickness = 2
            elif cand['is_valid_line']:
                color = (255, 120, 0) # Celeste/azul en BGR
                thickness = 1
            else:
                color = (0, 0, 255)   # Rojo (caja de cartón/cuerpo no lineal)
                thickness = 1
                
            cv2.drawContours(viz, [cand['box']], 0, color, thickness)
            cx = int(np.mean(cand['box'][:, 0]))
            cy = int(np.mean(cand['box'][:, 1]))
            cv2.putText(viz, f"el:{cand['elongation']:.1f}", (cx - 20, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, color, 1, cv2.LINE_AA)

        # Target marker
        if target_px is not None:
            cv2.circle(viz, target_px, 7, (0, 0, 255), 2)
            cv2.line(viz, (target_px[0], y0),
                     (target_px[0], y1), (0, 0, 255), 1)

        # Label
        label = 'AMARILLO OK' if target_px else 'NO LINE'
        col   = (0, 255, 0) if target_px else (0, 0, 255)
        cv2.putText(viz, label, (8, 18), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, col, 1, cv2.LINE_AA)

        # Encode JPEG
        ok, buf = cv2.imencode('.jpg', viz, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = 'jpeg'
            msg.data = buf.tobytes()
            self.dbg_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────────
    def _pixel_to_rear_axle_m(self, px, py):
        """Misma transformacion que lane_detector → consistente para el follower."""
        pt    = np.array([[[float(px), float(py)]]], dtype=np.float32)
        bev   = cv2.perspectiveTransform(pt, self._h_matrix)[0][0]
        bev_m = bev / self.bev_ppm

        cx = 0.5 * (np.min(self.bev_dst[:, 0]) + np.max(self.bev_dst[:, 0]))
        ny = 0.5 * (self.bev_dst[0][1] + self.bev_dst[3][1])
        fy = 0.5 * (self.bev_dst[1][1] + self.bev_dst[2][1])

        lateral = float(bev_m[0] - cx)
        forward = float(np.clip(abs(bev_m[1] - ny), 0.0, abs(fy - ny)))

        y_r = forward + float(self.get_parameter('camera_to_rear_axle_forward_m').value)
        return lateral, y_r


def main(args=None):
    rclpy.init(args=args)
    node = YellowLineTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Para el worker thread antes de destruir el nodo
    node._stop_worker = True
    node._worker_thread.join(timeout=1.0)
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass


if __name__ == '__main__':
    main()
