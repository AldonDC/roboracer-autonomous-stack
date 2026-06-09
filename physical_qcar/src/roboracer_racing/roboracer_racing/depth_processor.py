#!/usr/bin/env python3
"""
Depth Processor (PROXIMIDAD por PROFUNDIDAD) — Intel RealSense D435 en QCar
============================================================================

Detecta obstaculos por la imagen de PROFUNDIDAD (no YOLO, no color), buscando
densidad de pixeles cercanos en el corredor de manejo (ROI frontal).

Hardware: Jetson + ROS 2 Dashing + Python 3.6 + OpenCV/numpy. SIN cv2.dnn.
La "camara ZED" es en realidad una Intel RealSense D435; el nodo qcar/rgbd
publica:
  /qcar/rgbd_depth  -> sensor_msgs/Image, encoding 32FC1 (metros float),
                       QoS BEST_EFFORT.
  /qcar/rgbd_color  -> sensor_msgs/CompressedImage (JPEG BGR), QoS BEST_EFFORT,
                       opcional, solo para overlay debug.

Topics publicados (CONTRATO — no cambiar; ya los consumen follower y dashboard):
  /depth/obstacle_detected          std_msgs/Bool
  /depth/closest_angle              std_msgs/Float32 (rad; >0 = IZQUIERDA, <0 = DERECHA)
  /depth/min_distance_center_10deg  std_msgs/Float32 (metros; -1.0 si no hay)
  /depth/image_debug                sensor_msgs/CompressedImage (overlay)

Pipeline (todo en numpy vectorizado, baja resolucion para fluidez):
  1. Decodificar depth (32FC1) sin cv_bridge: np.frombuffer + reshape.
  2. Downscale por factor (cv2.resize INTER_NEAREST para preservar -inf/0).
  3. ROI parametrizable en fracciones [top, bottom, left, right].
  4. Mascara = (0.15 m < depth < obstacle_dist_thresh) & finito, AND ROI.
  5. obstacle_detected = (num_pixels >= min_obstacle_pixels).
  6. closest_angle = centroide horizontal del obstaculo, mapeado por hfov.
  7. min_distance_center_10deg = percentil 10 de depth en cono central de 10°.
  8. Overlay opcional: ROI sombreada, mascara en rojo, banner, flecha, HUD FPS.

Restricciones DASHING (NUNCA romper):
  - Todo valor publicado castea a tipo nativo: Bool(data=bool(...)),
    Float32(data=float(...)). numpy.bool_/numpy.float* hacen AssertionError.
  - CompressedImage: format='jpeg', data = bytes (no np.array).
  - Excepciones manejadas para no tirar el nodo si llega un frame raro.

Parametros (ros2 param set en vivo):
  depth_topic, color_topic, obstacle_dist_thresh, min_obstacle_pixels,
  roi_top, roi_bottom, roi_left, roi_right,
  downscale, process_hz, publish_debug, hfov_deg, use_color_debug.
"""

import math
import time
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, Float32


# Profundidad minima valida (descarta el carter de la lente / objetos en cero)
MIN_VALID_DEPTH_M = 0.15
# Profundidad maxima razonable a considerar para el percentil del cono central
MAX_VALID_DEPTH_M = 10.0


class DepthProcessor(Node):
    def __init__(self):
        super().__init__('depth_processor')

        # ── Parametros ──────────────────────────────────────────────────────
        self.declare_parameter('depth_topic',         '/qcar/rgbd_depth')
        self.declare_parameter('color_topic',         '/qcar/rgbd_color')
        self.declare_parameter('obstacle_dist_thresh', 1.20)   # m
        self.declare_parameter('min_obstacle_pixels',  1200)
        self.declare_parameter('roi_top',              0.37)
        self.declare_parameter('roi_bottom',           0.60)
        self.declare_parameter('roi_left',             0.20)
        self.declare_parameter('roi_right',            0.80)
        # Por defecto bajamos resolucion a factor 4 para maximo rendimiento en Jetson
        self.declare_parameter('downscale',            4)
        self.declare_parameter('process_hz',           30.0)
        self.declare_parameter('publish_debug',        True)
        self.declare_parameter('hfov_deg',             69.0)
        # Medio-angulo del cono central (a CADA lado del centro). Define la
        # separacion de las dos lineas azules del overlay y la zona donde se
        # mide min_distance_center_10deg. Cono total = 2 * center_cone_half_deg.
        self.declare_parameter('center_cone_half_deg', 5.0)
        self.declare_parameter('debug_jpeg_quality',   70)
        # Nueva opcion para no suscribirse al topico de color pesado salvo si es necesario
        self.declare_parameter('use_color_debug',      True)
        # FIX E: deteccion de obstaculo OSCURO.
        # La RealSense D435 con proyector IR no ve objetos que absorben IR
        # (negros mate, ciertos plasticos, telas). Esos pixeles caen como
        # depth==0 / NaN / inf -> nuestra mascara los descarta como invalidos.
        # Para detectarlos: contamos pixeles "invalidos" en la ROI; si forman
        # un cluster grande, asumimos obstaculo oscuro cercano.
        self.declare_parameter('detect_dark_obstacle',       True)
        # Umbral de pixeles invalidos para flagear obstaculo oscuro.
        # Escala con downscale (igual que min_obstacle_pixels).
        self.declare_parameter('dark_obstacle_pixels',       800)

        # ── Estado interno ──────────────────────────────────────────────────
        self.latest_color = None     # bytes JPEG del color mas reciente
        self.last_process_time = 0.0
        self.min_period = 1.0 / max(1.0, float(
            self.get_parameter('process_hz').value))
        self.fps = 0.0

        # ── Worker Thread para desacoplar procesamiento ──────────────────────
        self._latest_depth_msg = None
        self._msg_lock = threading.Lock()
        self._stop_worker = False
        self._worker_thread = threading.Thread(
            target=self._processing_loop, daemon=True, name='depth_cv_worker')

        # ── QoS BEST_EFFORT depth=1 (matchea publishers de camara) ──────────
        cam_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        # ── Suscriptor de Profundidad ────────────────────────────────────────
        self.depth_sub = self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self._depth_cb,
            cam_qos)
        
        self.color_sub = None

        # ── Publicadores (contrato exacto) ──────────────────────────────────
        self.pub_detected = self.create_publisher(
            Bool, '/depth/obstacle_detected', 10)
        self.pub_closest_angle = self.create_publisher(
            Float32, '/depth/closest_angle', 10)
        self.pub_min_center_10deg = self.create_publisher(
            Float32, '/depth/min_distance_center_10deg', 10)
        self.pub_debug = self.create_publisher(
            CompressedImage, '/depth/image_debug', 10)

        # Iniciar hilo de procesamiento
        self._worker_thread.start()

        self.get_logger().info(
            "DepthProcessor OPTIMIZED ready. depth_topic=%s "
            "obstacle_thresh=%.2fm downscale=%d hz=%.0f use_color=%s"
            % (self.get_parameter('depth_topic').value,
               float(self.get_parameter('obstacle_dist_thresh').value),
               int(self.get_parameter('downscale').value),
               float(self.get_parameter('process_hz').value),
               str(self.get_parameter('use_color_debug').value)))

    def _check_color_subscription(self):
        """Gestiona dinamicamente la suscripcion a la camara color para ahorrar ancho de banda y CPU."""
        use_color = bool(self.get_parameter('use_color_debug').value)
        if use_color and self.color_sub is None:
            cam_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1)
            self.color_sub = self.create_subscription(
                CompressedImage,
                self.get_parameter('color_topic').value,
                self._color_cb,
                cam_qos)
            self.get_logger().info("Suscrito a color para overlay de depuracion.")
        elif not use_color and self.color_sub is not None:
            self.destroy_subscription(self.color_sub)
            self.color_sub = None
            self.latest_color = None
            self.get_logger().info("Desuscrito de color para optimizar CPU y red.")

    def destroy_node(self):
        self._stop_worker = True
        if self._worker_thread.is_alive():
            self._worker_thread.join(timeout=1.0)
        super().destroy_node()

    # ── Callback color: guarda bytes solo si la suscripcion esta activa ──────
    def _color_cb(self, msg):
        try:
            self.latest_color = bytes(msg.data)
        except Exception:
            self.latest_color = None

    # ── Callback depth: guarda el frame de forma no bloqueante ──────────────
    def _depth_cb(self, msg):
        with self._msg_lock:
            self._latest_depth_msg = msg

    # ── Loop de Procesamiento del Worker Thread ──────────────────────────────
    def _processing_loop(self):
        while not self._stop_worker:
            self._check_color_subscription()
            with self._msg_lock:
                msg = self._latest_depth_msg
                self._latest_depth_msg = None

            if msg is None:
                time.sleep(0.003)
                continue

            # Throttle temporal
            now = time.time()
            self.min_period = 1.0 / max(1.0, float(
                self.get_parameter('process_hz').value))
            if (now - self.last_process_time) < self.min_period:
                continue
            self.last_process_time = now

            self._process_depth_frame(msg)

    def _process_depth_frame(self, msg):
        t0 = time.time()

        # Decodificar depth 32FC1
        try:
            raw = bytes(msg.data)
            arr = np.frombuffer(raw, dtype=np.float32)
            step_floats = int(msg.step // 4)
            depth_full = arr.reshape(int(msg.height), step_floats)[:, :int(msg.width)]
        except Exception as e:
            self.get_logger().warn(
                "depth decode failed: %s" % str(e)[:120],
                throttle_duration_sec=2.0)
            return

        # Downscale para fluidez.
        downscale = max(1, int(self.get_parameter('downscale').value))
        if downscale > 1:
            nh = max(1, depth_full.shape[0] // downscale)
            nw = max(1, depth_full.shape[1] // downscale)
            depth = cv2.resize(depth_full, (nw, nh),
                               interpolation=cv2.INTER_NEAREST)
        else:
            depth = depth_full

        H, W = depth.shape[:2]

        # ── ROI ─────────────────────────────────────────────────────────────
        roi_top = float(self.get_parameter('roi_top').value)
        roi_bot = float(self.get_parameter('roi_bottom').value)
        roi_lef = float(self.get_parameter('roi_left').value)
        roi_rig = float(self.get_parameter('roi_right').value)
        
        roi_top = max(0.0, min(1.0, roi_top))
        roi_bot = max(roi_top, min(1.0, roi_bot))
        roi_lef = max(0.0, min(1.0, roi_lef))
        roi_rig = max(roi_lef, min(1.0, roi_rig))
        ry0 = int(round(roi_top * H))
        ry1 = int(round(roi_bot * H))
        rx0 = int(round(roi_lef * W))
        rx1 = int(round(roi_rig * W))
        ry1 = max(ry1, ry0 + 1)
        rx1 = max(rx1, rx0 + 1)

        # ── Mascara de obstaculo (vectorizado) ──────────────────────────────
        obs_thresh = float(self.get_parameter('obstacle_dist_thresh').value)
        min_pixels = int(self.get_parameter('min_obstacle_pixels').value)

        # Escalar dinamicamente el umbral de pixeles segun la resolucion efectiva (downscale)
        # calibrado para downscale=2 como base (min_pixels = 1200)
        scale_factor = 2.0 / downscale
        effective_min_pixels = max(10, int(min_pixels * (scale_factor ** 2)))

        roi = depth[ry0:ry1, rx0:rx1]
        mask = np.isfinite(roi) & (roi > MIN_VALID_DEPTH_M) & (roi < obs_thresh)
        num_pixels = int(np.count_nonzero(mask))
        obstacle_detected = bool(num_pixels >= effective_min_pixels)

        # FIX E (refinado): deteccion de obstaculo OSCURO.
        # Solo entramos si la deteccion normal no encontro nada.
        # La mascara CORRECTA de invalidos: NaN/inf O <= MIN_VALID_DEPTH_M.
        # (La version anterior usaba ~mask que en NumPy no captura NaN/inf
        # cuando se hace AND con roi<obs_thresh — esos no son < cualquier
        # numero, devuelven False y los excluia.)
        dark_pixels = 0
        dark_cx_centroid = None
        if (not obstacle_detected
                and bool(self.get_parameter('detect_dark_obstacle').value)):
            invalid_mask = (~np.isfinite(roi)) | (roi <= MIN_VALID_DEPTH_M)
            # Limitar a la franja central 50% para evitar bordes (cielo,
            # techo, suelo cercano) que tipicamente reportan invalido.
            roi_h, roi_w = invalid_mask.shape[:2]
            x0c = int(roi_w * 0.25)
            x1c = int(roi_w * 0.75)
            invalid_center = invalid_mask[:, x0c:x1c]
            dark_pixels = int(np.count_nonzero(invalid_center))
            dark_thresh = int(self.get_parameter('dark_obstacle_pixels').value)
            dark_thresh_eff = max(10, int(dark_thresh * (scale_factor ** 2)))
            if dark_pixels >= dark_thresh_eff:
                obstacle_detected = True
                # Centroide del cluster oscuro (en coordenadas absolutas)
                col_counts_inv = invalid_center.sum(axis=0).astype(np.float64)
                total_inv = float(col_counts_inv.sum())
                if total_inv > 0.0:
                    cols_inv = np.arange(rx0 + x0c, rx0 + x0c + col_counts_inv.shape[0],
                                          dtype=np.float64)
                    dark_cx_centroid = float(
                        (cols_inv * col_counts_inv).sum() / total_inv)
                num_pixels = dark_pixels  # para que el HUD lo muestre

        # ── Centroide horizontal del obstaculo ──────────────────────────────
        closest_angle = 0.0
        if obstacle_detected:
            # Si la deteccion vino del cluster oscuro, usa ESE centroide
            # (mas fiable que la mascara normal, que aqui estaria vacia).
            if dark_cx_centroid is not None:
                cx_centroid = dark_cx_centroid
            else:
                col_counts = mask.sum(axis=0).astype(np.float64)
                total = float(col_counts.sum())
                if total > 0.0:
                    cols_abs = np.arange(rx0, rx0 + col_counts.shape[0],
                                         dtype=np.float64)
                    cx_centroid = float((cols_abs * col_counts).sum() / total)
                else:
                    cx_centroid = (rx0 + rx1) * 0.5
            closest_angle = self._angle_from_cx(cx_centroid, W)

        # ── Distancia minima (cono central ~10° / mascara de obstaculo si detectado) ──
        hfov_deg = float(self.get_parameter('hfov_deg').value)
        cone_half_deg = float(self.get_parameter('center_cone_half_deg').value)
        cx_img = W * 0.5
        half_cols = max(1, int(round(cone_half_deg * W / max(1.0, hfov_deg))))
        c_lo = max(0, int(cx_img - half_cols))
        c_hi = min(W, int(cx_img + half_cols + 1))
        
        cone = depth[ry0:ry1, c_lo:c_hi]
        cone_mask = (np.isfinite(cone)
                     & (cone > MIN_VALID_DEPTH_M)
                     & (cone < MAX_VALID_DEPTH_M))

        if obstacle_detected and int(np.count_nonzero(mask)) > 0:
            # Si hay obstaculo detectado, usamos la distancia real de la mascara del objeto
            distance_center = float(np.percentile(roi[mask], 10.0))
        elif int(np.count_nonzero(cone_mask)) > 0:
            distance_center = float(np.percentile(cone[cone_mask], 10.0))
        else:
            # FIX E: si detectamos obstaculo OSCURO (no hay depth valida) pero
            # SI hay obstacle_detected por el cluster invalido, reportamos una
            # estimacion conservadora justo bajo obs_thresh para que entre en
            # la zona preempt del follower. Sin esto, distance=-1 hace que el
            # follower no lo considere y choca.
            if obstacle_detected and dark_pixels > 0:
                distance_center = max(MIN_VALID_DEPTH_M + 0.05,
                                      obs_thresh - 0.10)
            else:
                distance_center = -1.0

        # ── Publicar (casteado a nativo para Dashing) ──────────────────
        self.pub_detected.publish(Bool(data=bool(obstacle_detected)))
        self.pub_closest_angle.publish(Float32(data=float(closest_angle)))
        self.pub_min_center_10deg.publish(Float32(data=float(distance_center)))

        # ── Log en terminal: distancia a la que la ZED detecta el objeto ─────
        if obstacle_detected and distance_center > 0.0:
            self.get_logger().info(
                "ZED -> objeto a %.2f m  (ang %+.1f deg, %d px)"
                % (distance_center, math.degrees(closest_angle), num_pixels),
                throttle_duration_sec=0.5)

        # ── FPS ─────────────────────────────────────────────────────────────
        dt = time.time() - t0
        inst = (1.0 / dt) if dt > 1e-6 else 0.0
        self.fps = (0.9 * self.fps + 0.1 * inst) if self.fps > 0.0 else inst

        # ── Overlay debug (opcional) ────────────────────────────────────────
        if bool(self.get_parameter('publish_debug').value):
            try:
                self._render_and_publish_debug(
                    depth=depth, mask_full=None,
                    roi_box=(rx0, ry0, rx1, ry1),
                    cone_cols=(c_lo, c_hi),
                    obstacle_detected=obstacle_detected,
                    closest_angle=closest_angle,
                    distance=distance_center,
                    num_pixels=num_pixels,
                    obs_thresh=obs_thresh,
                    mask_roi=mask,
                )
            except Exception as e:
                self.get_logger().warn(
                    "overlay failed: %s" % str(e)[:120],
                    throttle_duration_sec=2.0)

    def _angle_from_cx(self, cx_box, img_w):
        hfov_deg = float(self.get_parameter('hfov_deg').value)
        cx_img = img_w * 0.5
        rel = (cx_box - cx_img) / max(1.0, cx_img)
        return float(-rel * (hfov_deg * 0.5) * (math.pi / 180.0))

    def _render_and_publish_debug(self, depth, mask_full, roi_box, cone_cols,
                                   obstacle_detected, closest_angle, distance,
                                   num_pixels, obs_thresh, mask_roi):
        H, W = depth.shape[:2]
        rx0, ry0, rx1, ry1 = roi_box
        c_lo, c_hi = cone_cols

        base = None
        if self.latest_color is not None:
            try:
                arr = np.frombuffer(self.latest_color, dtype=np.uint8)
                color = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if color is not None:
                    if (color.shape[0] != H) or (color.shape[1] != W):
                        color = cv2.resize(color, (W, H),
                                           interpolation=cv2.INTER_LINEAR)
                    base = color
            except Exception:
                base = None
        if base is None:
            d = depth.copy()
            valid = np.isfinite(d) & (d > 0.0)
            if np.any(valid):
                lo = float(np.percentile(d[valid], 2.0))
                hi = float(np.percentile(d[valid], 98.0))
                hi = max(hi, lo + 0.1)
                norm = np.clip((d - lo) / (hi - lo), 0.0, 1.0)
                norm[~valid] = 0.0
                gray = (norm * 255.0).astype(np.uint8)
            else:
                gray = np.zeros((H, W), dtype=np.uint8)
            base = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
        img = base.copy()

        overlay = img.copy()
        cv2.rectangle(overlay, (0, 0), (W, ry0), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, ry1), (W, H), (0, 0, 0), -1)
        cv2.rectangle(overlay, (0, ry0), (rx0, ry1), (0, 0, 0), -1)
        cv2.rectangle(overlay, (rx1, ry0), (W, ry1), (0, 0, 0), -1)
        img = cv2.addWeighted(overlay, 0.40, img, 0.60, 0)
        cv2.rectangle(img, (rx0, ry0), (rx1, ry1), (0, 255, 255), 1)

        if mask_roi is not None and mask_roi.size > 0:
            sub = img[ry0:ry1, rx0:rx1]
            red = np.zeros_like(sub)
            red[:, :, 2] = 255
            m3 = np.dstack([mask_roi, mask_roi, mask_roi])
            blended = np.where(m3,
                               cv2.addWeighted(sub, 0.45, red, 0.55, 0),
                               sub)
            img[ry0:ry1, rx0:rx1] = blended

        cv2.line(img, (c_lo, ry0), (c_lo, ry1), (180, 180, 0), 1, cv2.LINE_AA)
        cv2.line(img, (c_hi - 1, ry0), (c_hi - 1, ry1),
                 (180, 180, 0), 1, cv2.LINE_AA)

        if obstacle_detected:
            side_str = ("EVADE RIGHT ->" if closest_angle > 0.0
                        else "<- EVADE LEFT")
            col_arrow = (0, 255, 0) if closest_angle > 0.0 else (255, 0, 0)
            cv2.rectangle(img, (8, 8), (270, 84), (0, 0, 150), -1)
            cv2.putText(img, "ZED OBSTACLE", (16, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2,
                        cv2.LINE_AA)
            txt_dist = ("Dist: %.2fm" % distance) if distance >= 0.0 else "Dist: ---"
            cv2.putText(img, "%s  Ang: %+.1fdeg"
                        % (txt_dist, math.degrees(closest_angle)),
                        (16, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                        (255, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(img, side_str, (16, 72),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.46, col_arrow, 2,
                        cv2.LINE_AA)
            arrow_y = H - 14
            pt_start = (W // 2, arrow_y)
            shift = 80 if closest_angle > 0.0 else -80
            pt_end = (W // 2 + shift, arrow_y)
            cv2.arrowedLine(img, pt_start, pt_end, col_arrow, 3,
                            tipLength=0.3)
        else:
            cv2.rectangle(img, (8, 8), (160, 38), (40, 40, 40), -1)
            cv2.putText(img, "CLEAR", (16, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                        cv2.LINE_AA)

        # Escalar dinamicamente el umbral de pixeles segun la resolucion efectiva (downscale)
        downscale = max(1, int(self.get_parameter('downscale').value))
        min_pixels = int(self.get_parameter('min_obstacle_pixels').value)
        scale_factor = 2.0 / downscale
        effective_min_pixels = max(10, int(min_pixels * (scale_factor ** 2)))

        hud = ("FPS: %.1f  px:%d/%d  th:%.2fm"
               % (self.fps, num_pixels,
                  effective_min_pixels,
                  obs_thresh))
        cv2.putText(img, hud, (8, H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 255, 200), 1,
                    cv2.LINE_AA)
        cv2.putText(img, "ZED (PROFUNDIDAD)", (W - 170, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 220), 1,
                    cv2.LINE_AA)

        q = int(self.get_parameter('debug_jpeg_quality').value)
        q = max(40, min(95, q))
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, q])
        if ok:
            out = CompressedImage()
            out.header.stamp = self.get_clock().now().to_msg()
            out.format = 'jpeg'
            out.data = buf.tobytes()
            self.pub_debug.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
