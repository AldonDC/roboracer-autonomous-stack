"""
Physical QCar Dashboard v4 — Qt + PyQtGraph (30 FPS)
=====================================================
Layout 2 columnas:
  IZQUIERDA (status): System / Lane / Lidar metrics + Telemetry plots
  DERECHA  (visual) : LiDAR Radar (hero) + Camara frontal + Lane debug

Topics:
  /qcar/csi_front      CompressedImage   → Camera panel
  /lane/image_debug    CompressedImage   → Lane debug panel
  /lidar/image_debug   CompressedImage   → Radar panel (render desde Jetson)
  /depth/image_debug   CompressedImage   → Camara ZED (deteccion obstaculos)
  /depth/obstacle_detected Bool          → indicador ZED OBSTACLE
  /depth/min_distance_center_10deg Float32 → distancia obstaculo ZED
  /lidar/min_distance  Float32
  /lidar/closest_angle Float32
  /lidar/obstacle      Bool
  /lane/center_offset  Float32
  /lane/confidence     Float32
  /lane/stop_sign      Bool
  /qcar/velocity       Vector3Stamped
  /qcar/stateBattery   sensor_msgs/BatteryState
"""
import sys, threading, math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSDurabilityPolicy, QoSHistoryPolicy)
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import CompressedImage, BatteryState
from geometry_msgs.msg import Vector3Stamped
from collections import deque

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                               QHBoxLayout, QLabel, QGroupBox,
                               QGridLayout, QSizePolicy, QSplitter)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
import pyqtgraph as pg

# ── Colores ───────────────────────────────────────────────────────────────────
BG     = '#0D1117';  CARD   = '#161B22';  BORDER = '#30363D'
GREEN  = '#3FB950';  YELLOW = '#E3B341';  RED    = '#FF4444'
CYAN   = '#00E5CC';  WHITE  = '#E6EDF3';  DIM    = '#8B949E'
M_BLUE = '#0072BD';  M_RED  = '#D95319';  M_YEL  = '#EDB120'
M_PURP = '#7E2F8E';  PLOT_BG= '#F6F8FC';  GRID_C = '#DDE4EF'

CAM_W, CAM_H = 320, 240
OBS_TH = 0.6       # m — coincide con lidar_processor
WARN_TH = 1.5      # m
LIDAR_MAX_R = 5.0  # m


def _compressed_to_bgr(msg):
    try:
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None


# ── ROS Backend ───────────────────────────────────────────────────────────────
class ROSBackend(Node):
    def __init__(self):
        super().__init__('physical_dashboard')
        # QoS BEST_EFFORT depth=1 para video — coincide con csinode (Jetson)
        # y nunca acumula frames viejos.
        _be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)

        # Imagenes
        self.create_subscription(CompressedImage, '/qcar/csi_front',
                                 self._cam_cb, _be)
        self.create_subscription(CompressedImage, '/lane/image_debug',
                                 self._lane_cb, _be)
        self.create_subscription(CompressedImage, '/lidar/image_debug',
                                 self._lidar_img_cb, _be)
        self.create_subscription(CompressedImage, '/yellow_line/image_debug',
                                 self._yellow_cb, _be)
        # Camara de profundidad ("ZED") — suscripcion dinamica y opcional
        self.declare_parameter('show_zed_image', True)
        self.zed_sub = None

        # Lane
        self.create_subscription(Float32, '/lane/center_offset', self._off_cb, 10)
        self.create_subscription(Float32, '/lane/confidence', self._conf_cb, 10)
        self.create_subscription(Bool, '/lane/stop_sign', self._stop_cb, 10)

        # Telemetria
        self.create_subscription(Vector3Stamped, '/qcar/velocity', self._vel_cb, _be)
        self.create_subscription(BatteryState, '/qcar/stateBattery', self._bat_cb, 10)

        # LIDAR metricas
        self.create_subscription(Float32, '/lidar/min_distance', self._lmin_cb, 10)
        self.create_subscription(Float32, '/lidar/closest_angle', self._lang_cb, 10)
        self.create_subscription(Bool, '/lidar/obstacle', self._lobs_cb, 10)

        # ZED (camara de profundidad) metricas
        self.create_subscription(Bool, '/depth/obstacle_detected', self._zobs_cb, 10)
        self.create_subscription(Float32, '/depth/min_distance_center_10deg',
                                 self._zdist_cb, 10)

        # Autonomous state
        self.create_subscription(Bool, '/lane/auto_active', self._auto_cb, 10)

        self.lock = threading.Lock()

        # Frames: guardamos los BYTES CRUDOS de cada CompressedImage, no la
        # imagen decodificada. El decode JPEG (5-15ms) se hace en el hilo Qt
        # solo cuando vamos a mostrar — un decode por tick, no por mensaje.
        # Esto evita que el callback ROS bloquee el executor en cada frame.
        self.raw_cam   = None;   self.raw_lane  = None;   self.raw_lidar = None
        self.raw_yellow = None;  self.raw_zed = None
        self.frame_cam = None;   self.frame_lane = None;  self.frame_lidar = None
        self.frame_yellow = None; self.frame_zed = None
        self.fps_cam = 0.0;      self.fps_lane = 0.0;     self.fps_lidar = 0.0
        self.fps_yellow = 0.0;   self.fps_zed = 0.0
        self._t_cam = 0.0;       self._t_lane = 0.0;      self._t_lidar = 0.0
        self._t_yellow = 0.0;    self._t_zed = 0.0

        # Lane state
        self.offset = 0.0;       self.conf = 0.0
        self.stop_flag = False;  self.offset_t = 0.0
        self.auto_active = False

        # Telemetria
        self.vel_x = 0.0;        self.vel_y = 0.0
        self.battery_v = 0.0;    self.battery_t = 0.0

        # Lidar metricas
        self.lidar_min = -1.0;   self.lidar_ang = 0.0
        self.lidar_obs = False;  self.lidar_t = 0.0

        # ZED (camara de profundidad) metricas
        self.zed_obs = False;    self.zed_dist = -1.0;    self.zed_t = 0.0

        # Historial graficas
        N = 300
        self.t_start = None
        self.h_time   = deque(maxlen=N)
        self.h_vel    = deque(maxlen=N)
        self.h_offset = deque(maxlen=N)
        self.h_conf   = deque(maxlen=N)
        self.h_lidar  = deque(maxlen=N)

        self.get_logger().info('Dashboard v4 — Layout 2 columnas — QCar fisico')

    # ── Callbacks de imagenes (LIGEROS: solo guardan bytes crudos) ───────────
    # NO se decodifica aqui — eso bloquearia el executor ROS en cada frame.
    # El decode JPEG se hace en _update() del hilo Qt, solo para el frame
    # mas reciente, una vez por tick.
    def _cam_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.raw_cam = msg.data
            self.fps_cam = 1.0 / max(now - self._t_cam, 0.001)
            self._t_cam = now

    def _lane_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.raw_lane = msg.data
            self.fps_lane = 1.0 / max(now - self._t_lane, 0.001)
            self._t_lane = now

    def _lidar_img_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.raw_lidar = msg.data
            self.fps_lidar = 1.0 / max(now - self._t_lidar, 0.001)
            self._t_lidar = now

    def _yellow_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.raw_yellow = msg.data
            self.fps_yellow = 1.0 / max(now - self._t_yellow, 0.001)
            self._t_yellow = now

    def _depth_cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.raw_zed = msg.data
            self.fps_zed = 1.0 / max(now - self._t_zed, 0.001)
            self._t_zed = now

    # ── Callbacks de datos ───────────────────────────────────────────────────
    def _off_cb(self, msg):
        self.offset = float(msg.data)
        self.offset_t = self.get_clock().now().nanoseconds / 1e9

    def _conf_cb(self, msg):
        self.conf = float(msg.data)

    def _stop_cb(self, msg):
        self.stop_flag = bool(msg.data)

    def _auto_cb(self, msg):
        self.auto_active = bool(msg.data)

    def _check_zed_subscription(self):
        show_zed = bool(self.get_parameter('show_zed_image').value)
        if show_zed and self.zed_sub is None:
            _be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=1)
            self.zed_sub = self.create_subscription(
                CompressedImage, '/depth/image_debug',
                self._depth_cb, _be)
            self.get_logger().info("Suscrito a /depth/image_debug en el dashboard.")
        elif not show_zed and self.zed_sub is not None:
            self.destroy_subscription(self.zed_sub)
            self.zed_sub = None
            with self.lock:
                self.raw_zed = None
                self.frame_zed = None
            self.get_logger().info("Desuscrito de /depth/image_debug en el dashboard.")

    def _vel_cb(self, msg):
        self.vel_x = float(msg.vector.x)
        self.vel_y = float(msg.vector.y)
        now = self.get_clock().now().nanoseconds / 1e9
        if self.t_start is None:
            self.t_start = now
        t = now - self.t_start
        v = math.hypot(self.vel_x, self.vel_y)
        with self.lock:
            self.h_time.append(t)
            self.h_vel.append(v)
            self.h_offset.append(self.offset)
            self.h_conf.append(self.conf)
            self.h_lidar.append(self.lidar_min if self.lidar_min >= 0 else LIDAR_MAX_R)

    def _bat_cb(self, msg):
        self.battery_v = float(msg.voltage)
        self.battery_t = self.get_clock().now().nanoseconds / 1e9

    def _lmin_cb(self, msg):
        self.lidar_min = float(msg.data)
        self.lidar_t = self.get_clock().now().nanoseconds / 1e9

    def _lang_cb(self, msg):
        self.lidar_ang = float(msg.data)

    def _lobs_cb(self, msg):
        self.lidar_obs = bool(msg.data)

    def _zobs_cb(self, msg):
        self.zed_obs = bool(msg.data)
        self.zed_t = self.get_clock().now().nanoseconds / 1e9

    def _zdist_cb(self, msg):
        self.zed_dist = float(msg.data)
        self.zed_t = self.get_clock().now().nanoseconds / 1e9


# ── Dashboard GUI ─────────────────────────────────────────────────────────────
class Dashboard(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('QCar Physical Dashboard v4')
        self.setMinimumSize(1500, 850)
        self._style()

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setSpacing(6); root.setContentsMargins(6, 6, 6, 6)

        splitter = QSplitter(Qt.Horizontal)
        root.addWidget(splitter)

        # ── IZQUIERDA (status + plots) ────────────────────────────────────────
        left = QWidget(); lv = QVBoxLayout(left)
        lv.setSpacing(4); lv.setContentsMargins(0, 0, 0, 0)
        lv.addWidget(self._build_system_status())
        lv.addWidget(self._build_lane_status())
        lv.addWidget(self._build_lidar_status())
        lv.addWidget(self._build_plots(), 1)
        splitter.addWidget(left)

        # ── DERECHA (visual) ──────────────────────────────────────────────────
        right = QWidget(); rv = QVBoxLayout(right)
        rv.setSpacing(4); rv.setContentsMargins(0, 0, 0, 0)
        rv.addWidget(self._cam_box('LIDAR RADAR — /lidar/image_debug', 'lidar'), 4)

        # Fila combinada: frontal + yellow_line lado a lado, mismo ancho
        front_row = QWidget()
        front_hl = QHBoxLayout(front_row)
        front_hl.setSpacing(4); front_hl.setContentsMargins(0, 0, 0, 0)
        front_hl.addWidget(self._cam_box('CAMARA FRONTAL — /qcar/csi_front', 'cam'), 1)
        front_hl.addWidget(self._cam_box('YELLOW TRACKER — /yellow_line/image_debug', 'yellow'), 1)
        front_hl.addWidget(self._cam_box('ZED (PROFUNDIDAD) — /depth/image_debug', 'zed'), 1)
        rv.addWidget(front_row, 3)

        rv.addWidget(self._cam_box('LANE DEBUG — /lane/image_debug', 'lane'), 4)
        splitter.addWidget(right)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 7)
        splitter.setSizes([420, 1080])

        # Cache de tamaños para evitar recalcular en cada frame
        self._last_sizes = {}

        # Dos timers independientes para evitar que el render pesado bloquee el video:
        #   - _timer_video (33ms = 30 FPS) → SOLO paneles de imagen
        #   - _timer_data  (200ms = 5 FPS) → plots, labels, stats
        # Los datos no necesitan 20 FPS de refresco; el video sí.
        self._timer_video = QTimer()
        self._timer_video.timeout.connect(self._update_video)
        self._timer_video.start(66)  # 15 FPS — much lighter on CPU!

        self._timer_data = QTimer()
        self._timer_data.timeout.connect(self._update_data)
        self._timer_data.start(200)

    def _style(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background:{BG}; color:{WHITE}; font-family:'DejaVu Sans'; }}
            QGroupBox {{ border:1px solid {BORDER}; border-radius:6px; margin-top:10px;
                         padding:4px; padding-top:14px; font-weight:bold; font-size:10px; color:{DIM}; }}
            QGroupBox::title {{ subcontrol-origin:margin; left:8px; color:{CYAN}; }}
            QSplitter::handle {{ background:{BORDER}; }}
        """)

    # ── System Status ────────────────────────────────────────────────────────
    def _build_system_status(self):
        grp = QGroupBox('System Status')
        g = QGridLayout(grp); g.setSpacing(4)
        def mk(t, c=WHITE, s=10, b=False):
            l = QLabel(t); l.setStyleSheet(f'color:{c};font-size:{s}px;font-weight:{"bold" if b else "normal"};')
            return l
        g.addWidget(mk('ROS:', DIM), 0, 0)
        self.l_ros = mk('OK', GREEN, 11, True); g.addWidget(self.l_ros, 0, 1)
        g.addWidget(mk('Piloto:', DIM), 1, 0)
        self.l_pilot = mk('MANUAL', DIM, 12, True); g.addWidget(self.l_pilot, 1, 1)
        g.addWidget(mk('Bateria:', DIM), 2, 0)
        self.l_bat = mk('—', CYAN, 12, True); g.addWidget(self.l_bat, 2, 1)
        g.addWidget(mk('Velocidad:', DIM), 3, 0)
        self.l_vel = mk('—', M_BLUE, 12, True); g.addWidget(self.l_vel, 3, 1)
        g.addWidget(mk('FPS:', DIM), 4, 0)
        self.l_fps = mk('—', CYAN, 9); g.addWidget(self.l_fps, 4, 1)
        return grp

    # ── Lane Status ──────────────────────────────────────────────────────────
    def _build_lane_status(self):
        grp = QGroupBox('Lane Detector')
        g = QGridLayout(grp); g.setSpacing(4)
        def mk(t, c=WHITE, s=10, b=False):
            l = QLabel(t); l.setStyleSheet(f'color:{c};font-size:{s}px;font-weight:{"bold" if b else "normal"};')
            return l
        g.addWidget(mk('Modo:', DIM), 0, 0)
        self.l_mode = mk('—', DIM, 12, True); g.addWidget(self.l_mode, 0, 1)
        g.addWidget(mk('Offset:', DIM), 1, 0)
        self.l_off = mk('—', YELLOW, 12, True); g.addWidget(self.l_off, 1, 1)
        g.addWidget(mk('Conf:', DIM), 2, 0)
        self.l_conf = mk('—', GREEN, 12, True); g.addWidget(self.l_conf, 2, 1)
        g.addWidget(mk('STOP:', DIM), 3, 0)
        self.l_stop = mk('OK', GREEN, 10, True); g.addWidget(self.l_stop, 3, 1)
        self.off_bar = QLabel(); self.off_bar.setFixedHeight(16)
        self.off_bar.setStyleSheet(f'background:{CARD};border:1px solid {BORDER};border-radius:3px;')
        g.addWidget(self.off_bar, 4, 0, 1, 2)
        return grp

    # ── Lidar Status ─────────────────────────────────────────────────────────
    def _build_lidar_status(self):
        grp = QGroupBox('LIDAR /qcar/scan')
        g = QGridLayout(grp); g.setSpacing(4)
        def mk(t, c=WHITE, s=10, b=False):
            l = QLabel(t); l.setStyleSheet(f'color:{c};font-size:{s}px;font-weight:{"bold" if b else "normal"};')
            return l
        g.addWidget(mk('Estado:', DIM), 0, 0)
        self.l_lstate = mk('—', DIM, 12, True); g.addWidget(self.l_lstate, 0, 1)
        g.addWidget(mk('Dist min:', DIM), 1, 0)
        self.l_lmin = mk('—', CYAN, 12, True); g.addWidget(self.l_lmin, 1, 1)
        g.addWidget(mk('Angulo:', DIM), 2, 0)
        self.l_lang = mk('—', M_YEL, 12, True); g.addWidget(self.l_lang, 2, 1)
        g.addWidget(mk('Obstaculo:', DIM), 3, 0)
        self.l_lobs = mk('CLEAR', GREEN, 10, True); g.addWidget(self.l_lobs, 3, 1)
        # ── Camara de profundidad ("ZED") ──
        g.addWidget(mk('ZED obst:', DIM), 4, 0)
        self.l_zobs = mk('CLEAR', GREEN, 10, True); g.addWidget(self.l_zobs, 4, 1)
        g.addWidget(mk('ZED dist:', DIM), 5, 0)
        self.l_zdist = mk('—', CYAN, 12, True); g.addWidget(self.l_zdist, 5, 1)
        self.lbar = QLabel(); self.lbar.setFixedHeight(16)
        self.lbar.setStyleSheet(f'background:{CARD};border:1px solid {BORDER};border-radius:3px;')
        g.addWidget(self.lbar, 6, 0, 1, 2)
        return grp

    # ── Telemetry Plots ──────────────────────────────────────────────────────
    def _build_plots(self):
        grp = QGroupBox('Telemetria')
        lay = QVBoxLayout(grp); lay.setSpacing(2)
        pw = pg.GraphicsLayoutWidget()
        pw.setBackground(PLOT_BG)
        lay.addWidget(pw)

        def mk_plot(row, title, ylabel, color):
            p = pw.addPlot(row=row, col=0, title=title)
            p.setLabel('left', ylabel, color=DIM, size='7pt')
            p.showGrid(x=True, y=True, alpha=0.3)
            p.getAxis('left').setPen(color, width=2)
            p.getAxis('bottom').setPen(BORDER)
            return p

        self.p_vel = mk_plot(0, 'Velocidad', 'm/s', M_BLUE)
        self.c_vel = self.p_vel.plot(pen=pg.mkPen(M_BLUE, width=2))

        self.p_off = mk_plot(1, 'Offset', 'off', M_YEL)
        self.c_off = self.p_off.plot(pen=pg.mkPen(M_YEL, width=2))
        self.p_off.setYRange(-1.1, 1.1)
        self.p_off.addLine(y=0, pen=pg.mkPen(BORDER, width=1, style=Qt.DashLine))

        self.p_conf = mk_plot(2, 'Confianza', 'conf', GREEN)
        self.c_conf = self.p_conf.plot(pen=pg.mkPen(GREEN, width=2))
        self.p_conf.setYRange(0, 1.1)

        self.p_lid = mk_plot(3, 'LIDAR Min Dist', 'm', M_RED)
        self.c_lid = self.p_lid.plot(pen=pg.mkPen(M_RED, width=2))
        self.p_lid.setYRange(0, LIDAR_MAX_R + 0.2)
        self.p_lid.addLine(y=OBS_TH, pen=pg.mkPen(RED, width=1, style=Qt.DashLine))
        self.p_lid.addLine(y=WARN_TH, pen=pg.mkPen(M_YEL, width=1, style=Qt.DashLine))

        return grp

    # ── Camera / visual box ──────────────────────────────────────────────────
    def _cam_box(self, title, key):
        grp = QGroupBox(title)
        lay = QVBoxLayout(grp); lay.setContentsMargins(2, 14, 2, 2)
        lbl = QLabel('NO SIGNAL')
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet('background:#050D16;color:#2A4060;font-size:14px;font-weight:bold;')
        lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lbl.setMinimumHeight(200 if key == 'lane' else 180)
        lay.addWidget(lbl)
        if key == 'cam':     self.v_cam = lbl
        elif key == 'lane':  self.v_lane = lbl
        elif key == 'yellow': self.v_yellow = lbl
        elif key == 'zed':   self.v_zed = lbl
        else:                self.v_lidar = lbl
        return grp

    # ── Render helper ────────────────────────────────────────────────────────
    @staticmethod
    def _to_pixmap(bgr, tw, th):
        if tw > 0 and th > 0:
            bh, bw = bgr.shape[:2]
            scale = min(tw / bw, th / bh)
            nw, nh = max(1, int(bw * scale)), max(1, int(bh * scale))
            if nw != bw or nh != bh:
                # INTER_NEAREST es extremadamente rapido y ahorra CPU en la Jetson
                bgr = cv2.resize(bgr, (nw, nh), interpolation=cv2.INTER_NEAREST)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        rgb = np.ascontiguousarray(rgb)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()
        return QPixmap.fromImage(qimg)

    # ── Update RAPIDO (30 FPS): SOLO video ───────────────────────────────────
    def _update_video(self):
        """Solo procesa imagenes — corre a 30 FPS para fluidez visual."""
        n = self.node
        with n.lock:
            raw_cam, n.raw_cam       = n.raw_cam,    None
            raw_lane, n.raw_lane     = n.raw_lane,   None
            raw_lidar, n.raw_lidar   = n.raw_lidar,  None
            raw_yellow, n.raw_yellow = n.raw_yellow, None
            raw_zed, n.raw_zed       = n.raw_zed,    None

        # Decode JPEG solo para los frames nuevos, fuera del lock
        def _decode(raw):
            if raw is None: return None
            buf = np.frombuffer(bytes(raw), np.uint8)
            return cv2.imdecode(buf, cv2.IMREAD_COLOR)

        if raw_cam    is not None: n.frame_cam    = _decode(raw_cam)
        if raw_lane   is not None: n.frame_lane   = _decode(raw_lane)
        if raw_lidar  is not None: n.frame_lidar  = _decode(raw_lidar)
        if raw_yellow is not None: n.frame_yellow = _decode(raw_yellow)
        if raw_zed    is not None: n.frame_zed    = _decode(raw_zed)
        cam, lane, lidar, yellow = n.frame_cam, n.frame_lane, n.frame_lidar, n.frame_yellow
        zed = n.frame_zed

        # Solo setPixmap si hay frame fresco — evita repintar lo mismo
        if cam    is not None and raw_cam    is not None:
            self.v_cam.setPixmap(self._to_pixmap(cam,       self.v_cam.width(),    self.v_cam.height()))
        if lane   is not None and raw_lane   is not None:
            self.v_lane.setPixmap(self._to_pixmap(lane,     self.v_lane.width(),   self.v_lane.height()))
        if lidar  is not None and raw_lidar  is not None:
            self.v_lidar.setPixmap(self._to_pixmap(lidar,   self.v_lidar.width(),  self.v_lidar.height()))
        if yellow is not None and raw_yellow is not None:
            self.v_yellow.setPixmap(self._to_pixmap(yellow, self.v_yellow.width(), self.v_yellow.height()))
        # Manejar el estado deshabilitado del panel ZED
        show_zed = bool(n.get_parameter('show_zed_image').value)
        if not show_zed:
            self.v_zed.setText("DISABLED (show_zed_image=False)")
            self.v_zed.setStyleSheet('background:#050D16;color:#556688;font-size:11px;font-weight:bold;')
        elif raw_zed is None and n.frame_zed is None:
            self.v_zed.setText("NO SIGNAL")
            self.v_zed.setStyleSheet('background:#050D16;color:#2A4060;font-size:14px;font-weight:bold;')
        elif zed is not None and raw_zed is not None:
            self.v_zed.setPixmap(self._to_pixmap(zed,       self.v_zed.width(),    self.v_zed.height()))

    # ── Update LENTO (5 FPS): plots + labels + stats ─────────────────────────
    def _update_data(self):
        """Plots, labels y stats — refrescan a 5 FPS, no necesitan video rate."""
        n = self.node
        n._check_zed_subscription()
        with n.lock:
            fc, fl, fr = n.fps_cam, n.fps_lane, n.fps_lidar
            t  = list(n.h_time)
            hv = list(n.h_vel)
            ho = list(n.h_offset)
            hc = list(n.h_conf)
            hl = list(n.h_lidar)

        # Plots
        if t:
            te = t[-1]
            xmin, xmax = max(0, te - 15), te + 0.5
            self.c_vel.setData(t, hv)
            self.c_off.setData(t, ho)
            self.c_conf.setData(t, hc)
            self.c_lid.setData(t, hl)
            if not hasattr(self, '_last_xmax') or abs(xmax - self._last_xmax) > 0.5:
                for p in (self.p_vel, self.p_off, self.p_conf, self.p_lid):
                    p.setXRange(xmin, xmax, padding=0)
                self._last_xmax = xmax

        # ── System Status ────────────────────────────────────────────────────
        now = n.get_clock().now().nanoseconds / 1e9
        v = math.hypot(n.vel_x, n.vel_y)
        
        # Pilot state
        if n.auto_active:
            self.l_pilot.setText('AUTONOMO')
            self.l_pilot.setStyleSheet(f'color:{CYAN};font-size:12px;font-weight:bold;')
        else:
            self.l_pilot.setText('MANUAL')
            self.l_pilot.setStyleSheet(f'color:{DIM};font-size:12px;font-weight:bold;')

        bat_age = now - n.battery_t if n.battery_t > 0 else 99.0
        if bat_age < 3.0:
            bv = n.battery_v
            bc = GREEN if bv > 11.5 else (YELLOW if bv > 10.8 else RED)
            self.l_bat.setText(f'{bv:.2f} V')
            self.l_bat.setStyleSheet(f'color:{bc};font-size:12px;font-weight:bold;')
        else:
            self.l_bat.setText('— V'); self.l_bat.setStyleSheet(f'color:{DIM};font-size:12px;font-weight:bold;')
        self.l_vel.setText(f'{v:.3f} m/s')
        self.l_vel.setStyleSheet(f'color:{M_BLUE};font-size:12px;font-weight:bold;')
        self.l_fps.setText(f'CAM:{fc:.0f}  LANE:{fl:.0f}  LIDAR:{fr:.0f}')
        ros_ok = (now - max(n._t_cam, n._t_lane, n._t_lidar, n.battery_t)) < 3.0
        self.l_ros.setText('OK' if ros_ok else 'SIN SENAL')
        self.l_ros.setStyleSheet(f'color:{GREEN if ros_ok else RED};font-size:11px;font-weight:bold;')

        # ── Lane Status ──────────────────────────────────────────────────────
        off, conf = n.offset, n.conf
        age = now - n.offset_t if n.offset_t > 0 else 99.0
        if age > 1.5:      mt, mc = 'SIN SENAL', RED
        elif conf >= 0.7:  mt, mc = 'DETECTADO', GREEN
        elif conf >= 0.25: mt, mc = 'PARCIAL', YELLOW
        else:              mt, mc = 'CIEGO', RED
        self.l_mode.setText(mt); self.l_mode.setStyleSheet(f'color:{mc};font-size:12px;font-weight:bold;')
        oc = YELLOW if abs(off) < 0.3 else (YELLOW if abs(off) < 0.6 else RED)
        self.l_off.setText(f'{off:+.3f}'); self.l_off.setStyleSheet(f'color:{oc};font-size:12px;font-weight:bold;')
        self.l_conf.setText(f'{conf:.3f}'); self.l_conf.setStyleSheet(f'color:{mc};font-size:12px;font-weight:bold;')
        if n.stop_flag:
            self.l_stop.setText('STOP!'); self.l_stop.setStyleSheet(f'color:{RED};font-size:10px;font-weight:bold;')
        else:
            self.l_stop.setText('OK'); self.l_stop.setStyleSheet(f'color:{GREEN};font-size:10px;font-weight:bold;')
        bw = max(self.off_bar.width(), 1)
        pos = int((float(np.clip(off, -1, 1)) + 1) * 0.5 * bw)
        pos = max(6, min(pos, bw - 6))
        bc = GREEN if abs(off) < 0.3 else (YELLOW if abs(off) < 0.6 else RED)
        self.off_bar.setStyleSheet(
            f'background:qlineargradient(x1:0,x2:1,'
            f'stop:0 {CARD},stop:{max(0,(pos-4)/bw):.3f} {CARD},'
            f'stop:{pos/bw:.3f} {bc},'
            f'stop:{min(1,(pos+4)/bw):.3f} {CARD},stop:1 {CARD});'
            f'border:1px solid {BORDER};border-radius:3px;')

        # ── Lidar Status ─────────────────────────────────────────────────────
        lage = now - n.lidar_t if n.lidar_t > 0 else 99.0
        d = n.lidar_min
        ang_deg = math.degrees(n.lidar_ang)
        has_data = (d is not None) and (d >= 0.0)
        if lage > 1.5:        lt, lc = 'SIN SENAL', RED
        elif not has_data:    lt, lc = 'FUERA DE RANGO', YELLOW
        elif n.lidar_obs:     lt, lc = 'OBSTACULO', RED
        elif d < WARN_TH:     lt, lc = 'CERCA', YELLOW
        else:                 lt, lc = 'LIBRE', GREEN
        self.l_lstate.setText(lt); self.l_lstate.setStyleSheet(f'color:{lc};font-size:12px;font-weight:bold;')
        dc = RED if (has_data and d < OBS_TH) else (YELLOW if (has_data and d < WARN_TH) else CYAN)
        self.l_lmin.setText(f'{d:.2f} m' if has_data else '---')
        self.l_lmin.setStyleSheet(f'color:{dc};font-size:12px;font-weight:bold;')
        self.l_lang.setText(f'{ang_deg:+.1f}°' if has_data else '---')
        self.l_lang.setStyleSheet(f'color:{M_YEL};font-size:12px;font-weight:bold;')
        if n.lidar_obs:
            self.l_lobs.setText('¡OBSTACULO!'); self.l_lobs.setStyleSheet(f'color:{RED};font-size:10px;font-weight:bold;')
        else:
            self.l_lobs.setText('CLEAR'); self.l_lobs.setStyleSheet(f'color:{GREEN};font-size:10px;font-weight:bold;')

        # ── ZED (camara de profundidad) ──
        zage = now - n.zed_t if n.zed_t > 0 else 99.0
        zd = n.zed_dist
        z_has = (zd is not None) and (zd >= 0.0)
        if zage > 1.5:
            self.l_zobs.setText('SIN SENAL'); self.l_zobs.setStyleSheet(f'color:{DIM};font-size:10px;font-weight:bold;')
        elif n.zed_obs:
            self.l_zobs.setText('¡OBSTACULO!'); self.l_zobs.setStyleSheet(f'color:{RED};font-size:10px;font-weight:bold;')
        else:
            self.l_zobs.setText('CLEAR'); self.l_zobs.setStyleSheet(f'color:{GREEN};font-size:10px;font-weight:bold;')
        zc = RED if (z_has and zd < OBS_TH) else (YELLOW if (z_has and zd < WARN_TH) else CYAN)
        self.l_zdist.setText(f'{zd:.2f} m' if (z_has and zage <= 1.5) else '---')
        self.l_zdist.setStyleSheet(f'color:{zc};font-size:12px;font-weight:bold;')

        # Barra de proximidad: llena = lejos (seguro)
        lbw = max(self.lbar.width(), 1)
        frac = 0.0 if not has_data else float(np.clip(d / LIDAR_MAX_R, 0.0, 1.0))
        bc2 = RED if (has_data and d < OBS_TH) else (YELLOW if (has_data and d < WARN_TH) else GREEN)
        stop_px = max(1, min(int(frac * lbw), lbw - 1)) / lbw
        self.lbar.setStyleSheet(
            f'background:qlineargradient(x1:0,x2:1,'
            f'stop:0 {bc2},stop:{stop_px:.3f} {bc2},'
            f'stop:{min(1, stop_px + 0.001):.3f} {CARD},stop:1 {CARD});'
            f'border:1px solid {BORDER};border-radius:3px;')


def main():
    rclpy.init()
    node = ROSBackend()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    app = QApplication(sys.argv)
    win = Dashboard(node)
    win.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
