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
        _qcar = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        _be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=2)

        # Imagenes
        self.create_subscription(CompressedImage, '/qcar/csi_front',
                                 self._cam_cb, _be)
        self.create_subscription(CompressedImage, '/lane/image_debug',
                                 self._lane_cb, _be)
        self.create_subscription(CompressedImage, '/lidar/image_debug',
                                 self._lidar_img_cb, _be)

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

        self.lock = threading.Lock()

        # Frames
        self.frame_cam = None;   self.frame_lane = None;   self.frame_lidar = None
        self.fps_cam = 0.0;      self.fps_lane = 0.0;      self.fps_lidar = 0.0
        self._t_cam = 0.0;       self._t_lane = 0.0;       self._t_lidar = 0.0

        # Lane state
        self.offset = 0.0;       self.conf = 0.0
        self.stop_flag = False;  self.offset_t = 0.0

        # Telemetria
        self.vel_x = 0.0;        self.vel_y = 0.0
        self.battery_v = 0.0;    self.battery_t = 0.0

        # Lidar metricas
        self.lidar_min = -1.0;   self.lidar_ang = 0.0
        self.lidar_obs = False;  self.lidar_t = 0.0

        # Historial graficas
        N = 300
        self.t_start = None
        self.h_time   = deque(maxlen=N)
        self.h_vel    = deque(maxlen=N)
        self.h_offset = deque(maxlen=N)
        self.h_conf   = deque(maxlen=N)
        self.h_lidar  = deque(maxlen=N)

        self.get_logger().info('Dashboard v4 — Layout 2 columnas — QCar fisico')

    # ── Callbacks de imagenes ────────────────────────────────────────────────
    def _cam_cb(self, msg):
        bgr = _compressed_to_bgr(msg)
        if bgr is None: return
        bgr = cv2.resize(bgr, (CAM_W, CAM_H), interpolation=cv2.INTER_NEAREST)
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.frame_cam = bgr
            self.fps_cam = 1.0 / max(now - self._t_cam, 0.001)
            self._t_cam = now

    def _lane_cb(self, msg):
        bgr = _compressed_to_bgr(msg)
        if bgr is None: return
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.frame_lane = bgr
            self.fps_lane = 1.0 / max(now - self._t_lane, 0.001)
            self._t_lane = now

    def _lidar_img_cb(self, msg):
        bgr = _compressed_to_bgr(msg)
        if bgr is None: return
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.frame_lidar = bgr
            self.fps_lidar = 1.0 / max(now - self._t_lidar, 0.001)
            self._t_lidar = now

    # ── Callbacks de datos ───────────────────────────────────────────────────
    def _off_cb(self, msg):
        self.offset = float(msg.data)
        self.offset_t = self.get_clock().now().nanoseconds / 1e9

    def _conf_cb(self, msg):
        self.conf = float(msg.data)

    def _stop_cb(self, msg):
        self.stop_flag = bool(msg.data)

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
        rv.addWidget(self._cam_box('🎯 LIDAR RADAR — /lidar/image_debug', 'lidar'), 5)
        rv.addWidget(self._cam_box('CAMARA FRONTAL — /qcar/csi_front', 'cam'), 3)
        rv.addWidget(self._cam_box('LANE DEBUG — /lane/image_debug', 'lane'), 3)
        splitter.addWidget(right)

        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 7)
        splitter.setSizes([420, 1080])

        # Timer 30 FPS
        self._timer = QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(33)

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
        g.addWidget(mk('Bateria:', DIM), 1, 0)
        self.l_bat = mk('—', CYAN, 12, True); g.addWidget(self.l_bat, 1, 1)
        g.addWidget(mk('Velocidad:', DIM), 2, 0)
        self.l_vel = mk('—', M_BLUE, 12, True); g.addWidget(self.l_vel, 2, 1)
        g.addWidget(mk('FPS:', DIM), 3, 0)
        self.l_fps = mk('—', CYAN, 9); g.addWidget(self.l_fps, 3, 1)
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
        self.lbar = QLabel(); self.lbar.setFixedHeight(16)
        self.lbar.setStyleSheet(f'background:{CARD};border:1px solid {BORDER};border-radius:3px;')
        g.addWidget(self.lbar, 4, 0, 1, 2)
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
        lbl.setMinimumHeight(180)
        lay.addWidget(lbl)
        if key == 'cam':     self.v_cam = lbl
        elif key == 'lane':  self.v_lane = lbl
        else:                self.v_lidar = lbl
        return grp

    # ── Render helper ────────────────────────────────────────────────────────
    @staticmethod
    def _to_pixmap(bgr, tw, th):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        return QPixmap.fromImage(qimg).scaled(tw, th, Qt.KeepAspectRatio, Qt.FastTransformation)

    # ── Update 30 FPS ────────────────────────────────────────────────────────
    def _update(self):
        n = self.node
        with n.lock:
            cam, lane, lidar = n.frame_cam, n.frame_lane, n.frame_lidar
            fc, fl, fr = n.fps_cam, n.fps_lane, n.fps_lidar
            t  = list(n.h_time)
            hv = list(n.h_vel)
            ho = list(n.h_offset)
            hc = list(n.h_conf)
            hl = list(n.h_lidar)

        # Visual panels
        if cam   is not None: self.v_cam.setPixmap(self._to_pixmap(cam,   self.v_cam.width(),   self.v_cam.height()))
        if lane  is not None: self.v_lane.setPixmap(self._to_pixmap(lane, self.v_lane.width(),  self.v_lane.height()))
        if lidar is not None: self.v_lidar.setPixmap(self._to_pixmap(lidar,self.v_lidar.width(),self.v_lidar.height()))

        # Plots
        if t:
            self.c_vel.setData(t, hv)
            self.c_off.setData(t, ho)
            self.c_conf.setData(t, hc)
            self.c_lid.setData(t, hl)
            te = t[-1]
            for p in (self.p_vel, self.p_off, self.p_conf, self.p_lid):
                p.setXRange(max(0, te - 15), te + 0.5, padding=0)

        # ── System Status ────────────────────────────────────────────────────
        now = n.get_clock().now().nanoseconds / 1e9
        v = math.hypot(n.vel_x, n.vel_y)
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
