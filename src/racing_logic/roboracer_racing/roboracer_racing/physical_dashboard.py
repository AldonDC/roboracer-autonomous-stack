"""
Physical QCar Dashboard v3 — Qt + PyQtGraph (30 FPS)
=====================================================
- Camara frontal cruda (CompressedImage JPEG, resolución reducida)
- Lane detector debug (Image, resolución reducida)
- Graficas: Velocidad, Offset, Confianza (estilo MATLAB)
- Sliders HSV para calibrar amarillo en vivo
- Estado completo del lane detector
"""
import sys, threading, math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSDurabilityPolicy, QoSHistoryPolicy)
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from collections import deque

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                               QHBoxLayout, QLabel, QGroupBox, QSlider,
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

# Resolucion reducida para fluidez
CAM_W, CAM_H = 320, 240


def _compressed_to_bgr(msg):
    try:
        return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
    except Exception:
        return None

def _imgmsg_to_bgr(msg):
    enc = msg.encoding.lower()
    try:
        buf = np.frombuffer(bytes(msg.data), np.uint8)
        if enc == 'bgr8':
            return buf.reshape(msg.height, msg.width, 3).copy()
        if enc == 'rgb8':
            return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2BGR)
    except Exception:
        pass
    return None


# ── ROS Backend ───────────────────────────────────────────────────────────────
class ROSBackend(Node):
    def __init__(self):
        super().__init__('physical_dashboard')
        _qcar = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                           history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        _be = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, depth=2)

        self.create_subscription(CompressedImage, '/qcar/csi_front',
                                 self._comp_cb, _qcar)
        self.create_subscription(CompressedImage, '/lane/image_debug',
                                 self._dbg_cb, 10)
        self.create_subscription(Float32, '/lane/center_offset', self._off_cb, 10)
        self.create_subscription(Float32, '/lane/confidence', self._conf_cb, 10)
        self.create_subscription(Bool, '/lane/stop_sign', self._stop_cb, 10)
        self.create_subscription(Vector3Stamped, '/qcar/velocity',
                                 self._vel_cb, _qcar)

        self.lock = threading.Lock()
        self.frame_raw = None
        self.frame_dbg = None
        self.fps_raw = 0.0;  self.fps_dbg = 0.0
        self._t_raw = 0.0;   self._t_dbg = 0.0
        self.offset = 0.0;   self.conf = 0.0
        self.stop_flag = False;  self.offset_t = 0.0
        self.vel_x = 0.0;    self.vel_y = 0.0

        # Historiales para graficas
        N = 300
        self.t_start = None
        self.h_time   = deque(maxlen=N)
        self.h_vel    = deque(maxlen=N)
        self.h_offset = deque(maxlen=N)
        self.h_conf   = deque(maxlen=N)

        self.get_logger().info('Dashboard v3 — Qt+PyQtGraph — QCar fisico')

    def _comp_cb(self, msg):
        bgr = _compressed_to_bgr(msg)
        if bgr is None: return
        bgr = cv2.resize(bgr, (CAM_W, CAM_H), interpolation=cv2.INTER_NEAREST)
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.frame_raw = bgr
            self.fps_raw = 1.0 / max(now - self._t_raw, 0.001)
            self._t_raw = now

    def _dbg_cb(self, msg):
        bgr = _compressed_to_bgr(msg)
        if bgr is None: return
        now = self.get_clock().now().nanoseconds / 1e9
        with self.lock:
            self.frame_dbg = bgr
            self.fps_dbg = 1.0 / max(now - self._t_dbg, 0.001)
            self._t_dbg = now

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
        # Guardar en historial
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

    def push_hsv(self, name, val):
        cli = self.create_client(SetParameters, '/lane_detector/set_parameters')
        if not cli.service_is_ready(): return
        req = SetParameters.Request()
        p = Parameter(); p.name = name
        p.value.type = ParameterType.PARAMETER_INTEGER
        p.value.integer_value = int(val)
        req.parameters = [p]
        cli.call_async(req)


# ── Dashboard GUI ─────────────────────────────────────────────────────────────
class Dashboard(QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('QCar Physical Dashboard v3')
        self.setMinimumSize(1400, 800)
        self._style()

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setSpacing(6)
        root.setContentsMargins(6, 6, 6, 6)

        # ── Left: Status + HSV + Graphs ───────────────────────────────────────
        left = QVBoxLayout()
        left.setSpacing(4)
        root.addLayout(left, 4)

        left.addWidget(self._build_status())
        left.addWidget(self._build_hsv())
        left.addWidget(self._build_plots(), 1)

        # ── Right: Cameras ────────────────────────────────────────────────────
        right = QVBoxLayout()
        right.setSpacing(4)
        root.addLayout(right, 6)

        right.addWidget(self._cam_box('CAMARA FRONTAL — /qcar/csi_front', 'raw'), 1)
        right.addWidget(self._cam_box('LANE DEBUG — /lane/image_debug', 'dbg'), 1)

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
            QSlider::groove:horizontal {{ height:4px; background:{BORDER}; border-radius:2px; }}
            QSlider::handle:horizontal {{ background:{CYAN}; width:10px; height:10px; border-radius:5px; margin:-3px 0; }}
            QSlider::sub-page:horizontal {{ background:{CYAN}; border-radius:2px; }}
        """)

    # ── Status ────────────────────────────────────────────────────────────────
    def _build_status(self):
        grp = QGroupBox('Lane Detector')
        g = QGridLayout(grp); g.setSpacing(4)
        def mk(t, c=WHITE, s=10, b=False):
            l = QLabel(t); l.setStyleSheet(f'color:{c};font-size:{s}px;font-weight:{"bold" if b else "normal"};')
            return l
        g.addWidget(mk('Modo:', DIM), 0, 0)
        self.l_mode = mk('—', DIM, 13, True); g.addWidget(self.l_mode, 0, 1)
        g.addWidget(mk('Offset:', DIM), 1, 0)
        self.l_off = mk('—', YELLOW, 13, True); g.addWidget(self.l_off, 1, 1)
        g.addWidget(mk('Conf:', DIM), 2, 0)
        self.l_conf = mk('—', GREEN, 13, True); g.addWidget(self.l_conf, 2, 1)
        g.addWidget(mk('Vel:', DIM), 3, 0)
        self.l_vel = mk('—', M_BLUE, 13, True); g.addWidget(self.l_vel, 3, 1)
        g.addWidget(mk('STOP:', DIM), 4, 0)
        self.l_stop = mk('NORMAL', GREEN, 10, True); g.addWidget(self.l_stop, 4, 1)
        g.addWidget(mk('FPS:', DIM), 5, 0)
        self.l_fps = mk('—', CYAN, 9); g.addWidget(self.l_fps, 5, 1)
        # Offset bar
        self.off_bar = QLabel(); self.off_bar.setFixedHeight(20)
        self.off_bar.setStyleSheet(f'background:{CARD};border:1px solid {BORDER};border-radius:3px;')
        g.addWidget(self.off_bar, 6, 0, 1, 2)
        return grp

    # ── HSV ───────────────────────────────────────────────────────────────────
    def _build_hsv(self):
        grp = QGroupBox('HSV Amarillo')
        g = QGridLayout(grp); g.setSpacing(3)
        self._hsv = {}
        for r, (lb, k, lo, hi, d) in enumerate([
            ('H min','h_min',0,179,18), ('H max','h_max',0,179,38),
            ('S min','s_min',0,255,80),  ('S max','s_max',0,255,255),
            ('V min','v_min',0,255,80),  ('V max','v_max',0,255,255)]):
            l = QLabel(lb); l.setStyleSheet(f'color:{DIM};font-size:8px;')
            g.addWidget(l, r, 0)
            sl = QSlider(Qt.Horizontal); sl.setRange(lo, hi); sl.setValue(d)
            g.addWidget(sl, r, 1)
            vl = QLabel(str(d)); vl.setStyleSheet(f'color:{YELLOW};font-size:8px;min-width:24px;')
            vl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            g.addWidget(vl, r, 2)
            sl.valueChanged.connect(lambda v, _l=vl, _k=k: (_l.setText(str(v)), self.node.push_hsv(_k, v)))
            self._hsv[k] = sl
        self.hsv_prev = QLabel(); self.hsv_prev.setFixedHeight(18)
        self.hsv_prev.setStyleSheet('background:#C8A000;border-radius:2px;font-size:8px;')
        g.addWidget(self.hsv_prev, len(self._hsv), 0, 1, 3)
        return grp

    # ── Plots (PyQtGraph) ─────────────────────────────────────────────────────
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

        return grp

    # ── Camera box ────────────────────────────────────────────────────────────
    def _cam_box(self, title, key):
        grp = QGroupBox(title)
        lay = QVBoxLayout(grp); lay.setContentsMargins(2, 14, 2, 2)
        lbl = QLabel('NO SIGNAL')
        lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet('background:#050D16;color:#2A4060;font-size:14px;font-weight:bold;')
        lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lbl.setMinimumHeight(180)
        lay.addWidget(lbl)
        if key == 'raw': self.cam_raw = lbl
        else:            self.cam_dbg = lbl
        return grp

    # ── Render helper ─────────────────────────────────────────────────────────
    @staticmethod
    def _to_pixmap(bgr, tw, th):
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        return QPixmap.fromImage(qimg).scaled(tw, th, Qt.KeepAspectRatio, Qt.FastTransformation)

    # ── Update 30 FPS ─────────────────────────────────────────────────────────
    def _update(self):
        n = self.node
        with n.lock:
            raw, dbg = n.frame_raw, n.frame_dbg
            fr, fd = n.fps_raw, n.fps_dbg
            t = list(n.h_time)
            hv = list(n.h_vel)
            ho = list(n.h_offset)
            hc = list(n.h_conf)

        # Cameras
        if raw is not None:
            self.cam_raw.setPixmap(self._to_pixmap(raw, self.cam_raw.width(), self.cam_raw.height()))
        if dbg is not None:
            self.cam_dbg.setPixmap(self._to_pixmap(dbg, self.cam_dbg.width(), self.cam_dbg.height()))

        # Plots
        if t:
            self.c_vel.setData(t, hv)
            self.c_off.setData(t, ho)
            self.c_conf.setData(t, hc)
            te = t[-1]
            for p in (self.p_vel, self.p_off, self.p_conf):
                p.setXRange(max(0, te - 15), te + 0.5, padding=0)

        # Status
        off, conf = n.offset, n.conf
        now = n.get_clock().now().nanoseconds / 1e9
        age = now - n.offset_t if n.offset_t > 0 else 99.0
        v = math.hypot(n.vel_x, n.vel_y)

        if age > 1.5:      mt, mc = 'SIN SENAL', RED
        elif conf >= 0.7:   mt, mc = 'DETECTADO', GREEN
        elif conf >= 0.25:  mt, mc = 'PARCIAL', YELLOW
        else:               mt, mc = 'CIEGO', RED

        self.l_mode.setText(mt); self.l_mode.setStyleSheet(f'color:{mc};font-size:13px;font-weight:bold;')
        oc = YELLOW if abs(off) < 0.3 else (YELLOW if abs(off) < 0.6 else RED)
        self.l_off.setText(f'{off:+.3f}'); self.l_off.setStyleSheet(f'color:{oc};font-size:13px;font-weight:bold;')
        self.l_conf.setText(f'{conf:.3f}'); self.l_conf.setStyleSheet(f'color:{mc};font-size:13px;font-weight:bold;')
        self.l_vel.setText(f'{v:.3f} m/s'); self.l_vel.setStyleSheet(f'color:{M_BLUE};font-size:13px;font-weight:bold;')
        self.l_fps.setText(f'RAW:{fr:.0f}  DBG:{fd:.0f}')

        if n.stop_flag:
            self.l_stop.setText('STOP!'); self.l_stop.setStyleSheet(f'color:{RED};font-size:10px;font-weight:bold;')
        else:
            self.l_stop.setText('OK'); self.l_stop.setStyleSheet(f'color:{GREEN};font-size:10px;font-weight:bold;')

        # Offset bar
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

        # HSV preview
        hm = (self._hsv['h_min'].value() + self._hsv['h_max'].value()) // 2
        sm = (self._hsv['s_min'].value() + self._hsv['s_max'].value()) // 2
        vm = (self._hsv['v_min'].value() + self._hsv['v_max'].value()) // 2
        px = np.array([[[hm, sm, vm]]], dtype=np.uint8)
        rgb = cv2.cvtColor(px, cv2.COLOR_HSV2RGB)[0, 0]
        self.hsv_prev.setStyleSheet(
            f'background:rgb({rgb[0]},{rgb[1]},{rgb[2]});border-radius:2px;'
            f'font-size:8px;padding:1px;color:{"#000" if vm > 128 else "#FFF"};')
        self.hsv_prev.setText(f' H:{hm} S:{sm} V:{vm}')


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
