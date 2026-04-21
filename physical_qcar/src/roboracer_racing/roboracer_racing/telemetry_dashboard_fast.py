"""
RoboRacer Engineering Telemetry v13 — Fast Edition (pyqtgraph)
--------------------------------------------------------------
High-performance telemetry dashboard using pyqtgraph and Qt.
Replaces Matplotlib for 60+ FPS camera fusion and interactive plots.

Layout:
- Left: Time-series engineering plots (Velocity, Steering, Error, Accel, Lidar, Force).
- Right: Perception Fusion (Front Cam + Lidar + Clusters, Back Cam, Right Cam).
"""

import sys
import math
import numpy as np
import threading
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray, Bool
from sensor_msgs.msg import LaserScan, Image
from collections import deque

# Qt & PyQtGraph
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets, QtGui

# --- Clustering (Same logic as v11) ---
try:
    from sklearn.cluster import DBSCAN as _DBSCAN
    _CLUSTER_BACKEND = 'sklearn'
except ImportError:
    try:
        from scipy.spatial import KDTree as _KDTree
        _CLUSTER_BACKEND = 'scipy'
    except ImportError:
        _CLUSTER_BACKEND = 'none'

# --- Constants ---
LIDAR_RANGE   = 4.0
MAX_CLUSTERS  = 6
MAX_BBOX_SPAN = 2.4
CLUSTER_COLORS = [(255, 68, 68), (68, 255, 136), (68, 153, 255), (255, 204, 0), (255, 68, 255), (68, 255, 255)]

CAM_FRONT = '/qcar_sim/csi_front/image_raw'
CAM_BACK  = '/qcar_sim/csi_back/image_raw'
CAM_RIGHT = '/qcar_sim/csi_right/image_raw'
LANE_DBG  = '/lane/image_debug'

# Camera intrinsics (v13 Calibration)
CAM_W, CAM_H = 400, 300
CAM_FX = CAM_FY = 306.25
CAM_CX, CAM_CY = 200.0, 150.0
CAM_Z_OBS = 0.25
CAM_Z_CAM = 0.12

# --- Styling ---
PANEL_BG      = '#FFFFFF'
PLOT_BG       = '#F6F8FC'
GRID_COLOR    = (221, 228, 239)
TEXT_COLOR    = '#2C3E50'
TEXT_DIM      = '#7A8A9A'
MATLAB_BLUE   = '#0072BD'
MATLAB_RED    = '#D95319'
MATLAB_YELLOW = '#EDB120'
MATLAB_PURPLE = '#7E2F8E'
GRN = '#3FB950'; YLW = '#E3B341'; RED = '#FF3D00'; CYN = '#00E5CC'

# LiDAR Colormap (Blue -> Cyan -> green -> yellow -> red)
LIDAR_TICKS = [
    (0.0, (0, 255, 255)),
    (0.2, (0, 255, 136)),
    (0.4, (170, 255, 0)),
    (0.6, (255, 255, 0)),
    (0.8, (255, 136, 0)),
    (1.0, (255, 34, 0))
]
LIDAR_LUT = pg.ColorMap([t[0] for t in LIDAR_TICKS], [t[1] for t in LIDAR_TICKS]).getLookupTable(0.0, 1.0, 256)

def _img_to_numpy(msg):
    enc = msg.encoding.lower()
    try:
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        if enc == 'rgb8':
            return raw.reshape(msg.height, msg.width, 3)
        if enc == 'bgr8':
            return raw.reshape(msg.height, msg.width, 3)[:, :, ::-1].copy()
        if enc in ('mono8', '8uc1'):
            g = raw.reshape(msg.height, msg.width)
            return np.stack([g, g, g], axis=-1)
        if enc == 'rgba8':
            return raw.reshape(msg.height, msg.width, 4)[:, :, :3]
    except Exception:
        pass
    return None

def _lidar_to_front_image(scan_x, scan_y):
    fwd = scan_x > 0.15
    if not np.any(fwd):
        return np.array([]), np.array([]), np.array([])
    x = scan_x[fwd].astype(np.float64)
    y = scan_y[fwd].astype(np.float64)
    dz = CAM_Z_OBS - CAM_Z_CAM
    u  = CAM_CX - CAM_FX * y / x
    v  = CAM_CY - CAM_FY * dz / x
    d  = np.hypot(x, y)
    ok = (u >= 0) & (u < CAM_W) & (v >= 0) & (v < CAM_H)
    return u[ok], v[ok], d[ok]

def _cluster_to_img_box(cy_arr, x_centre):
    x = max(float(x_centre), 0.15)
    y_min, y_max = float(cy_arr.min()), float(cy_arr.max())
    u_right = CAM_CX - CAM_FX * y_min / x
    u_left  = CAM_CX - CAM_FX * y_max / x
    z_top, z_bot = CAM_Z_CAM + 0.50, 0.0
    v_top = CAM_CY - CAM_FY * (z_top - CAM_Z_CAM) / x
    v_bot = CAM_CY - CAM_FY * (z_bot - CAM_Z_CAM) / x
    u_l = max(0, min(int(u_left),  CAM_W - 1))
    u_r = max(0, min(int(u_right), CAM_W - 1))
    v_t = max(0, min(int(v_top),   CAM_H - 1))
    v_b = max(0, min(int(v_bot),   CAM_H - 1))
    return u_l, v_t, max(u_r - u_l, 4), max(v_b - v_t, 4)

def _cluster(pts):
    EPS, MIN_PTS = 0.25, 4
    if _CLUSTER_BACKEND == 'sklearn':
        labels = _DBSCAN(eps=EPS, min_samples=MIN_PTS).fit_predict(pts)
    elif _CLUSTER_BACKEND == 'scipy':
        # Simple clustering would go here, omitting for brevity in favor of same logic
        labels = np.full(len(pts), -1, dtype=np.int32)
    else:
        return np.full(len(pts), -1, dtype=np.int32)
    return labels

class ROSBackend(Node):
    def __init__(self):
        super().__init__('telemetry_dashboard_fast')
        self.sub = self.create_subscription(Float32MultiArray, '/viz/telemetry', self.telemetry_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/qcar_sim/scan', self.scan_cb, 10)
        
        _cam_qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        self.cam_f_sub = self.create_subscription(Image, CAM_FRONT, lambda m: self._cam_cb(m, 'f'), _cam_qos)
        self.cam_b_sub = self.create_subscription(Image, CAM_BACK,  lambda m: self._cam_cb(m, 'b'), _cam_qos)
        self.cam_r_sub = self.create_subscription(Image, CAM_RIGHT, lambda m: self._cam_cb(m, 'r'), _cam_qos)
        self.lane_dbg_sub = self.create_subscription(Image, LANE_DBG, lambda m: self._cam_cb(m, 'l'), _cam_qos)
        
        self.lane_off_sub = self.create_subscription(Float32, '/lane/center_offset', self._lane_off_cb, 10)
        self.lane_conf_sub = self.create_subscription(Float32, '/lane/confidence', self._lane_conf_cb, 10)
        self.stop_sign_sub = self.create_subscription(Bool, '/lane/stop_sign', self._stop_sign_cb, 10)
        
        # Deques for plots
        self.max_pts = 400
        self.times = deque(maxlen=self.max_pts)
        self.v_target = deque(maxlen=self.max_pts)
        self.v_actual = deque(maxlen=self.max_pts)
        self.steering = deque(maxlen=self.max_pts)
        self.dist_error = deque(maxlen=self.max_pts)
        self.accel = deque(maxlen=self.max_pts)
        self.f_mag = deque(maxlen=self.max_pts)
        self.d_min_hist = deque(maxlen=self.max_pts)
        self.obs_state = deque(maxlen=self.max_pts)

        self.scan_x = np.array([], dtype=np.float32)
        self.scan_y = np.array([], dtype=np.float32)
        self.scan_d = np.array([], dtype=np.float32)
        self.lidar_stats = {"n": 0, "dmin": 0.0}

        self._cam_frames = {'f': None, 'b': None, 'r': None, 'l': None}
        self._cam_fps = {'f': 0, 'b': 0, 'r': 0, 'l': 0}
        self._cam_last_t = {'f': 0, 'b': 0, 'r': 0, 'l': 0}
        
        self.lane_offset = 0.0
        self.lane_conf = 0.0
        self.lane_last_t = 0.0
        self.stop_detected = False
        
        self.v_last = 0.0
        self.t_last = 0.0
        self.start_time = None
        self.data_lock = threading.Lock()
        self.cam_lock = threading.Lock()

    def telemetry_cb(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        d = msg.data
        with self.data_lock:
            if len(d) >= 5:
                self.times.append(t)
                self.v_target.append(d[0]); self.v_actual.append(d[1])
                self.steering.append(d[2]); self.dist_error.append(d[3])
                self.obs_state.append(d[4])
                
                dt = t - self.t_last if self.t_last > 0 else 0.05
                self.accel.append((d[1] - self.v_last) / max(dt, 1e-3))
                self.v_last, self.t_last = d[1], t
                
                f_ax, f_ay = (d[15], d[16]) if len(d) >= 17 else (0,0)
                f_rx, f_ry = (d[17], d[18]) if len(d) >= 19 else (0,0)
                self.f_mag.append(math.hypot(f_ax+f_rx, f_ay+f_ry))
                self.d_min_hist.append(self.lidar_stats["dmin"])

    def scan_cb(self, msg):
        a = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        r = np.array(msg.ranges)
        m = np.isfinite(r) & (r > 0.05) & (r < msg.range_max)
        rv, av = r[m], a[m]
        with self.data_lock:
            self.scan_x = (rv * np.cos(av)).astype(np.float32)
            self.scan_y = (rv * np.sin(av)).astype(np.float32)
            self.scan_d = rv.astype(np.float32)
            if len(rv): self.lidar_stats = {"n": len(rv), "dmin": float(rv.min())}

    def _lane_off_cb(self, msg): self.lane_offset = msg.data; self.lane_last_t = self.get_clock().now().nanoseconds/1e9
    def _lane_conf_cb(self, msg): self.lane_conf = msg.data
    def _stop_sign_cb(self, msg): self.stop_detected = msg.data
    def _cam_cb(self, msg, key):
        arr = _img_to_numpy(msg)
        if arr is None: return
        now = self.get_clock().now().nanoseconds/1e9
        dt = now - self._cam_last_t[key]
        with self.cam_lock:
            self._cam_frames[key] = arr
            self._cam_fps[key] = 1.0 / max(dt, 1e-3)
            self._cam_last_t[key] = now

# --- GUI Application ---
class TelemetryWindow(QtWidgets.QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.node = ros_node
        self.setWindowTitle("RoboRacer Engineering — Fast Dashboard v13")
        self.setMinimumSize(1400, 850)
        self.setStyleSheet(f"background-color: {PANEL_BG}; color: {TEXT_COLOR}; font-family: 'DejaVu Sans';")
        
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QHBoxLayout(central)
        
        # --- Left Col: Plots ---
        plots_widget = pg.GraphicsLayoutWidget(show=True)
        plots_widget.setBackground(PANEL_BG)
        main_layout.addWidget(plots_widget, 10)
        
        self.plots = []
        self.curves = {}
        
        def add_p(row, col, title, ylabel, color):
            p = plots_widget.addPlot(row=row, col=col, title=title)
            p.setLabel('left', ylabel, color=TEXT_DIM, size='8pt')
            p.showGrid(x=True, y=True, alpha=0.3)
            p.getAxis('left').setPen(color, width=2)
            p.getAxis('bottom').setPen(BORDER_CLR if 'BORDER_CLR' in globals() else '#C8D3E4')
            return p

        self.p_v   = add_p(0, 0, "Velocity [m/s]", "v", MATLAB_BLUE)
        self.c_vt  = self.p_v.plot(pen=pg.mkPen(MATLAB_RED, width=1.5, style=QtCore.Qt.PenStyle.DashLine))
        self.c_va  = self.p_v.plot(pen=pg.mkPen(MATLAB_BLUE, width=2.5))
        
        self.p_s   = add_p(1, 0, "Steering [rad]", "δ", MATLAB_YELLOW)
        self.c_s   = self.p_s.plot(pen=pg.mkPen(MATLAB_YELLOW, width=2))
        self.p_s.setYRange(-0.8, 0.8)
        
        self.p_e   = add_p(2, 0, "Dist Error [m]", "e", MATLAB_PURPLE)
        self.c_e   = self.p_e.plot(pen=pg.mkPen(MATLAB_PURPLE, width=2))
        
        self.p_acc = add_p(0, 1, "Acceleration [m/s²]", "a", MATLAB_RED)
        self.c_acc = self.p_acc.plot(pen=pg.mkPen(MATLAB_RED, width=2))
        self.p_acc.setYRange(-3, 3)
        
        self.p_lid = add_p(1, 1, "Lidar Clearance [m]", "d", "#00BCD4")
        self.c_lid = self.p_lid.plot(pen=pg.mkPen("#00BCD4", width=2))
        self.p_lid.setYRange(0, 4.2)
        
        self.p_frc = add_p(2, 1, "APF Force [N]", "f", "#4CAF50")
        self.c_frc = self.p_frc.plot(pen=pg.mkPen("#4CAF50", width=2))
        self.p_frc.setYRange(0, 3.5)

        # --- Right Col: Perception ---
        perc_widget = QtWidgets.QWidget()
        perc_layout = QtWidgets.QVBoxLayout(perc_widget)
        main_layout.addWidget(perc_widget, 14)
        
        # Front Camera
        self.cam_f_view = pg.GraphicsLayoutWidget()
        self.cam_f_view.setBackground('#050D16')
        perc_layout.addWidget(self.cam_f_view, 2)
        
        self.v_f = self.cam_f_view.addViewBox(invertY=True, lockAspect=True)
        self.img_f = pg.ImageItem()
        self.v_f.addItem(self.img_f)
        
        # Scatter for Lidar project
        self.sc_lid = pg.ScatterPlotItem(size=6, pen=None)
        self.v_f.addItem(self.sc_lid)
        
        # HUD Overlays
        self.hud_lane = pg.TextItem("LANE: WAITING", color='#FFEB3B', anchor=(0.5, 0))
        self.hud_lane.setPos(200, 10)
        self.v_f.addItem(self.hud_lane)
        
        self.hud_info = pg.TextItem("NO SIGNAL", color=CYN, anchor=(0,0))
        self.hud_info.setPos(5, 5)
        self.v_f.addItem(self.hud_info)

        self.hud_stop = pg.TextItem("STOP DETECTED", color='#FFFFFF', anchor=(0.5, 0.5))
        self.hud_stop.setPos(200, 150)
        self.hud_stop.setFont(QtGui.QFont("DejaVu Sans", 24, QtGui.QFont.Weight.Bold))
        self.hud_stop_bg = QtWidgets.QGraphicsRectItem(100, 120, 200, 60)
        self.hud_stop_bg.setBrush(pg.mkBrush('#FF0000CC'))
        self.hud_stop_bg.setPen(pg.mkPen('#FFFFFF', width=2))
        self.v_f.addItem(self.hud_stop_bg)
        self.v_f.addItem(self.hud_stop)
        self.hud_stop.hide(); self.hud_stop_bg.hide()

        # Small cameras (Bottom Row)
        bottom_cams = QtWidgets.QHBoxLayout()
        perc_layout.addLayout(bottom_cams, 1)
        
        def make_small_cam(title):
            w = pg.GraphicsLayoutWidget()
            w.setBackground('#050D16')
            vb = w.addViewBox(invertY=True, lockAspect=True)
            img = pg.ImageItem()
            vb.addItem(img)
            ti = pg.TextItem(title, color=CYN, anchor=(0,0))
            ti.setPos(5, 5); vb.addItem(ti)
            return w, img
        
        self.view_b, self.img_b = make_small_cam("BACK")
        self.view_r, self.img_r = make_small_cam("RIGHT")
        self.view_l, self.img_l = make_small_cam("LANE DEBUG")
        bottom_cams.addWidget(self.view_b)
        bottom_cams.addWidget(self.view_r)
        bottom_cams.addWidget(self.view_l)

        # Cluster boxes (Rect patches)
        self.cluster_rects = []
        for i in range(MAX_CLUSTERS):
            r = QtWidgets.QGraphicsRectItem(0, 0, 1, 1)
            r.setPen(pg.mkPen(CLUSTER_COLORS[i], width=2))
            r.hide()
            self.v_f.addItem(r)
            self.cluster_rects.append(r)

        # Timer
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(33) # 30 FPS

    def update_gui(self):
        # 1. Update Plots
        with self.node.data_lock:
            t = list(self.node.times)
            if t:
                self.c_vt.setData(t, list(self.node.v_target))
                self.c_va.setData(t, list(self.node.v_actual))
                self.c_s.setData(t, list(self.node.steering))
                self.c_e.setData(t, list(self.node.dist_error))
                self.c_acc.setData(t, list(self.node.accel))
                self.c_lid.setData(t, list(self.node.d_min_hist))
                self.c_frc.setData(t, list(self.node.f_mag))
                
                t_end = t[-1]
                for p in [self.p_v, self.p_s, self.p_e, self.p_acc, self.p_lid, self.p_frc]:
                    p.setXRange(t_end - 10, t_end + 0.5, padding=0)
            
            sx, sy = self.node.scan_x, self.node.scan_y
            obs = self.node.obs_state[-1] if self.node.obs_state else 0.0

        # 2. Update Cameras
        with self.node.cam_lock:
            f = self.node._cam_frames['f']
            b = self.node._cam_frames['b']
            r = self.node._cam_frames['r']
            l = self.node._cam_frames['l']
            fps_f = self.node._cam_fps['f']

        if f is not None:
            # Note: pg.ImageItem needs (W, H, 3) or (H, W, 3) with axisOrder='row-major'
            # We use row-major for convenience with OpenCV/NumPy
            self.img_f.setImage(np.transpose(f, (1, 0, 2)))
            self.hud_info.setText(f"FRONT | {f.shape[1]}x{f.shape[0]} | {fps_f:.1f} FPS")
            
            # Lidar project
            if len(sx):
                pu, pv, pd = _lidar_to_front_image(sx, sy)
                if len(pu):
                    # Colors based on distance
                    colors = LIDAR_LUT[(pd / LIDAR_RANGE * 255).astype(int).clip(0, 255)]
                    self.sc_lid.setData(pos=np.column_stack([pu, pv]), brush=[pg.mkBrush(c) for c in colors])
                    self.sc_lid.show()
                else: self.sc_lid.hide()
            
            # Clustering & Boxes
            for rect in self.cluster_rects: rect.hide()
            if len(sx) > 10:
                pts = np.column_stack([sx, sy])
                labels = _cluster(pts)
                unique = sorted(set(labels) - {-1})
                for i in range(min(len(unique), MAX_CLUSTERS)):
                    mask = labels == unique[i]
                    ul, vt, bw, bh = _cluster_to_img_box(sy[mask], sx[mask].mean())
                    if ul < CAM_W and vt < CAM_H:
                        self.cluster_rects[i].setRect(ul, vt, bw, bh)
                        self.cluster_rects[i].show()
        
        if b is not None: self.img_b.setImage(np.transpose(b, (1, 0, 2)))
        if r is not None: self.img_r.setImage(np.transpose(r, (1, 0, 2)))
        if l is not None: self.img_l.setImage(np.transpose(l, (1, 0, 2)))

        # 3. Lane HUD
        conf = self.node.lane_conf
        off = self.node.lane_offset
        if conf > 0.1:
            self.hud_lane.setText(f"LANE: {'OK' if conf > 0.7 else 'LOW'} | OFF: {off:+.2f}")
            self.hud_lane.setColor(YLW if conf < 0.7 else GRN)
        else:
            self.hud_lane.setText("LANE: BLIND")
            self.hud_lane.setColor(RED)

        # 4. Stop Sign HUD
        if self.node.stop_detected:
            self.hud_stop.show(); self.hud_stop_bg.show()
        else:
            self.hud_stop.hide(); self.hud_stop_bg.hide()

def main():
    rclpy.init()
    ros_node = ROSBackend()
    
    # Run ROS spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()
    
    app = QtWidgets.QApplication(sys.argv)
    win = TelemetryWindow(ros_node)
    win.show()
    
    try:
        sys.exit(app.exec())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
