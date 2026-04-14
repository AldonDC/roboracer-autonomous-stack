"""
RoboRacer Engineering Telemetry v11
Left   col : MATLAB time-series  (velocity / steering / distance-error)
Middle col : 2D LiDAR Nav Intelligence  +  bottom (sector-radar / racing-metrics)
Right  col : Perception Fusion Panel
             ┌─────────────────────────────────────┐
             │  FRONT  camera  +  LiDAR projection │  (tall)
             │  DBSCAN cluster boxes overlaid       │
             ├────────────────┬────────────────────┤
             │  BACK  camera  │  RIGHT  camera     │  (small)
             └────────────────┴────────────────────┘
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import LaserScan, Image
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from matplotlib.patches import FancyArrowPatch, Rectangle
from matplotlib.colors import LinearSegmentedColormap, Normalize
import numpy as np
from collections import deque
import threading

# ── Clustering ────────────────────────────────────────────────────────────────
try:
    from sklearn.cluster import DBSCAN as _DBSCAN
    _CLUSTER_BACKEND = 'sklearn'
except ImportError:
    try:
        from scipy.spatial import KDTree as _KDTree
        _CLUSTER_BACKEND = 'scipy'
    except ImportError:
        _CLUSTER_BACKEND = 'none'

# ── Constants ─────────────────────────────────────────────────────────────────
LIDAR_RANGE   = 4.0
MAX_CLUSTERS  = 6
MAX_BBOX_SPAN = 2.4
CLUSTER_COLORS = ['#FF4444', '#44FF88', '#4499FF', '#FFCC00', '#FF44FF', '#44FFFF']

# Camera topics
CAM_FRONT = '/qcar_sim/csi_front/image_raw'
CAM_BACK  = '/qcar_sim/csi_back/image_raw'
CAM_RIGHT = '/qcar_sim/csi_right/image_raw'

# Camera intrinsics (640×480, ~65° H-FoV)
CAM_W, CAM_H = 640, 480
CAM_FX = CAM_FY = 490.0
CAM_CX, CAM_CY = CAM_W / 2, CAM_H / 2
CAM_Z_OBS = 0.25   # assumed obstacle-centre height above ground [m]
CAM_Z_CAM = 0.12   # camera height above ground [m]

# ── Colour palette ────────────────────────────────────────────────────────────
PANEL_BG      = '#FFFFFF'          # figure / panel background (pure white)
PLOT_BG       = '#F6F8FC'          # very-light-blue tint inside axes
GRID_COLOR    = '#DDE4EF'          # grid lines
BORDER_CLR    = '#C8D3E4'          # axis spines
TEXT_COLOR    = '#2C3E50'          # main text
TEXT_DIM      = '#7A8A9A'          # secondary / axis labels
MATLAB_BLUE   = '#0072BD'
MATLAB_RED    = '#D95319'
MATLAB_YELLOW = '#EDB120'
MATLAB_PURPLE = '#7E2F8E'
GRN = '#3FB950';  YLW = '#E3B341';  CYN = '#00E5CC';  DIM = '#8B949E'

LIDAR_CMAP = LinearSegmentedColormap.from_list(
    'lidar', ['#00FFFF', '#00FF88', '#AAFF00', '#FFFF00', '#FF8800', '#FF2200'])


# ── Helpers ───────────────────────────────────────────────────────────────────
def _bfs_cluster(pts, eps=0.25, min_pts=4):
    n, labels = len(pts), np.full(len(pts), -1, dtype=np.int32)
    visited = np.zeros(n, dtype=bool)
    tree = _KDTree(pts)
    cid = 0
    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        nbrs = tree.query_ball_point(pts[i], eps)
        if len(nbrs) < min_pts:
            continue
        labels[i] = cid
        stack = [j for j in nbrs if j != i and not visited[j]]
        while stack:
            j = stack.pop()
            if visited[j]:
                continue
            visited[j] = True
            labels[j] = cid
            new_nbrs = tree.query_ball_point(pts[j], eps)
            if len(new_nbrs) >= min_pts:
                stack.extend(k for k in new_nbrs if not visited[k])
        cid += 1
    return labels


def _cluster(pts):
    EPS, MIN_PTS = 0.25, 4
    if _CLUSTER_BACKEND == 'sklearn':
        labels = _DBSCAN(eps=EPS, min_samples=MIN_PTS).fit_predict(pts)
    elif _CLUSTER_BACKEND == 'scipy':
        labels = _bfs_cluster(pts, EPS, MIN_PTS)
    else:
        return np.full(len(pts), -1, dtype=np.int32)
    for lbl in set(labels):
        if lbl < 0:
            continue
        m = labels == lbl
        if max(pts[m, 0].ptp(), pts[m, 1].ptp()) > MAX_BBOX_SPAN:
            labels[m] = -1
    return labels


def _ackermann_arc(delta, travel=2.2, n=45):
    if abs(delta) < 0.015:
        return np.array([0.0, travel]), np.array([0.0, 0.0])
    R = 0.256 / math.tan(abs(delta))
    side = 1.0 if delta > 0 else -1.0
    cy = side * R
    phi0 = math.atan2(-cy, 0.0)
    phis = np.linspace(phi0, phi0 + side * travel / R, n)
    return R * np.cos(phis), cy + R * np.sin(phis)


def _img_to_numpy(msg):
    """Convert sensor_msgs/Image to RGB numpy array (no cv_bridge)."""
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
    """
    Project 2D LiDAR points (robot frame: x=fwd, y=left) onto the front
    camera image plane using a pinhole model.
    Returns (u_px, v_px, dist) arrays for points visible in the image.
    """
    fwd = scan_x > 0.15
    if not np.any(fwd):
        return np.array([]), np.array([]), np.array([])
    x = scan_x[fwd].astype(np.float64)
    y = scan_y[fwd].astype(np.float64)
    dz = CAM_Z_OBS - CAM_Z_CAM
    u  = CAM_CX - CAM_FX * y / x          # lateral: left→image-left
    v  = CAM_CY - CAM_FY * dz / x         # vertical: above cam → above centre
    d  = np.hypot(x, y)
    ok = (u >= 0) & (u < CAM_W) & (v >= 0) & (v < CAM_H)
    return u[ok], v[ok], d[ok]


def _cluster_to_img_box(cy_arr, x_centre):
    """
    Returns (u_left, v_top, box_w_px, box_h_px) in image pixels
    for a cluster given its LiDAR-frame bounding box.
    """
    x = max(float(x_centre), 0.15)
    # Horizontal: project y_min and y_max
    y_min, y_max = float(cy_arr.min()), float(cy_arr.max())
    u_right = CAM_CX - CAM_FX * y_min / x   # y_min → rightmost px
    u_left  = CAM_CX - CAM_FX * y_max / x   # y_max → leftmost px
    # Vertical: obstacle from ground to ~0.5m above camera
    z_top, z_bot = CAM_Z_CAM + 0.50, 0.0
    v_top = CAM_CY - CAM_FY * (z_top - CAM_Z_CAM) / x
    v_bot = CAM_CY - CAM_FY * (z_bot - CAM_Z_CAM) / x
    # Clamp to image
    u_l = max(0, min(int(u_left),  CAM_W - 1))
    u_r = max(0, min(int(u_right), CAM_W - 1))
    v_t = max(0, min(int(v_top),   CAM_H - 1))
    v_b = max(0, min(int(v_bot),   CAM_H - 1))
    return u_l, v_t, max(u_r - u_l, 4), max(v_b - v_t, 4)


# ─────────────────────────────────────────────────────────────────────────────
class TelemetryDashboard(Node):
    def __init__(self):
        super().__init__('telemetry_dashboard')
        self.sub = self.create_subscription(
            Float32MultiArray, '/viz/telemetry', self.telemetry_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/qcar_sim/scan', self.scan_cb, 10)
        # Camera subscribers (BEST_EFFORT, depth=2 to avoid queue build-up)
        _cam_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        self.cam_f_sub = self.create_subscription(
            Image, CAM_FRONT, lambda m: self._cam_cb(m, 'f'), _cam_qos)
        self.cam_b_sub = self.create_subscription(
            Image, CAM_BACK,  lambda m: self._cam_cb(m, 'b'), _cam_qos)
        self.cam_r_sub = self.create_subscription(
            Image, CAM_RIGHT, lambda m: self._cam_cb(m, 'r'), _cam_qos)

        # Lane-Detector debug feed
        self.lane_off_sub = self.create_subscription(
            Float32, '/lane/center_offset', self._lane_off_cb, 10)
        self.lane_conf_sub = self.create_subscription(
            Float32, '/lane/confidence', self._lane_conf_cb, 10)
        self.lane_img_sub = self.create_subscription(
            Image, '/lane/image_debug', self._lane_img_cb, _cam_qos)
        self.lane_offset = 0.0
        self.lane_conf = 0.0
        self.lane_last_t = 0.0
        self.lane_dbg_frame = None

        self.max_pts    = 200
        self.times      = deque(maxlen=self.max_pts)
        self.v_target   = deque(maxlen=self.max_pts)
        self.v_actual   = deque(maxlen=self.max_pts)
        self.steering   = deque(maxlen=self.max_pts)
        self.dist_error = deque(maxlen=self.max_pts)
        self.accel      = deque(maxlen=self.max_pts)
        self.f_mag      = deque(maxlen=self.max_pts)
        self.d_min_hist = deque(maxlen=self.max_pts)
        self.obs_state  = deque(maxlen=self.max_pts)

        self.scan_x      = np.array([], dtype=np.float32)
        self.scan_y      = np.array([], dtype=np.float32)
        self.scan_d      = np.array([], dtype=np.float32)
        self.lidar_stats = dict(n=0, dmin=0.0)

        self.pred_px = [];  self.pred_py = []
        self.f_att   = (0.0, 0.0);  self.f_rep = (0.0, 0.0)
        self.sectors = [LIDAR_RANGE] * 12
        self.v_last  = 0.0
        self.t_last  = 0.0
        self.start_time = None

        # Camera frames & info (per key 'f', 'b', 'r')
        self._cam_frames = {'f': None, 'b': None, 'r': None}
        self._cam_fps    = {'f': 0.0,  'b': 0.0,  'r': 0.0}
        self._cam_last_t = {'f': 0.0,  'b': 0.0,  'r': 0.0}
        self._cam_lock   = threading.Lock()

        self.get_logger().info(
            f'RoboRacer Telemetry v11  ·  3-cam fusion  ·  cluster={_CLUSTER_BACKEND}')

    def telemetry_cb(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        d = msg.data
        if len(d) >= 5:
            self.times.append(t);  self.v_target.append(d[0])
            self.v_actual.append(d[1]);  self.steering.append(d[2])
            self.dist_error.append(d[3]);  self.obs_state.append(d[4])

            # Synchronized derived metrics
            dt = t - self.t_last if self.t_last > 0 else 0.05
            dv = d[1] - self.v_last
            acc = dv / max(dt, 1e-3)
            self.accel.append(acc)
            self.v_last, self.t_last = d[1], t
            
            f_ax, f_ay = (float(d[15]), float(d[16])) if len(d) >= 17 else (0.0, 0.0)
            f_rx, f_ry = (float(d[17]), float(d[18])) if len(d) >= 19 else (0.0, 0.0)
            self.f_mag.append(math.hypot(f_ax+f_rx, f_ay+f_ry))
            self.d_min_hist.append(self.lidar_stats.get("dmin", 0.0))

        if len(d) >= 15:
            self.pred_px = list(d[5:15:2])
            self.pred_py = list(d[6:15:2])
        if len(d) >= 31:
            self.sectors = [float(v) for v in d[19:31]]

    def scan_cb(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        r = np.array(msg.ranges)
        mask = np.isfinite(r) & (r > 0.05) & (r < msg.range_max)
        rv, av = r[mask], angles[mask]
        self.scan_x = (rv * np.cos(av)).astype(np.float32)
        self.scan_y = (rv * np.sin(av)).astype(np.float32)
        self.scan_d = rv.astype(np.float32)
        if len(rv):
            self.lidar_stats = dict(n=int(len(rv)), dmin=float(rv.min()))

    def _lane_off_cb(self, msg):
        self.lane_offset = float(msg.data)
        self.lane_last_t = self.get_clock().now().nanoseconds / 1e9

    def _lane_conf_cb(self, msg):
        self.lane_conf = float(msg.data)

    def _lane_img_cb(self, msg):
        arr = _img_to_numpy(msg)
        if arr is None:
            return
        with self._cam_lock:
            self.lane_dbg_frame = arr

    def _cam_cb(self, msg, key):
        arr = _img_to_numpy(msg)
        if arr is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self._cam_last_t[key]
        with self._cam_lock:
            self._cam_frames[key] = arr
            self._cam_fps[key]    = 1.0 / max(dt, 1e-3)
            self._cam_last_t[key] = now


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TelemetryDashboard()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    plt.rcParams.update({
        'font.family':          'DejaVu Sans',
        'font.size':            8.5,
        'axes.titlesize':       9.5,
        'axes.titlepad':        5,
        'text.color':           TEXT_COLOR,
        'axes.labelcolor':      TEXT_DIM,
        'xtick.color':          TEXT_DIM,
        'ytick.color':          TEXT_DIM,
        'figure.facecolor':     PANEL_BG,
        'savefig.facecolor':    PANEL_BG,
        'axes.spines.top':      False,
        'axes.spines.right':    False,
    })

    fig = plt.figure(figsize=(24, 9), facecolor=PANEL_BG)
    fig.canvas.manager.set_window_title('RoboRacer — Engineering Telemetry v12')

    # ── 2-column layout ───────────────────────────────────────────────────────
    gs = gridspec.GridSpec(3, 2, figure=fig,
                           left=0.055, right=0.975, top=0.89, bottom=0.08,
                           wspace=0.32, hspace=0.56,
                           width_ratios=[1.0, 1.35])

    # Sub-grid for left column (3 rows x 2 columns)
    gs_left = gridspec.GridSpecFromSubplotSpec(3, 2, gs[:, 0], hspace=0.48, wspace=0.35)
    ax_v   = fig.add_subplot(gs_left[0, 0])
    ax_s   = fig.add_subplot(gs_left[1, 0])
    ax_e   = fig.add_subplot(gs_left[2, 0])
    ax_acc = fig.add_subplot(gs_left[0, 1])
    ax_lid = fig.add_subplot(gs_left[1, 1])
    ax_frc = fig.add_subplot(gs_left[2, 1])

    # Right col: 3 camera panels
    gs_perc   = gridspec.GridSpecFromSubplotSpec(3, 2, gs[:, 1],
                                                  hspace=0.28, wspace=0.14)
    ax_cf = fig.add_subplot(gs_perc[0:2, :])  # FRONT — tall (2 rows × 2 cols)
    ax_cb = fig.add_subplot(gs_perc[2, 0])    # BACK  — small bottom-left
    ax_cr = fig.add_subplot(gs_perc[2, 1])    # RIGHT — small bottom-right

    # ══════════════════════════════════════════════════════════════════════════
    # MATLAB time-series panels
    # ══════════════════════════════════════════════════════════════════════════
    def _mstyle(ax, title, ylabel, accent=MATLAB_BLUE):
        ax.set_facecolor(PLOT_BG)
        ax.set_title(title, fontsize=9, color=TEXT_COLOR, fontweight='bold',
                     pad=5, loc='left')
        ax.set_ylabel(ylabel, fontsize=7.5, color=TEXT_DIM, labelpad=4)
        ax.set_xlabel('t  [s]', fontsize=7, color=TEXT_DIM, labelpad=2)
        ax.tick_params(axis='both', which='major', direction='out', length=3,
                       width=0.7, colors=TEXT_DIM, labelsize=7, pad=2)
        ax.grid(True, color=GRID_COLOR, linewidth=0.55, linestyle='--',
                alpha=0.9, zorder=0)
        ax.set_axisbelow(True)
        ax.spines['left'].set_color(accent);     ax.spines['left'].set_linewidth(2.5)
        ax.spines['bottom'].set_color(BORDER_CLR); ax.spines['bottom'].set_linewidth(0.8)

    _mstyle(ax_v,   'Velocity',            'm/s',  MATLAB_BLUE)
    _mstyle(ax_s,   'Steering Angle',      'rad',  MATLAB_YELLOW)
    _mstyle(ax_e,   'Waypoint Error',      'm',    MATLAB_PURPLE)
    _mstyle(ax_acc, 'Acceleration',        'm/s²', MATLAB_RED)
    _mstyle(ax_lid, 'LiDAR Clearance',     'm',    '#00BCD4')
    _mstyle(ax_frc, 'APF Force',           'N',    '#4CAF50')

    line_vt, = ax_v.plot([], [], '--', color=MATLAB_RED,  lw=1.6, alpha=0.80,
                          label='v_ref',    zorder=3)
    line_va, = ax_v.plot([], [], '-',  color=MATLAB_BLUE, lw=2.4,
                          label='v_actual', zorder=4)
    ax_v.legend(fontsize=7.5, loc='upper right', framealpha=0.92,
                facecolor=PANEL_BG, edgecolor=BORDER_CLR, labelcolor=TEXT_COLOR,
                borderpad=0.5, handlelength=1.8)
    ax_v.axhline(0, color=BORDER_CLR, lw=0.8, zorder=1)
    line_s,   = ax_s.plot([], [], '-', color=MATLAB_YELLOW, lw=2.2, zorder=3)
    ax_s.set_ylim(-0.75, 0.75);  ax_s.axhline(0, color=BORDER_CLR, lw=0.8, zorder=1)
    line_e,   = ax_e.plot([], [], '-', color=MATLAB_PURPLE, lw=2.2, zorder=3)
    ax_e.set_ylim(0, 5)
    line_acc, = ax_acc.plot([], [], '-', color=MATLAB_RED, lw=2.0, zorder=3)
    ax_acc.set_ylim(-2.5, 2.5); ax_acc.axhline(0, color=BORDER_CLR, lw=0.8, zorder=1)
    line_lid, = ax_lid.plot([], [], '-', color='#00BCD4', lw=2.0, zorder=3)
    ax_lid.set_ylim(0, 4.2)
    # Danger threshold lines on clearance plot
    ax_lid.axhline(0.42, color='#EF5350', lw=0.9, ls='--', alpha=0.7, zorder=2)
    ax_lid.axhline(1.80, color='#FFA726', lw=0.9, ls='--', alpha=0.7, zorder=2)
    line_frc, = ax_frc.plot([], [], '-', color='#4CAF50', lw=2.0, zorder=3)
    ax_frc.set_ylim(0, 3.0)

    # ══════════════════════════════════════════════════════════════════════════
    # PERCEPTION FUSION — 3 camera panels
    # ══════════════════════════════════════════════════════════════════════════
    _blank_f = np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8)
    _blank_s = np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8)

    def _cam_style(ax, title, badge='◉'):
        ax.set_facecolor('#050D16')
        ax.axis('off')
        ax.set_title(f'{badge}  {title}', fontsize=8, color=CYN,
                     fontweight='bold', pad=5, loc='left',
                     fontfamily='DejaVu Sans')
        for sp in ax.spines.values():
            sp.set_color('#1C3D56');  sp.set_linewidth(1.5)

    _cam_style(ax_cf, 'PERCEPTION FUSION  ·  FRONT  +  LiDAR', '◈')
    _cam_style(ax_cb, 'BACK CAM',   '◉')
    _cam_style(ax_cr, 'RIGHT CAM',  '◉')

    # Base images
    im_cf = ax_cf.imshow(_blank_f, aspect='auto', interpolation='bilinear', zorder=1)
    im_cb = ax_cb.imshow(_blank_s, aspect='auto', interpolation='bilinear', zorder=1)
    im_cr = ax_cr.imshow(_blank_s, aspect='auto', interpolation='bilinear', zorder=1)

    # ── Corner HUD markers (front camera) ─────────────────────────────────────
    _clen = 0.07;  _cpad = 0.018;  _hcc = '#00FFAA'
    for (ox, oy, dx, dy) in [
        (0,0,1,0),(0,0,0,1),(1,0,-1,0),(1,0,0,1),
        (0,1,1,0),(0,1,0,-1),(1,1,-1,0),(1,1,0,-1)]:
        ax_cf.plot([ox+dx*_cpad, ox+dx*(_cpad+_clen)],
                   [oy+dy*_cpad, oy+dy*(_cpad+_clen)],
                   transform=ax_cf.transAxes,
                   color=_hcc, lw=1.8, zorder=6, solid_capstyle='round')

    # Crosshair reticle
    for (x0,y0,x1,y1) in [(0.46,0.5,0.49,0.5),(0.51,0.5,0.54,0.5),
                            (0.5,0.46,0.5,0.49),(0.5,0.51,0.5,0.54)]:
        ax_cf.plot([x0,x1],[y0,y1], transform=ax_cf.transAxes,
                   color='#00FFAA', lw=0.9, alpha=0.55, zorder=6)

    # LiDAR projection scatter (on front camera image coords)
    norm_proj = Normalize(vmin=0, vmax=LIDAR_RANGE)
    proj_sc = ax_cf.scatter([], [], s=12, c=[], cmap=LIDAR_CMAP,
                             norm=norm_proj, alpha=0.75, zorder=5,
                             edgecolors='none')

    # DBSCAN cluster boxes drawn as Rectangle patches on the camera axes
    # We pre-allocate patches and reuse them
    _MAX_BOX = MAX_CLUSTERS
    cam_boxes   = []   # Rectangle patches
    cam_box_lbl = []   # Text labels
    for i in range(_MAX_BOX):
        rect = Rectangle((0, 0), 1, 1, lw=2.0, edgecolor=CLUSTER_COLORS[i],
                          facecolor='none', visible=False, zorder=7)
        ax_cf.add_patch(rect)
        cam_boxes.append(rect)
        lbl = ax_cf.text(0, 0, '', color=CLUSTER_COLORS[i],
                          fontsize=6.5, fontweight='bold',
                          va='bottom', ha='left', zorder=8, visible=False,
                          bbox=dict(fc='#00000088', ec='none',
                                    boxstyle='round,pad=0.2'))
        cam_box_lbl.append(lbl)

    # Info overlays (front camera)
    cf_info = ax_cf.text(0.01, 0.985, 'NO SIGNAL', transform=ax_cf.transAxes,
                          fontsize=7, color='#00FFAA', va='top', ha='left',
                          fontfamily='monospace',
                          bbox=dict(fc='#00000088',ec='none',
                                    boxstyle='round,pad=0.3'), zorder=9)
    cf_fps  = ax_cf.text(0.99, 0.985, '',  transform=ax_cf.transAxes,
                          fontsize=7, color='#FFD740', va='top', ha='right',
                          fontfamily='monospace',
                          bbox=dict(fc='#00000088',ec='none',
                                    boxstyle='round,pad=0.3'), zorder=9)
    cf_aeb  = ax_cf.text(0.01, 0.025, '●  NOMINAL', transform=ax_cf.transAxes,
                          fontsize=7.5, fontweight='bold', color=GRN,
                          va='bottom', ha='left', fontfamily='monospace',
                          bbox=dict(fc='#00000099',ec='none',
                                    boxstyle='round,pad=0.35'), zorder=9)
    cf_det  = ax_cf.text(0.99, 0.025, '',  transform=ax_cf.transAxes,
                          fontsize=7, color='#58A6FF', va='bottom', ha='right',
                          fontfamily='monospace',
                          bbox=dict(fc='#00000088',ec='none',
                                    boxstyle='round,pad=0.3'), zorder=9)

    # ── LANE-DETECTOR debug HUD (amarillo) ────────────────────────────────────
    # Texto con offset/conf + estado DETECTED/BLIND
    cf_lane_txt = ax_cf.text(
        0.5, 0.94, 'LANE  —  waiting...', transform=ax_cf.transAxes,
        fontsize=8, color='#FFEB3B', va='top', ha='center',
        fontfamily='monospace', fontweight='bold',
        bbox=dict(fc='#00000099', ec='#FFEB3B', lw=1.2,
                  boxstyle='round,pad=0.35'), zorder=10)
    # Barra horizontal de offset (center track): de -1 (izq) a +1 (der)
    # Ejes transAxes: x en [0.30, 0.70], y fijo a 0.89
    _bar_y = 0.885
    _bar_left, _bar_right = 0.30, 0.70
    # Fondo de la barra
    ax_cf.plot([_bar_left, _bar_right], [_bar_y, _bar_y],
               transform=ax_cf.transAxes, color='#555555', lw=5,
               solid_capstyle='round', zorder=10, alpha=0.6)
    # Tick central (referencia carril ideal)
    ax_cf.plot([0.5, 0.5], [_bar_y - 0.012, _bar_y + 0.012],
               transform=ax_cf.transAxes, color='#FFFFFF', lw=1.5, zorder=11)
    # Marcador móvil (punto sobre la barra)
    lane_marker, = ax_cf.plot([0.5], [_bar_y], 'o', transform=ax_cf.transAxes,
                              color='#FFEB3B', markersize=10,
                              markeredgecolor='#000000', markeredgewidth=1.2,
                              zorder=12)

    # No-signal overlays for all 3 cameras
    _ns_kw = dict(fontsize=10, color='#1A3A50', fontweight='bold',
                  va='center', ha='center', fontfamily='DejaVu Sans', alpha=0.55)
    ns_cf = ax_cf.text(0.5, 0.5, 'NO SIGNAL', transform=ax_cf.transAxes, **_ns_kw)
    ns_cb = ax_cb.text(0.5, 0.5, 'NO SIGNAL', transform=ax_cb.transAxes, **_ns_kw)
    ns_cr = ax_cr.text(0.5, 0.5, 'NO SIGNAL', transform=ax_cr.transAxes, **_ns_kw)

    # Corner markers for small cameras
    for ax_s_ in (ax_cb, ax_cr):
        for (ox, oy, dx, dy) in [(0,0,1,0),(0,0,0,1),(1,0,-1,0),(1,0,0,1),
                                   (0,1,1,0),(0,1,0,-1),(1,1,-1,0),(1,1,0,-1)]:
            ax_s_.plot([ox+dx*0.04, ox+dx*0.14],[oy+dy*0.04, oy+dy*0.14],
                       transform=ax_s_.transAxes,
                       color='#00FFAA', lw=1.2, zorder=5, solid_capstyle='round')

    # ── Global header ─────────────────────────────────────────────────────────
    fig.suptitle('RoboRacer  ·  Engineering Telemetry  v12',
                 fontsize=14, fontweight='bold', color=TEXT_COLOR, y=0.978,
                 fontfamily='DejaVu Sans')
    fig.text(0.5, 0.958, 'Real-time Autonomous Navigation Dashboard',
             ha='center', fontsize=8, color=TEXT_DIM, fontfamily='DejaVu Sans')
    # Thin rule under the header
    fig.add_artist(plt.Line2D([0.03, 0.97], [0.948, 0.948],
                               transform=fig.transFigure,
                               color=BORDER_CLR, lw=0.8, zorder=0))
    # ── Animation ─────────────────────────────────────────────────────────────
    def update(_frame):
        # ── Time-series ───────────────────────────────────────────────────────
        if node.times:
            # Snapshot data to ensure consistent lengths during this frame
            with threading.Lock(): # Simple way to prevent mid-cb updates
                t  = list(node.times)
                vt = list(node.v_target)
                va = list(node.v_actual)
                st = list(node.steering)
                de = list(node.dist_error)
                ac = list(node.accel)
                fm = list(node.f_mag)
                dm = list(node.d_min_hist)

            t0 = max(0.0, t[-1] - 10.0);  t1 = t[-1] + 0.5
            line_vt.set_data(t, vt)
            line_va.set_data(t, va)
            line_s.set_data( t, st)
            line_e.set_data( t, de)
            line_acc.set_data(t, ac)
            line_lid.set_data(t, dm)
            line_frc.set_data(t, fm)

            for ax in (ax_v, ax_s, ax_e, ax_acc, ax_lid, ax_frc):
                ax.set_xlim(t0, t1)

            vv = vt + va
            if vv:
                ax_v.set_ylim(min(vv) - 0.3, max(vv) + 0.3)
            if de:
                ax_e.set_ylim(0, max(max(de) * 1.2, 0.5))

        obs = float(node.obs_state[-1]) if node.obs_state else 0.0
        v_now = float(node.v_actual[-1]) if node.v_actual else 0.0

        # ── Cluster data for camera boxes ─────────────────────────────────────
        sx, sy, sd = node.scan_x, node.scan_y, node.scan_d
        n_clusters = 0
        cluster_data = []

        if len(sx) > 0:
            pts    = np.column_stack([sx, sy])
            labels = _cluster(pts)
            unique = sorted(set(labels) - {-1})
            n_clusters = min(len(unique), MAX_CLUSTERS)
            for i in range(n_clusters):
                mask = labels == unique[i]
                cx, cy = pts[mask, 0], pts[mask, 1]
                ctr_d = float(np.hypot(cx.mean(), cy.mean()))
                cluster_data.append((cx, cy, float(cx.mean()),
                                     ctr_d, int(mask.sum())))

        # ── Perception Fusion ─────────────────────────────────────────────────
        with node._cam_lock:
            frames = dict(node._cam_frames)
            fps    = dict(node._cam_fps)

        # Front camera
        if frames['f'] is not None:
            im_cf.set_data(frames['f'])
            ns_cf.set_visible(False)
            cf_info.set_text(f'{CAM_W}×{CAM_H}  csi_front')
            cf_fps.set_text(f'{fps["f"]:.1f} fps')
        else:
            ns_cf.set_visible(True)
            cf_info.set_text('NO SIGNAL — FRONT')
            cf_fps.set_text('—')

        # LiDAR → camera projection
        if len(sx) > 0:
            pu, pv, pd = _lidar_to_front_image(sx, sy)
            if len(pu) > 0:
                proj_sc.set_offsets(np.column_stack([pu, pv]))
                proj_sc.set_array(pd);  proj_sc.set_visible(True)
            else:
                proj_sc.set_offsets(np.empty((0,2)));  proj_sc.set_visible(False)
        else:
            proj_sc.set_offsets(np.empty((0,2)));  proj_sc.set_visible(False)

        # DBSCAN cluster boxes on camera image
        for i in range(_MAX_BOX):
            if i < len(cluster_data):
                cx_a, cy_a, x_c, d_c, n_pts = cluster_data[i]
                ul, vt, bw, bh = _cluster_to_img_box(cy_a, x_c)
                # Only show if box is reasonably within image
                if ul < CAM_W and vt < CAM_H and bw > 5 and d_c < LIDAR_RANGE:
                    cam_boxes[i].set_xy((ul, vt))
                    cam_boxes[i].set_width(bw);  cam_boxes[i].set_height(bh)
                    cam_boxes[i].set_visible(True)
                    cam_box_lbl[i].set_position((ul + 2, max(vt - 2, 0)))
                    cam_box_lbl[i].set_text(f'OBJ{i+1}  {d_c:.2f}m  [{n_pts}pt]')
                    cam_box_lbl[i].set_visible(True)
                else:
                    cam_boxes[i].set_visible(False);  cam_box_lbl[i].set_visible(False)
            else:
                cam_boxes[i].set_visible(False);  cam_box_lbl[i].set_visible(False)

        # AEB + detection summary on front camera
        if obs == 2.0:   cf_aeb.set_text('⚠  AEB  ACTIVE'); cf_aeb.set_color('#FF3D00')
        elif obs == 1.0: cf_aeb.set_text('◉  OBSTACLE');     cf_aeb.set_color(YLW)
        else:            cf_aeb.set_text('●  NOMINAL');       cf_aeb.set_color(GRN)
        cf_det.set_text(f'{n_clusters} obj  |  v={v_now:+.2f}m/s')

        # ── LANE-DETECTOR HUD ─────────────────────────────────────────────────
        off = float(node.lane_offset)
        conf = float(node.lane_conf)
        now_t = node.get_clock().now().nanoseconds / 1e9
        age = now_t - node.lane_last_t if node.lane_last_t > 0 else 99.0
        stale = age > 1.0

        if stale:
            cf_lane_txt.set_text('LANE  ✗  NO SIGNAL')
            cf_lane_txt.set_color('#FF5252')
            cf_lane_txt.get_bbox_patch().set_edgecolor('#FF5252')
            lane_marker.set_color('#666666')
        elif conf >= 0.25:
            status = '✓ DETECTED' if conf > 0.7 else '◐ PARTIAL'
            cf_lane_txt.set_text(
                f'LANE  {status}   off={off:+.2f}  conf={conf:.2f}')
            cf_lane_txt.set_color('#FFEB3B' if conf > 0.7 else '#FFB300')
            cf_lane_txt.get_bbox_patch().set_edgecolor(
                '#FFEB3B' if conf > 0.7 else '#FFB300')
            lane_marker.set_color('#FFEB3B' if conf > 0.7 else '#FFB300')
        else:
            cf_lane_txt.set_text(
                f'LANE  ✗  BLIND   off={off:+.2f}  conf={conf:.2f}')
            cf_lane_txt.set_color('#FF8A65')
            cf_lane_txt.get_bbox_patch().set_edgecolor('#FF8A65')
            lane_marker.set_color('#FF8A65')

        # Posición del marcador en la barra (offset ∈ [-1, 1] → [_bar_left, _bar_right])
        off_clip = max(-1.0, min(1.0, off))
        x_marker = _bar_left + (off_clip + 1.0) * 0.5 * (_bar_right - _bar_left)
        lane_marker.set_data([x_marker], [_bar_y])

        # Back camera
        if frames['b'] is not None:
            im_cb.set_data(frames['b']);  ns_cb.set_visible(False)
        else:
            ns_cb.set_visible(True)

        # Right camera
        if frames['r'] is not None:
            im_cr.set_data(frames['r']);  ns_cr.set_visible(False)
        else:
            ns_cr.set_visible(True)

        return (line_vt, line_va, line_s, line_e,
                line_acc, line_lid, line_frc,
                im_cf, im_cb, im_cr, proj_sc,
                cf_info, cf_fps, cf_aeb, cf_det,
                cf_lane_txt, lane_marker,
                *cam_boxes, *cam_box_lbl)

    ani = animation.FuncAnimation(fig, update, interval=80, blit=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
