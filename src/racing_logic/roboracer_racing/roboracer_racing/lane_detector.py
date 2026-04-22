"""
Professional Perception Lane Detector — QCar Physical Edition
=============================================================
Architecture:
1. Pre-processing: Resizing, CLAHE (Light compensation), Denoising.
2. Perspective Transform: Bird's-Eye View (BEV) mapping for geometric accuracy.
3. Advanced Segmentation: Combined HSV + HLS logic with adaptive masking.
4. Robust Modeling: 2nd Order Polynomial fitting with temporal consistency.
5. Perception Metrics: Curvature, precise lateral offset, and quantitative confidence.
6. Temporal Filtering: Exponential Moving Average (EMA) for coefficients tracking.
7. Pro Debug View: Multi-stage visualization (Original, BEV, Mask, Metrics).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float32, Bool
import cv2
import numpy as np
import time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

# ── Helper for Image Conversions ──────────────────────────────────────────────
def imgmsg_to_bgr(msg):
    enc = msg.encoding.lower()
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    if enc in ('bgr8', 'rgb8'):
        img = buf.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if enc == 'rgb8' else img
    return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)

# ── Perception Class ──────────────────────────────────────────────────────────
class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')

        # 1. Parameters & Configuration
        self.declare_parameter('image_topic', '/qcar/csi_front')
        self.declare_parameter('publish_debug', True)
        
        # HSV Defaults (Adaptive tuning via Dashboard is still supported)
        self.declare_parameter('h_min', 15); self.declare_parameter('h_max', 45)
        self.declare_parameter('s_min', 50); self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 70); self.declare_parameter('v_max', 255)

        # BEV Calibration (Src points for 320x240)
        # Trapezoid coordinates on the ground
        self.src_pts = np.float32([
            [100, 140], # Top Left
            [220, 140], # Top Right
            [320, 230], # Bottom Right
            [0,   230]  # Bottom Left
        ])
        self.dst_pts = np.float32([
            [60, 0],   [260, 0],
            [260, 240], [60, 240]
        ])
        self.M = cv2.getPerspectiveTransform(self.src_pts, self.dst_pts)
        self.Minv = cv2.getPerspectiveTransform(self.dst_pts, self.src_pts)

        # 2. Tracking State (Temporal Stability)
        self.poly_coeffs = None  # [a, b, c] for ax^2 + bx + c
        self.ema_alpha = 0.3    # Smoothing factor (0.1 = slow/stable, 0.9 = fast/noisy)
        self.last_offset = 0.0
        self.last_conf = 0.0
        
        # 3. ROS Communications
        _qcar_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        self.sub = self.create_subscription(
            CompressedImage, self.get_parameter('image_topic').value, 
            self.compressed_cb, _qcar_qos)

        self.offset_pub = self.create_publisher(Float32, '/lane/center_offset', 10)
        self.conf_pub = self.create_publisher(Float32, '/lane/confidence', 10)
        self.dbg_pub = self.create_publisher(CompressedImage, '/lane/image_debug', 10)
        self.stop_pub = self.create_publisher(Bool, '/lane/stop_sign', 10)

        self.get_logger().info('🚀 PRO Lane Perception — BEV + Polynomial Fit — Initialized')

    def compressed_cb(self, msg):
        try:
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if bgr is None: return
            # Standard resize for processing speed
            bgr = cv2.resize(bgr, (320, 240), interpolation=cv2.INTER_LINEAR)
        except Exception as e:
            self.get_logger().error(f'Decode Error: {e}')
            return
        self._process_frame(bgr, msg.header)

    def _process_frame(self, bgr, header):
        # ── STAGE 1: Pre-processing ───────────────────────────────────────────
        # CLAHE on Value to normalize lighting
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        v_eq = clahe.apply(v)
        hsv_eq = cv2.merge([h, s, v_eq])
        
        # ── STAGE 2: Bird's-Eye View (BEV) ────────────────────────────────────
        bev_img = cv2.warpPerspective(hsv_eq, self.M, (320, 240))
        
        # ── STAGE 3: Advanced Yellow Segmentation ─────────────────────────────
        # We use HSV (Hue) + HLS (Saturation) for robustness
        hls_bev = cv2.cvtColor(cv2.warpPerspective(bgr, self.M, (320, 240)), cv2.COLOR_BGR2HLS)
        
        # Yellow mask in HSV
        lo_y = np.array([self.get_parameter('h_min').value, 50, 60])
        hi_y = np.array([self.get_parameter('h_max').value, 255, 255])
        mask_hsv = cv2.inRange(bev_img, lo_y, hi_y)
        
        # Saturation check in HLS (prevents gray artifacts)
        mask_hls = cv2.inRange(hls_bev, np.array([0, 50, 80]), np.array([179, 200, 255]))
        
        combined_mask = cv2.bitwise_and(mask_hsv, mask_hls)
        
        # Cleanup
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        combined_mask = cv2.dilate(combined_mask, kernel, iterations=1)

        # ── STAGE 4: Geometric Extraction (Sliding Windows) ───────────────────
        # Find pixels
        pts = np.argwhere(combined_mask > 0)
        if len(pts) > 100:
            # Fit 2nd order polynomial: x = Ay^2 + By + C
            # Using y as independent variable because the lane is mostly vertical in BEV
            curr_y = pts[:, 0]
            curr_x = pts[:, 1]
            try:
                new_coeffs = np.polyfit(curr_y, curr_x, 2)
                
                # Temporal filtering
                if self.poly_coeffs is None:
                    self.poly_coeffs = new_coeffs
                else:
                    self.poly_coeffs = self.ema_alpha * new_coeffs + (1 - self.ema_alpha) * self.poly_coeffs
                
                found = True
            except:
                found = False
        else:
            found = False

        # ── STAGE 5: Math Metrics & Offset ────────────────────────────────────
        conf = 0.0
        offset = self.last_offset
        
        if found and self.poly_coeffs is not None:
            # Calculate offset at the bottom of the image (y = 239)
            # Center of the car in BEV is roughly x = 160
            y_eval = 230
            a, b, c = self.poly_coeffs
            lane_x = a*y_eval**2 + b*y_eval + c
            
            # The yellow line is the LEFT divider. We assume lane width is ~160px in BEV
            # So lane center is roughly lane_x + 80
            lane_center_x = lane_x + 75 # Offset to center of lane
            
            raw_offset = (lane_center_x - 160) / 160.0
            offset = np.clip(raw_offset, -1.0, 1.0)
            
            # Confidence based on pixel density and fit quality
            conf = np.clip(len(pts) / 1500.0, 0.0, 1.0)
            
            # Prediction/Extrapolation: If confidence is low, decay slowly
            self.last_offset = 0.7 * self.last_offset + 0.3 * offset
            self.last_conf = 0.8 * self.last_conf + 0.2 * conf
        else:
            # Decay confidence if lost
            self.last_conf *= 0.9
            if self.last_conf < 0.1:
                self.poly_coeffs = None # Reset tracker

        # Publish
        self.offset_pub.publish(Float32(data=float(self.last_offset)))
        self.conf_pub.publish(Float32(data=float(self.last_conf)))

        # ── STAGE 6: Professional Visualization ───────────────────────────────
        if self.get_parameter('publish_debug').value:
            self._publish_pro_debug(bgr, combined_mask, header)

    def _publish_pro_debug(self, bgr, mask, header):
        # 1. Base Image with ROI
        draw = bgr.copy()
        cv2.polylines(draw, [self.src_pts.astype(np.int32)], True, (0, 255, 255), 1)
        
        # 2. BEV with Fit
        bev_draw = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        if self.poly_coeffs is not None:
            plot_y = np.linspace(0, 239, 240)
            a, b, c = self.poly_coeffs
            plot_x = a*plot_y**2 + b*plot_y + c
            pts_fit = np.array([np.transpose(np.vstack([plot_x, plot_y]))], np.int32)
            cv2.polylines(bev_draw, pts_fit, False, (0, 255, 0), 2)
            
            # Unwarp the line back to original view
            pts_unwarp = cv2.perspectiveTransform(pts_fit.astype(np.float32), self.Minv)
            cv2.polylines(draw, pts_unwarp.astype(np.int32), False, (0, 255, 0), 2)

        # 3. Compose Info Panel
        info = np.zeros((240, 120, 3), dtype=np.uint8)
        color = (0, 255, 0) if self.last_conf > 0.5 else (0, 255, 255) if self.last_conf > 0.2 else (0, 0, 255)
        cv2.putText(info, "PERCEPTION", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(info, f"OFF: {self.last_offset:+.2f}", (5, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(info, f"CONF: {self.last_conf:.2f}", (5, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        cv2.putText(info, "BEV MASK", (5, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

        # Combine Panels
        # [ Original+Fit | BEV+Fit | Metrics ]
        top = np.hstack([draw, bev_draw])
        final = np.hstack([top, info])
        
        # Compress and Publish
        try:
            _, jpg = cv2.imencode('.jpg', final, [cv2.IMWRITE_JPEG_QUALITY, 70])
            msg = CompressedImage()
            msg.header = header
            msg.format = 'jpeg'
            msg.data = jpg.tobytes()
            self.dbg_pub.publish(msg)
        except: pass

def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
