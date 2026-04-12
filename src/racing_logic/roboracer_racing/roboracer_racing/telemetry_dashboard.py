import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

class TelemetryDashboard(Node):
    def __init__(self):
        super().__init__('telemetry_dashboard')
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/viz/telemetry',
            self.telemetry_cb,
            10
        )
        self.max_pts = 100
        # Deques para almacenar los datos
        self.times = deque(maxlen=self.max_pts)
        self.v_target = deque(maxlen=self.max_pts)
        self.v_actual = deque(maxlen=self.max_pts)
        self.steering = deque(maxlen=self.max_pts)
        self.dist_error = deque(maxlen=self.max_pts)
        self.start_time = None

        self.get_logger().info("📊 Telemetry Dashboard iniciando...")

    def telemetry_cb(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        t = (self.get_clock().now().nanoseconds / 1e9) - self.start_time
        
        # [Vel_Target, Vel_Actual, Steering, Error_Distance]
        data = msg.data
        if len(data) == 4:
            self.times.append(t)
            self.v_target.append(data[0])
            self.v_actual.append(data[1])
            self.steering.append(data[2])
            self.dist_error.append(data[3])

def main(args=None):
    rclpy.init(args=args)
    node = TelemetryDashboard()

    # ROS en background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # -- MATPLOTLIB SETUP --
    fig, (ax_v, ax_s, ax_e) = plt.subplots(3, 1, figsize=(8, 9))
    fig.canvas.manager.set_window_title('RoboRacer Telemetry Pro')
    fig.patch.set_facecolor('#1a1a2e')
    
    # Colores oscuros y pro
    for ax in [ax_v, ax_s, ax_e]:
        ax.set_facecolor('#16213e')
        ax.tick_params(colors='white')
        ax.grid(True, color='#333333', linewidth=0.5)
        for spine in ax.spines.values():
            spine.set_color('#555555')

    # --- VELOCITY PLOT ---
    ax_v.axhspan(0.0, 1.5, color='#00ff88', alpha=0.05) # Safe Speed
    ax_v.axhspan(1.5, 2.0, color='#ff4444', alpha=0.1)  # High Speed Danger
    line_vt, = ax_v.plot([], [], '--', color='#aaaaaa', label='V Target', lw=2)
    line_va, = ax_v.plot([], [], '-', color='#00ff88', label='V Actual', lw=2.5)
    ax_v.set_ylabel('Velocity (m/s)', color='white')
    ax_v.legend(loc='lower left', facecolor='#16213e', edgecolor='none', labelcolor='white')
    ax_v.set_ylim(-0.1, 2.0)
    text_v = ax_v.text(0.02, 0.90, '', transform=ax_v.transAxes, color='white', 
                       fontsize=12, fontweight='bold', bbox=dict(facecolor='#000000', alpha=0.5, edgecolor='none'))

    # --- STEERING PLOT ---
    ax_s.axhspan(-0.5, 0.5, color='#00ff88', alpha=0.05) # Normal Steering
    ax_s.axhspan(0.5, 0.6, color='#ff4444', alpha=0.15)  # Hard Left
    ax_s.axhspan(-0.6, -0.5, color='#ff4444', alpha=0.15) # Hard Right
    line_s, = ax_s.plot([], [], '-', color='#ffaa00', label='Steering (rad)', lw=2.5)
    ax_s.set_ylabel('Steering', color='white')
    ax_s.legend(loc='lower left', facecolor='#16213e', edgecolor='none', labelcolor='white')
    ax_s.set_ylim(-0.6, 0.6)
    text_s = ax_s.text(0.02, 0.90, '', transform=ax_s.transAxes, color='white', 
                       fontsize=12, fontweight='bold', bbox=dict(facecolor='#000000', alpha=0.5, edgecolor='none'))

    # --- ERROR PLOT ---
    ax_e.axhspan(0.0, 0.5, color='#00ff88', alpha=0.08) # Arrival Radius (Success)
    line_e, = ax_e.plot([], [], '-', color='#ff4444', label='Dist. to Goal', lw=2.5)
    ax_e.set_ylabel('Error Dist (m)', color='white')
    ax_e.set_xlabel('Time (s)', color='white')
    ax_e.legend(loc='lower left', facecolor='#16213e', edgecolor='none', labelcolor='white')
    ax_e.set_ylim(0, 5)
    text_e = ax_e.text(0.02, 0.90, '', transform=ax_e.transAxes, color='white', 
                       fontsize=12, fontweight='bold', bbox=dict(facecolor='#000000', alpha=0.5, edgecolor='none'))

    def update(frame):
        if not node.times: return line_vt, line_va, line_s, line_e, text_v, text_s, text_e
        
        t = list(node.times)
        
        # update arrays
        line_vt.set_data(t, list(node.v_target))
        line_va.set_data(t, list(node.v_actual))
        line_s.set_data(t, list(node.steering))
        line_e.set_data(t, list(node.dist_error))

        # update text panels
        curr_vt = node.v_target[-1]
        curr_va = node.v_actual[-1]
        curr_s = node.steering[-1]
        curr_e = node.dist_error[-1]

        v_warn = "⚠️ DANGER" if curr_va > 1.5 else "✅ SAFE"
        text_v.set_text(f"ACTUAL: {curr_va:.2f} m/s | TARGET: {curr_vt:.2f} m/s [{v_warn}]")

        s_warn = "⚠️ HARD TURN" if abs(curr_s) >= 0.5 else "✅ STABLE"
        text_s.set_text(f"STEER: {curr_s:+.2f} rad [{s_warn}]")

        e_warn = "🎯 ARRIVAL ZONE" if curr_e <= 0.5 else "🏁 RACING"
        text_e.set_text(f"ERR DIST: {curr_e:.2f} m [{e_warn}]")

        # Scrolling x-axis
        min_x = max(0, t[-1] - 10) # 10 seconds trailing window
        max_x = max(10, t[-1])
        
        for ax in [ax_v, ax_s, ax_e]:
            ax.set_xlim(min_x, max_x)
            
        # Ajuste dinámico del error Y
        max_e = max(node.dist_error) if node.dist_error else 1.0
        ax_e.set_ylim(0, max((max_e * 1.1), 1.0))
        
        return line_vt, line_va, line_s, line_e, text_v, text_s, text_e

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # No disparamos rclpy.shutdown() directamente aquí porque
        # puede causar errores de "already shutdown", o lo hacemos de forma segura:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
