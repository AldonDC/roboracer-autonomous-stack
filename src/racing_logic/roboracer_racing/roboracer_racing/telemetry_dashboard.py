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

    # -- MATLAB ENGINEERING AESTHETIC SETUP --
    # Usar un estilo blanco, limpio con grids (típico de ingeniería/MATLAB)
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, (ax_v, ax_s, ax_e) = plt.subplots(3, 1, figsize=(9, 10))
    fig.canvas.manager.set_window_title('RoboRacer Engineering Telemetry')
    fig.patch.set_facecolor('#F0F2F6') # Gris muy claro fondo
    fig.suptitle('Control System Real-Time Telemetry', fontsize=16, fontweight='bold', color='#333333')

    # Configuración base para todos los ejes
    for ax in [ax_v, ax_s, ax_e]:
        ax.set_facecolor('#FFFFFF')
        ax.grid(True, which='major', color='#D0D0D0', linestyle='-', linewidth=1.0)
        ax.grid(True, which='minor', color='#E0E0E0', linestyle='--', linewidth=0.5)
        ax.minorticks_on()
        # Línea gruesa en el marco
        for spine in ax.spines.values():
            spine.set_color('#888888')
            spine.set_linewidth(1.5)

    # --- VELOCITY PLOT (System Response) ---
    ax_v.set_title('Plant Response: Longitudinal Velocity', fontsize=12, fontweight='bold', loc='left')
    ax_v.axhline(0, color='black', linewidth=1.5)
    
    line_vt, = ax_v.plot([], [], '--', color='tab:red', label='Target Reference ($v_{ref}$)', lw=2)
    line_va, = ax_v.plot([], [], '-', color='tab:blue', label='Actual Velocity ($v$)', lw=2.5)
    
    # Text box ingeniero
    text_v = ax_v.text(0.98, 0.90, '', transform=ax_v.transAxes, color='black', 
                       fontsize=10, ha='right', va='top', bbox=dict(facecolor='white', alpha=0.9, edgecolor='#CCCCCC', boxstyle='round,pad=0.5'))
    
    ax_v.set_ylabel('Velocity [m/s]', fontweight='bold')
    ax_v.legend(loc='lower left')
    ax_v.set_ylim(-0.1, 2.0)

    # --- STEERING PLOT (Control Input) ---
    ax_s.set_title('Control Effort: Ackermann Steering Limit', fontsize=12, fontweight='bold', loc='left')
    ax_s.axhline(0, color='black', linewidth=1.5)
    
    # Zonas de saturación (rojas) del servo
    ax_s.axhspan(0.5, 0.6, color='tab:red', alpha=0.1)
    ax_s.axhspan(-0.6, -0.5, color='tab:red', alpha=0.1)
    
    line_s, = ax_s.plot([], [], '-', color='tab:orange', label='Steering Angle ($\delta$)', lw=2.5)
    
    text_s = ax_s.text(0.98, 0.90, '', transform=ax_s.transAxes, color='black', 
                       fontsize=10, ha='right', va='top', bbox=dict(facecolor='white', alpha=0.9, edgecolor='#CCCCCC', boxstyle='round,pad=0.5'))
                       
    ax_s.set_ylabel('Angle [rad]', fontweight='bold')
    ax_s.set_ylim(-0.6, 0.6)

    # --- ERROR PLOT (Tracking Performance) ---
    ax_e.set_title('Tracking Performance: Dist. to Waypoint', fontsize=12, fontweight='bold', loc='left')
    ax_e.axhline(0, color='black', linewidth=1.5)
    
    # Sombreado de tolerancia (banda verde de llegada)
    ax_e.axhspan(0.0, 0.5, color='tab:green', alpha=0.15)
    
    line_e, = ax_e.plot([], [], '-', color='tab:purple', label='Distance Error ($e$)', lw=2.5)
    
    text_e = ax_e.text(0.98, 0.90, '', transform=ax_e.transAxes, color='black', 
                       fontsize=10, ha='right', va='top', bbox=dict(facecolor='white', alpha=0.9, edgecolor='#CCCCCC', boxstyle='round,pad=0.5'))

    ax_e.set_ylabel('Error [m]', fontweight='bold')
    ax_e.set_xlabel('Time [s]', fontweight='bold')
    ax_e.set_ylim(0, 5)

    def update(frame):
        if not node.times: return line_vt, line_va, line_s, line_e, text_v, text_s, text_e
        
        t = list(node.times)
        
        line_vt.set_data(t, list(node.v_target))
        line_va.set_data(t, list(node.v_actual))
        line_s.set_data(t, list(node.steering))
        line_e.set_data(t, list(node.dist_error))

        # Rellenar (Fill Between) el error de velocidad en tiempo real
        # Quitamos la colección previa para no sobrecargar RAM
        for collection in ax_v.collections:
            collection.remove()
        ax_v.fill_between(t, list(node.v_actual), list(node.v_target), color='tab:gray', alpha=0.3)

        curr_vt = node.v_target[-1]
        curr_va = node.v_actual[-1]
        curr_s = node.steering[-1]
        curr_e = node.dist_error[-1]

        # Formato Ingenieril tipo consola
        text_v.set_text(f"v_ref: {curr_vt:.2f} m/s\nv_act: {curr_va:.2f} m/s\nError: {abs(curr_vt-curr_va):.2f} m/s")
        
        status_steer = "SATURATED" if abs(curr_s) >= 0.5 else "OPERATIONAL"
        text_s.set_text(f"δ: {curr_s:+.3f} rad\nState: {status_steer}")

        status_err = "GOAL REACHED (<0.5)" if curr_e <= 0.5 else "TRACKING"
        text_e.set_text(f"e(t): {curr_e:.2f} m\nPhase: {status_err}")

        # Scrolling x-axis
        min_x = max(0, t[-1] - 10)
        max_x = max(10, t[-1])
        
        for ax in [ax_v, ax_s, ax_e]:
            ax.set_xlim(min_x, max_x)
            
        max_e = max(node.dist_error) if node.dist_error else 1.0
        ax_e.set_ylim(0, max((max_e * 1.1), 1.0))
        
        return line_vt, line_va, line_s, line_e, text_v, text_s, text_e

    ani = animation.FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)

    plt.tight_layout()
    # Ajustar para no tapar el suptitle
    fig.subplots_adjust(top=0.92)
    
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
