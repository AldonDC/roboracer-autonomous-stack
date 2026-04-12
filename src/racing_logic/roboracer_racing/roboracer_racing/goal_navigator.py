"""
Goal Navigator — Click en RViz y el carro va hacia ese punto.

En RViz:
  1. Click el botón "2D Goal Pose" (flecha verde arriba)
  2. Click en el mapa donde quieres que vaya el carro
  3. El carro se mueve solo hacia ese punto

Repite para ir a otro punto.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
import numpy as np


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GoalNavigator(Node):
    def __init__(self):
        super().__init__('goal_navigator')

        # Parámetros
        self.declare_parameter('v_ref', 1.0)
        self.declare_parameter('arrival_radius', 0.25)  # metros para considerar "llegó"
        self.declare_parameter('odom_topic', '/qcar_sim/odom')
        self.declare_parameter('cmd_topic', '/qcar_sim/user_command')

        self.v_ref = self.get_parameter('v_ref').value
        self.arrival_radius = self.get_parameter('arrival_radius').value
        self.L = 0.256  # Wheelbase
        self.max_steer = 0.5

        # Estado
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.goal_x = None
        self.goal_y = None
        self.navigating = False
        self.current_v = 0.0

        # Publishers
        cmd_topic = self.get_parameter('cmd_topic').value
        self.cmd_pub = self.create_publisher(Vector3Stamped, cmd_topic, 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/viz/goal_marker', 10)

        # Subscribers
        odom_topic = self.get_parameter('odom_topic').value
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # Control loop a 50Hz
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info('🎯 GOAL NAVIGATOR LISTO')
        self.get_logger().info('   En RViz: click "2D Goal Pose" → click en el mapa')
        self.get_logger().info('   El carro irá hacia ese punto automáticamente')

    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def goal_cb(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.navigating = True
        self.current_v = 0.0

        dist = math.hypot(self.goal_x - self.current_x, self.goal_y - self.current_y)
        self.get_logger().info(
            f'🎯 NEW GOAL: ({self.goal_x:.2f}, {self.goal_y:.2f}) | '
            f'Distance: {dist:.2f}m'
        )
        self.publish_goal_marker()

    def publish_goal_marker(self):
        """Muestra el objetivo como una esfera roja en RViz."""
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'goal'
        m.id = 0
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = self.goal_x
        m.pose.position.y = self.goal_y
        m.pose.position.z = 0.1
        m.scale.x = 0.3
        m.scale.y = 0.3
        m.scale.z = 0.2
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.2, 0.2, 0.9
        self.goal_marker_pub.publish(m)

    def control_loop(self):
        if not self.navigating or self.goal_x is None:
            return

        # Distancia al objetivo
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist = math.hypot(dx, dy)

        # ¿Llegamos?
        if dist < self.arrival_radius:
            self.stop()
            self.navigating = False
            self.get_logger().info(f'✅ ARRIVED! Distance: {dist:.3f}m')
            # Borrar marker
            m = Marker()
            m.header.frame_id = 'world'
            m.ns = 'goal'
            m.id = 0
            m.action = Marker.DELETE
            self.goal_marker_pub.publish(m)
            return

        # Velocidad: rápido lejos, lento al llegar
        if dist > 1.0:
            v_target = self.v_ref
        else:
            v_target = max(0.2, self.v_ref * dist)  # Desaceleración progresiva

        # Rampa de aceleración
        if self.current_v < v_target:
            self.current_v = min(self.current_v + 0.02, v_target)
        else:
            self.current_v = max(self.current_v - 0.04, v_target)

        # Pure Pursuit hacia el goal
        alpha = normalize_angle(math.atan2(dy, dx) - self.current_yaw)

        # Si el punto está atrás, girar más agresivo
        lookahead = max(dist, 0.3)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), lookahead)
        delta = float(np.clip(delta, -self.max_steer, self.max_steer))

        # Reducir vel en curvas
        curve_factor = 1.0 - (abs(delta) / self.max_steer) * 0.5
        final_v = self.current_v * curve_factor

        # Publicar comando
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = float(final_v)
        cmd.vector.y = float(delta)
        self.cmd_pub.publish(cmd)

    def stop(self):
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = 0.0
        cmd.vector.y = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
