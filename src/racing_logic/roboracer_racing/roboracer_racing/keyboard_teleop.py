"""
Keyboard Teleop — Maneja el QCar con el teclado.

Controles:
    W / ↑  : Acelerar
    S / ↓  : Reversa / Frenar
    A / ←  : Girar izquierda
    D / →  : Girar derecha
    SPACE  : Freno de emergencia
    Q      : Salir

    E / R  : Subir/Bajar velocidad máxima
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
import sys
import termios
import tty
import select

KEYS_HELP = """
╔══════════════════════════════════╗
║   🏎️  ROBORACER KEYBOARD TELEOP  ║
╠══════════════════════════════════╣
║                                  ║
║          W / ↑  = Avanzar        ║
║          S / ↓  = Reversa        ║
║          A / ←  = Izquierda      ║
║          D / →  = Derecha        ║
║         SPACE   = FRENO          ║
║          E / R  = +/- vel max    ║
║          Q      = Salir          ║
║                                  ║
╚══════════════════════════════════╝
"""

# Teclas especiales (flechas)
ARROW_PREFIX = '\x1b'

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_topic', '/qcar_sim/user_command')
        self.declare_parameter('max_speed', 3.0)
        self.declare_parameter('max_steer', 0.45)
        self.declare_parameter('accel_step', 0.3)
        self.declare_parameter('steer_step', 0.12)

        cmd_topic = self.get_parameter('cmd_topic').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steer = self.get_parameter('max_steer').value
        self.accel_step = self.get_parameter('accel_step').value
        self.steer_step = self.get_parameter('steer_step').value

        self.speed = 0.0
        self.steer = 0.0

        self.cmd_pub = self.create_publisher(Vector3Stamped, cmd_topic, 10)

        # Publish at 50Hz
        self.timer = self.create_timer(0.02, self.publish_cmd)

        self.get_logger().info(KEYS_HELP)
        self.get_logger().info(f'Max speed: {self.max_speed} m/s | Max steer: {self.max_steer} rad')

    def publish_cmd(self):
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = float(self.speed)
        cmd.vector.y = float(self.steer)
        self.cmd_pub.publish(cmd)

    def process_key(self, key):
        """Procesa una tecla y actualiza velocidad/dirección."""
        if key in ('w', 'W', '\x1b[A'):  # W o flecha arriba
            self.speed = min(self.speed + self.accel_step, self.max_speed)
        elif key in ('s', 'S', '\x1b[B'):  # S o flecha abajo
            self.speed = max(self.speed - self.accel_step, -self.max_speed * 0.5)
        elif key in ('a', 'A', '\x1b[D'):  # A o flecha izquierda
            self.steer = min(self.steer + self.steer_step, self.max_steer)
        elif key in ('d', 'D', '\x1b[C'):  # D o flecha derecha
            self.steer = max(self.steer - self.steer_step, -self.max_steer)
        elif key == ' ':  # Espacio = freno
            self.speed = 0.0
            self.steer = 0.0
        elif key in ('e', 'E'):
            self.max_speed = min(self.max_speed + 0.2, 4.0)
            self.get_logger().info(f'⬆️ Max speed: {self.max_speed:.1f} m/s')
        elif key in ('r', 'R'):
            self.max_speed = max(self.max_speed - 0.2, 0.2)
            self.get_logger().info(f'⬇️ Max speed: {self.max_speed:.1f} m/s')
        elif key in ('q', 'Q'):
            return False  # Señal de salida

        # Auto-centrar el steering gradualmente si no se toca A/D
        if key not in ('a', 'A', 'd', 'D', '\x1b[D', '\x1b[C'):
            if abs(self.steer) > 0.01:
                self.steer *= 0.85  # Retorno suave al centro
            else:
                self.steer = 0.0

        # Mostrar estado
        bar_len = 20
        speed_bar = int(abs(self.speed) / self.max_speed * bar_len)
        steer_pos = int((self.steer + self.max_steer) / (2 * self.max_steer) * bar_len)
        direction = '▶' if self.speed >= 0 else '◀'

        print(f'\r  {direction} vel: {self.speed:+.2f} [{"█" * speed_bar}{"░" * (bar_len - speed_bar)}]'
              f'  steer: {self.steer:+.2f} [{"░" * steer_pos}{"█"}{"░" * (bar_len - steer_pos)}]  ',
              end='', flush=True)
        return True


def get_key(timeout=0.05):
    """Lee una tecla del terminal sin bloquear."""
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            # Detectar secuencias de escape (flechas)
            if key == ARROW_PREFIX:
                key += sys.stdin.read(2)
            return key
        return None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        while rclpy.ok():
            key = get_key(timeout=0.02)
            if key is not None:
                if not node.process_key(key):
                    break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        # Parar el carro
        node.speed = 0.0
        node.steer = 0.0
        node.publish_cmd()
        print('\n\n🛑 Teleop stopped. Car stopped.')
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
