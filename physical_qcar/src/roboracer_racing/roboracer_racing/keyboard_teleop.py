import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSHistoryPolicy, QoSDurabilityPolicy)
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool
import sys
import termios
import tty
import select
import time

KEYS_HELP = """
╔════════════════════════════════════════════════════════╗
║          ROBORACER KEYBOARD TELEOP PRO  v4             ║
╠════════════════════════════════════════════════════════╣
║  MOVIMIENTO              AJUSTE EN VIVO                ║
║  W / Arriba  +Vel        E/R   +/- vel max             ║
║  S / Abajo   -Vel        T/Y   +/- paso accel          ║
║  A / Izq     STEER IZQ   F/G   +/- giro max            ║
║  D / Der     STEER DER   H/J   +/- paso giro           ║
║  Z           Centrar volante                           ║
║                                                        ║
║  SPACE   Freno + auto OFF                              ║
║  X       Stop total (vel=0, steer=0)                   ║
║  M       Activar modo autonomo                         ║
║  N       Toggle modo SNAP (1 toque = tope) / GRADUAL   ║
║  C       Toggle auto-center al soltar A/D              ║
║  P       Mostrar parametros                            ║
║  Q       Salir                                         ║
║                                                        ║
║  >> SNAP ON  (default): 1 toque A/D = giro al tope     ║
║  >> SNAP OFF: A/D acumulan paso a paso                 ║
╚════════════════════════════════════════════════════════╝
"""

ARROW_PREFIX = '\x1b'

# Limites absolutos de seguridad
# SPEED_ABS_MAX bajado a 0.10 para evitar accidentes por inputs externos
# (mouse/scroll que pudieran disparar acceleracion). Antes: 5.0
SPEED_ABS_MAX = 0.10
SPEED_ABS_MIN = 0.02
STEER_ABS_MAX = 0.70
ACCEL_STEP_MIN = 0.02
ACCEL_STEP_MAX = 0.5
STEER_STEP_MIN = 0.02
STEER_STEP_MAX = 0.3


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('cmd_topic',   '/qcar/user_command')
        # max_speed default bajado a 0.10 m/s — limite de seguridad para
        # evitar accidentes (antes era 0.8). El tope absoluto duro (no
        # se puede subir mas con E/R) esta en SPEED_ABS_MAX = 0.10.
        self.declare_parameter('max_speed',   0.10)
        self.declare_parameter('max_steer',   0.30)   # rango util del QCar (igual que gamepad)
        self.declare_parameter('accel_step',  0.05)
        self.declare_parameter('steer_step',  0.10)
        self.declare_parameter('snap_steer',  True)   # 1 toque = tope
        self.declare_parameter('auto_center', False)

        cmd_topic        = self.get_parameter('cmd_topic').value
        self.max_speed   = self.get_parameter('max_speed').value
        self.max_steer   = self.get_parameter('max_steer').value
        self.accel_step  = self.get_parameter('accel_step').value
        self.steer_step  = self.get_parameter('steer_step').value
        self.snap_steer  = self.get_parameter('snap_steer').value
        self.auto_center = self.get_parameter('auto_center').value

        self.speed = 0.0
        self.steer = 0.0

        # Tracking para auto-center opcional (estilo coche con resorte)
        self.last_steer_key_time = time.time()
        self.steer_return_factor = 0.85   # mas suave para que se vea

        # QoS BEST_EFFORT depth=1 para control a 50 Hz:
        # - Sin ACK ni retransmision (no bloquea bajo rafaga de teclas)
        # - Si se pierde 1 comando, otro llega en 20 ms
        # - Estandar para teleop reactivo (igual que un gamepad RC)
        cmd_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability  = QoSDurabilityPolicy.VOLATILE,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 1,
        )
        self.cmd_pub  = self.create_publisher(Vector3Stamped, cmd_topic, cmd_qos)
        # auto_pub queda RELIABLE: eventos one-shot (START/STOP) deben llegar
        self.auto_pub = self.create_publisher(Bool, '/lane/enable_auto', 10)

        # Subscribe a /lane/auto_active para saber si hay otro nodo controlando.
        # Cuando autonomo esta activo, el teleop NO publica al /qcar/user_command
        # para no competir con el lane_follower. Solo el lane_follower controla.
        # Las teclas SPACE/X/M/Q SIGUEN funcionando para emergencia y toggle.
        self.auto_active = False
        self.create_subscription(Bool, '/lane/auto_active',
                                 self._auto_active_cb, 10)

        self.timer = self.create_timer(0.02, self._tick)  # 50 Hz

        self.get_logger().info(KEYS_HELP)
        self._print_params()

    # ------------------------------------------------------------------
    def _auto_active_cb(self, msg):
        """Recibe el estado del modo autonomo del lane_follower."""
        prev = self.auto_active
        self.auto_active = bool(msg.data)
        if self.auto_active and not prev:
            self.get_logger().info('⚠️  AUTONOMO ACTIVO — teleop deja de publicar')
        elif not self.auto_active and prev:
            self.get_logger().info('✅ AUTONOMO INACTIVO — teleop retoma publicacion')

    # ------------------------------------------------------------------
    def _tick(self):
        """50 Hz: publica el comando actual + HUD. Auto-center opcional.

        IMPORTANTE: si el modo autonomo esta activo (otro nodo controla),
        este metodo NO publica al /qcar/user_command. Solo el lane_follower
        controla durante el autonomo. Esto evita que dos publishers compitan
        (carro vacilaba entre comando real y (0,0,0) del teleop inactivo).
        """
        # Si autonomo controla, no competir con sus comandos
        if self.auto_active:
            self._render_hud()
            return

        now = time.time()

        if self.auto_center and (now - self.last_steer_key_time) > 0.25:
            if abs(self.steer) > 0.01:
                self.steer *= self.steer_return_factor
            else:
                self.steer = 0.0

        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = float(self.speed)
        cmd.vector.y = float(self.steer)
        self.cmd_pub.publish(cmd)
        self._render_hud()

    def publish_cmd(self):
        cmd = Vector3Stamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.vector.x = 0.0
        cmd.vector.y = 0.0
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    def process_key(self, key):
        if key in ('w', 'W', '\x1b[A'):
            self.speed = min(self.speed + self.accel_step, self.max_speed)

        elif key in ('s', 'S', '\x1b[B'):
            self.speed = max(self.speed - self.accel_step, -self.max_speed * 0.5)

        elif key in ('a', 'A', '\x1b[D'):
            if self.snap_steer:
                # SNAP: un toque = tope izquierdo (movimiento físico maximo y visible)
                self.steer = self.max_steer
            else:
                self.steer = min(self.steer + self.steer_step, self.max_steer)
            self.last_steer_key_time = time.time()

        elif key in ('d', 'D', '\x1b[C'):
            if self.snap_steer:
                # SNAP: un toque = tope derecho
                self.steer = -self.max_steer
            else:
                self.steer = max(self.steer - self.steer_step, -self.max_steer)
            self.last_steer_key_time = time.time()

        elif key in ('z', 'Z'):
            # Centrar volante manualmente
            self.steer = 0.0

        elif key in ('x', 'X'):
            # Stop total: vel y steer en cero
            self.speed = 0.0
            self.steer = 0.0

        elif key == ' ':
            # Espacio: freno + desactivar auto
            self.speed = 0.0
            self.steer = 0.0
            self.auto_pub.publish(Bool(data=False))

        elif key in ('c', 'C'):
            # Toggle auto-center
            self.auto_center = not self.auto_center
            print(f'\n  >> auto_center = {"ON" if self.auto_center else "OFF (sticky RC)"}',
                  flush=True)

        elif key in ('n', 'N'):
            # Toggle modo SNAP / GRADUAL
            self.snap_steer = not self.snap_steer
            mode = 'SNAP (1 toque = tope)' if self.snap_steer else 'GRADUAL (paso a paso)'
            print(f'\n  >> modo steering = {mode}', flush=True)

        # --- ajuste de velocidad maxima ---
        elif key in ('e', 'E'):
            self.max_speed = min(round(self.max_speed + 0.1, 2), SPEED_ABS_MAX)
            self._log_param('max_speed', self.max_speed)
        elif key in ('r', 'R'):
            self.max_speed = max(round(self.max_speed - 0.1, 2), SPEED_ABS_MIN)
            self._log_param('max_speed', self.max_speed)

        # --- ajuste de paso de aceleracion ---
        elif key in ('t', 'T'):
            self.accel_step = min(round(self.accel_step + 0.01, 3), ACCEL_STEP_MAX)
            self._log_param('accel_step', self.accel_step)
        elif key in ('y', 'Y'):
            self.accel_step = max(round(self.accel_step - 0.01, 3), ACCEL_STEP_MIN)
            self._log_param('accel_step', self.accel_step)

        # --- ajuste de giro maximo ---
        elif key in ('f', 'F'):
            self.max_steer = min(round(self.max_steer + 0.05, 3), STEER_ABS_MAX)
            self._log_param('max_steer', self.max_steer)
        elif key in ('g', 'G'):
            self.max_steer = max(round(self.max_steer - 0.05, 3), 0.10)
            self._log_param('max_steer', self.max_steer)

        # --- ajuste de paso de giro ---
        elif key in ('h', 'H'):
            self.steer_step = min(round(self.steer_step + 0.01, 3), STEER_STEP_MAX)
            self._log_param('steer_step', self.steer_step)
        elif key in ('j', 'J'):
            self.steer_step = max(round(self.steer_step - 0.01, 3), STEER_STEP_MIN)
            self._log_param('steer_step', self.steer_step)

        elif key in ('m', 'M'):
            self.auto_pub.publish(Bool(data=True))
            self.get_logger().info('Enviando START para modo autonomo...')

        elif key in ('p', 'P'):
            print()
            self._print_params()

        elif key in ('q', 'Q'):
            return False

        return True

    # ------------------------------------------------------------------
    def _render_hud(self):
        BAR = 20

        # Si el follower controla, mostrar banner claro en vez del HUD normal.
        # NOTA: las teclas aqui son las del PANEL DEL TELEOP, no del follower.
        if self.auto_active:
            print(
                f'\r [AUTONOMO ON] El follower controla. Teclas en ESTE panel: '
                f'SPACE=stop+auto_off | M=reanudar | W/S/A/D bloqueadas   ',
                end='', flush=True,
            )
            return

        speed_ratio = abs(self.speed) / max(self.max_speed, 0.01)
        speed_fill  = min(int(speed_ratio * BAR), BAR)
        direction   = 'FWD' if self.speed > 0.0 else ('REV' if self.speed < 0.0 else 'STP')

        # Volante visual
        pct = abs(self.steer) / max(self.max_steer, 0.01)
        if pct < 0.05:
            wheel = '  ( | )  '
        elif self.steer > 0:   # IZQUIERDA
            if pct < 0.40:    wheel = ' (/   )  '
            elif pct < 0.75:  wheel = ' (/   )< '
            else:             wheel = '(//   )<<'
        else:                  # DERECHA
            if pct < 0.40:    wheel = '  (   \\) '
            elif pct < 0.75:  wheel = ' >(   \\) '
            else:             wheel = '>>(   \\\\)'

        # Barra posicional steer
        SBAR = 13
        ratio = (self.steer + self.max_steer) / max(2 * self.max_steer, 0.01)
        ratio = max(0.0, min(1.0, ratio))
        spos = max(0, min(SBAR - 1, int(ratio * SBAR)))
        sbar = ['.'] * SBAR
        sbar[SBAR // 2] = '|'
        sbar[spos] = '#'

        mode = 'SNAP' if self.snap_steer else 'GRAD'
        ac   = 'AC' if self.auto_center else '--'

        print(
            f'\r [{direction}] {self.speed:+5.2f}m/s '
            f'[{"#" * speed_fill}{"." * (BAR - speed_fill)}] '
            f'STEER {self.steer:+.3f}rad '
            f'[{"".join(sbar)}] '
            f'{wheel} '
            f'[{mode}|{ac}] max:{self.max_steer:.2f}   ',
            end='', flush=True,
        )

    def _log_param(self, name, value):
        print(f'\n  >> {name} = {value}', flush=True)

    def _print_params(self):
        print(
            f'  PARAMS  max_speed={self.max_speed:.2f}  max_steer={self.max_steer:.3f}'
            f'  accel_step={self.accel_step:.3f}  steer_step={self.steer_step:.3f}'
            f'  auto_center={self.auto_center}',
            flush=True,
        )


# ----------------------------------------------------------------------
def get_key(timeout=0.02):
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
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
            key = get_key(timeout=0.01)
            if key is not None:
                if not node.process_key(key):
                    break
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.speed = 0.0
        node.steer = 0.0
        node.publish_cmd()
        print('\n\n  Teleop stopped. Car stopped safely.')
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
