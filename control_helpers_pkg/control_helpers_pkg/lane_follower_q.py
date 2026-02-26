import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from qcar2_interfaces.msg import MotorCommands
import math

class LaneFollowerQ(Node):
    def __init__(self):
        super().__init__('lane_follower_q')
        
        self.subscription = self.create_subscription(
            Float32MultiArray, '/lane_lines', self.lines_callback, 10)
            
        self.cmd_pub = self.create_publisher(
            MotorCommands, '/qcar2_motor_speed_cmd', 10) # Cambiar a /cmd_ackermann si tu base lo requiere
            
        # Parámetros físicos e imagen
        self.image_width = 409.0  # Ajusta esto a la resolución de tu cámara
        self.image_height = 203.0
        self.wheelbase = 0.256      # L: Distancia entre ejes en metros
        self.look_ahead_dist = 0.56 # Ld: Distancia de look-ahead simulada

    def lines_callback(self, msg):
        data = msg.data
        izq_detectado = data[0] != -1.0
        der_detectado = data[4] != -1.0
        
        cmd = MotorCommands()

        # Si perdemos ambos carriles, nos detenemos por seguridad
        if not izq_detectado and not der_detectado:
            self.get_logger().warn("Carriles perdidos. Deteniendo el vehículo.")
            cmd.motor_names = ["steering_angle", "motor_throttle"]
            cmd.values = [0.0, 0.0]
            self.cmd_pub.publish(cmd)
            return

        # 1. Calcular el punto objetivo (Target) en el horizonte de la imagen
        # Usamos x2 (el punto superior de la línea extrapolada)
        target_x = self.image_width / 2.0  # Por defecto al centro
        
        if izq_detectado and der_detectado:
            # Centro exacto entre ambos carriles
            target_x = (data[2] + data[6]) / 2.0
        elif izq_detectado:
            # Si solo vemos el izquierdo, estimamos el centro sumando el ancho del carril en píxeles
            target_x = data[2] + 50.0 # Ajustar offset según la imagen
        elif der_detectado:
            target_x = data[6] - 50.0 # Ajustar offset según la imagen

        # 2. Transformar a coordenadas relativas al vehículo
        # Centro de la imagen es 0. Izquierda es negativo, derecha es positivo.
        error_x = target_x - (self.image_width / 2.0)
        
        # 3. Calcular Alpha (Ángulo de Pure Pursuit)
        # Convertimos el error en píxeles a un ángulo aproximado
        # math.atan2(opuesto, adyacente). El adyacente es la altura de la ROI.
        altura_roi = self.image_height * 0.4 # La línea va de Y=1.0 a Y=0.6 (40% de altura)
        alpha = math.atan2(error_x, altura_roi)
        
        # 4. Fórmula de Pure Pursuit: delta = atan(2 * L * sin(alpha) / Ld)
        steering_angle = math.atan2(2.0 * self.wheelbase * math.sin(alpha), self.look_ahead_dist)
        
        # Limitar ángulo de dirección (ej: max +/- 0.5 radianes)
        steering_angle = max(min(steering_angle, 0.5), -0.5)

        # 5. Control Dinámico de Velocidad
        # Si el ángulo es muy cerrado, bajamos la velocidad
        if abs(steering_angle) > 0.2:
            speed = 0.0775  # Curva
        else:
            speed = 0.15  # Recta

        # 6. Publicar Comandos
        # cmd.linear.x = speed
        # cmd.angular.z = -steering_angle # Invertir si el giro sale al revés en tu robot
        cmd.motor_names = ["steering_angle", "motor_throttle"]
        cmd.values = [-steering_angle, speed]
        
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    lane_follower_q = LaneFollowerQ()
    rclpy.spin(lane_follower_q)
    lane_follower_q.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()