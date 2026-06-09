#!/usr/bin/env python3
"""
BNO055 IMU Publisher — Lee del puerto serial y publica en topics ROS.

Topics publicados:
  /imu/data        sensor_msgs/Imu          — estándar ROS, completo
  /imu/accel_raw   geometry_msgs/TwistStamped — aceleración + Euler

Parámetros:
  port          (string) — puerto serial, default '/dev/ttyUSB0'
  baudrate      (int)    — velocidad serial, default 115200
  frame_id      (string) — frame_id de los mensajes, default 'imu_link'
  publish_rate  (float)  — Hz del timer de lectura, default 100.0

Espera líneas JSON del IMU con campos:
  ax, ay, az (m/s²), gx, gy, gz (rad/s), roll, pitch, yaw (grados)
"""

import json
import math
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                       QoSHistoryPolicy, QoSDurabilityPolicy)
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu


def euler_to_quat(roll_rad, pitch_rad, yaw_rad):
    """Convierte angulos Euler (rad) a quaternion (x, y, z, w)."""
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class BNO055Publisher(Node):
    def __init__(self):
        super().__init__('imu_bno055_publisher')

        # ── Parametros ────────────────────────────────────────────────────────
        self.declare_parameter('port',         '/dev/ttyUSB0')
        self.declare_parameter('baudrate',     115200)
        self.declare_parameter('frame_id',     'imu_link')
        self.declare_parameter('publish_rate', 100.0)

        port      = self.get_parameter('port').get_parameter_value().string_value
        baudrate  = int(self.get_parameter('baudrate').value)
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        rate      = float(self.get_parameter('publish_rate').value)

        # ── Abrir puerto serial ───────────────────────────────────────────────
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.05)
            self.get_logger().info(f'✓ IMU conectado: {port} @ {baudrate} bps')
        except Exception as e:
            self.get_logger().error(f'✗ Error abriendo {port}: {e}')
            raise

        # ── QoS: RELIABLE depth=10 ────────────────────────────────────────────
        # IMU corre local en la Jetson (no viaja por WiFi). RELIABLE permite
        # que `ros2 topic echo` (Dashing no soporta --qos-reliability) reciba
        # mensajes para diagnostico, sin penalizar throughput a 100Hz.
        imu_qos = QoSProfile(
            reliability = QoSReliabilityPolicy.RELIABLE,
            durability  = QoSDurabilityPolicy.VOLATILE,
            history     = QoSHistoryPolicy.KEEP_LAST,
            depth       = 10,
        )

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_imu   = self.create_publisher(Imu,           '/imu/data',      imu_qos)
        self.pub_accel = self.create_publisher(TwistStamped,  '/imu/accel_raw', imu_qos)

        # ── Timer ─────────────────────────────────────────────────────────────
        self.timer = self.create_timer(1.0 / rate, self._read_and_publish)

        # Stats para log periodico
        self._msg_count    = 0
        self._error_count  = 0

        self.get_logger().info(
            f'BNO055 publisher OK — {rate:.0f} Hz | frame_id={self.frame_id}'
        )

    # ──────────────────────────────────────────────────────────────────────────
    def _read_and_publish(self):
        try:
            if self.ser.in_waiting <= 0:
                return
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                return

            data = json.loads(line)
            if 'ax' not in data:
                return

            # Convertir Euler grados → radianes una sola vez
            roll  = math.radians(data['roll'])
            pitch = math.radians(data['pitch'])
            yaw   = math.radians(data['yaw'])

            stamp = self.get_clock().now().to_msg()

            # ── TwistStamped (aceleracion + Euler en angular) ────────────────
            tw = TwistStamped()
            tw.header.stamp    = stamp
            tw.header.frame_id = self.frame_id
            tw.twist.linear.x  = float(data['ax'])
            tw.twist.linear.y  = float(data['ay'])
            tw.twist.linear.z  = float(data['az'])
            tw.twist.angular.x = roll
            tw.twist.angular.y = pitch
            tw.twist.angular.z = yaw
            self.pub_accel.publish(tw)

            # ── Imu estandar ROS ─────────────────────────────────────────────
            imu = Imu()
            imu.header.stamp    = stamp
            imu.header.frame_id = self.frame_id

            imu.linear_acceleration.x = float(data['ax'])
            imu.linear_acceleration.y = float(data['ay'])
            imu.linear_acceleration.z = float(data['az'])

            imu.angular_velocity.x = float(data.get('gx', 0.0))
            imu.angular_velocity.y = float(data.get('gy', 0.0))
            imu.angular_velocity.z = float(data.get('gz', 0.0))

            qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
            imu.orientation.x = qx
            imu.orientation.y = qy
            imu.orientation.z = qz
            imu.orientation.w = qw

            self.pub_imu.publish(imu)

            # ── Log periodico (cada ~2 seg si rate=100 Hz) ───────────────────
            self._msg_count += 1
            if self._msg_count % 200 == 0:
                self.get_logger().info(
                    f'ax={data["ax"]:+.2f} ay={data["ay"]:+.2f} az={data["az"]:+.2f} | '
                    f'yaw={data["yaw"]:+.1f}° pitch={data["pitch"]:+.1f}° roll={data["roll"]:+.1f}° | '
                    f'msgs={self._msg_count} errors={self._error_count}'
                )

        except json.JSONDecodeError:
            self._error_count += 1   # linea no-JSON, ignorada silenciosamente
        except KeyError as e:
            self.get_logger().warn(f'Campo faltante en JSON: {e}', throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().error(f'Error leyendo IMU: {e}', throttle_duration_sec=2.0)

    # ──────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = BNO055Publisher()
    except Exception as e:
        print(f'[imu_bno055] Failed to start: {e}')
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
