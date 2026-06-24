#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_separation', 0.17)
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('max_pwm', 255)
        self.declare_parameter('max_linear_speed', 0.5)

        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baudrate').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.wheel_r = self.get_parameter('wheel_radius').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.max_lin = self.get_parameter('max_linear_speed').value

        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except Exception as e:
            self.get_logger().error(f"Serial failed: {e}")
            self.ser = None

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.timer = self.create_timer(0.1, self.watchdog)
        self.last_cmd_time = self.get_clock().now()

    def cmd_vel_cb(self, msg):
        self.last_cmd_time = self.get_clock().now()
        if self.ser is None or not self.ser.is_open:
            return

        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (w * self.wheel_sep / 2.0)
        v_right = v + (w * self.wheel_sep / 2.0)

        def vel_to_pwm(vel):
            pwm = int((vel / self.max_lin) * self.max_pwm)
            return max(-self.max_pwm, min(self.max_pwm, pwm))

        pwm_left = vel_to_pwm(v_left)
        pwm_right = vel_to_pwm(v_right)

        cmd = f"M {pwm_left} {pwm_right}\n"
        try:
            self.ser.write(cmd.encode())
        except Exception as e:
            self.get_logger().warn(f"Serial write error: {e}")

    def watchdog(self):
        now = self.get_clock().now()
        if (now - self.last_cmd_time).nanoseconds > 5e8:
            if self.ser and self.ser.is_open:
                try:
                    self.ser.write(b"M 0 0\n")
                except Exception:
                    pass

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
