#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros
import math
import time

try:
    import smbus2
except ImportError:
    smbus2 = None

class MPU6050:
    ADDR = 0x68
    REG_PWR_MGMT_1 = 0x6B
    REG_GYRO_ZOUT_H = 0x47
    REG_ACCEL_XOUT_H = 0x3B

    def __init__(self, bus=1):
        if smbus2 is None:
            raise RuntimeError("smbus2 not installed. Run: pip3 install smbus2")
        self.bus = smbus2.SMBus(bus)
        self.bus.write_byte_data(self.ADDR, self.REG_PWR_MGMT_1, 0)
        time.sleep(0.1)
        self.gyro_z_bias = 0.0
        self._calibrate()

    def _calibrate(self, samples=100):
        s = 0
        for _ in range(samples):
            s += self._read_word(self.REG_GYRO_ZOUT_H)
            time.sleep(0.005)
        self.gyro_z_bias = s / samples / 131.0

    def _read_word(self, reg):
        high = self.bus.read_byte_data(self.ADDR, reg)
        low = self.bus.read_byte_data(self.ADDR, reg+1)
        val = (high << 8) + low
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val

    def read(self):
        accel_x = self._read_word(self.REG_ACCEL_XOUT_H) / 16384.0
        accel_y = self._read_word(self.REG_ACCEL_XOUT_H + 2) / 16384.0
        accel_z = self._read_word(self.REG_ACCEL_XOUT_H + 4) / 16384.0
        gyro_z = (self._read_word(self.REG_GYRO_ZOUT_H) / 131.0) - self.gyro_z_bias
        return accel_x, accel_y, accel_z, gyro_z

class IMUOdometry(Node):
    def __init__(self):
        super().__init__('imu_odometry')
        self.declare_parameter('bus', 1)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('child_frame_id', 'base_footprint')
        self.declare_parameter('frame_id', 'odom')

        self.imu = MPU6050(self.get_parameter('bus').value)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self.vel_cb, 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.last_time = time.time()

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0/rate, self.update)

    def vel_cb(self, msg):
        self.vx = msg.linear.x

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        ax, ay, az, gz = self.imu.read()
        gz_rad = math.radians(gz)

        self.yaw += gz_rad * dt
        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.angular_velocity.z = gz_rad
        imu_msg.linear_acceleration.x = ax * 9.81
        imu_msg.linear_acceleration.y = ay * 9.81
        imu_msg.linear_acceleration.z = az * 9.81
        self.imu_pub.publish(imu_msg)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.get_parameter('frame_id').value
        odom.child_frame_id = self.get_parameter('child_frame_id').value
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw/2)
        odom.pose.pose.orientation.w = math.cos(self.yaw/2)
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = gz_rad
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = IMUOdometry()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
