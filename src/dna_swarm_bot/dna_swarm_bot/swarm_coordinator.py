#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Int32MultiArray
import math
import time

class SwarmCoordinator(Node):
    def __init__(self):
        super().__init__('swarm_coordinator')

        self.declare_parameter('bot_id', 0)
        self.declare_parameter('num_bots', 2)
        self.declare_parameter('base_speed', 0.2)
        self.declare_parameter('formation_spacing', 0.30)
        self.declare_parameter('max_linear_speed', 0.4)
        self.declare_parameter('max_angular_speed', 1.5)
        self.declare_parameter('safety_distance', 0.25)
        self.declare_parameter('wall_stop_distance', 0.50)
        self.declare_parameter('overtake_slow_speed', 0.05)
        self.declare_parameter('overtake_fast_speed', 0.35)
        self.declare_parameter('overtake_side', 1.0)
        self.declare_parameter('overtake_curve_duration', 1.5)
        self.declare_parameter('overtake_merge_duration', 1.5)
        self.declare_parameter('Kp_spacing', 1.5)

        self.bot_id = self.get_parameter('bot_id').value
        self.num_bots = self.get_parameter('num_bots').value
        self.base_speed = self.get_parameter('base_speed').value
        self.spacing = self.get_parameter('formation_spacing').value
        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.safe_dist = self.get_parameter('safety_distance').value
        self.wall_dist = self.get_parameter('wall_stop_distance').value
        self.overtake_slow = self.get_parameter('overtake_slow_speed').value
        self.overtake_fast = self.get_parameter('overtake_fast_speed').value
        self.overtake_side = self.get_parameter('overtake_side').value
        self.curve_dur = self.get_parameter('overtake_curve_duration').value
        self.merge_dur = self.get_parameter('overtake_merge_duration').value
        self.Kp = self.get_parameter('Kp_spacing').value

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'bot_pose', 10)

        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(PoseArray, 'detected_bots', self.detection_cb, 10)
        self.create_subscription(PoseArray, 'detected_walls', self.wall_cb, 10)
        self.create_subscription(Empty, '/overtake_trigger', self.overtake_trigger_cb, 10)
        self.create_subscription(Empty, '/resume_trigger', self.resume_trigger_cb, 10)

        self.other_poses = {}
        for i in range(self.num_bots):
            if i != self.bot_id:
                topic = f'/bot_{i}/bot_pose'
                self.create_subscription(PoseStamped, topic, lambda msg, bid=i: self.other_pose_cb(msg, bid), 10)

        self.state = 'FORMATION'
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.vx = 0.0
        self.overtake_start_time = None
        self.wall_detected = False
        self.wall_distance = float('inf')
        self.detected_bots_cam = []

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Bot {self.bot_id} coordinator ready. State: FORMATION")

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)
        self.vx = msg.twist.twist.linear.x

    def other_pose_cb(self, msg, bot_id):
        self.other_poses[bot_id] = (msg.pose.position.x, msg.pose.position.y)

    def detection_cb(self, msg):
        self.detected_bots_cam = [p.position.z for p in msg.poses]

    def wall_cb(self, msg):
        min_z = float('inf')
        for p in msg.poses:
            if p.position.z < min_z:
                min_z = p.position.z
        self.wall_distance = min_z
        if min_z < self.wall_dist:
            if not self.wall_detected:
                self.get_logger().warn(f"WALL DETECTED at {min_z:.2f}m! Stopping.")
            self.wall_detected = True
        else:
            self.wall_detected = False

    def overtake_trigger_cb(self, msg):
        if self.state == 'FORMATION':
            self.get_logger().info("OVERTAKE TRIGGERED!")
            self.state = 'OVERTAKE_CURVE'
            self.overtake_start_time = time.time()
        else:
            self.get_logger().warn("Overtake trigger ignored - not in FORMATION mode.")

    def resume_trigger_cb(self, msg):
        if self.state == 'WALL_STOP':
            self.get_logger().info("Resuming from wall stop.")
            self.state = 'FORMATION'

    def get_other_bot_position(self):
        for bid, pos in self.other_poses.items():
            return pos
        return None

    def determine_role(self):
        other = self.get_other_bot_position()
        if other is None:
            if any(d > 0.1 for d in self.detected_bots_cam):
                return 'BACK'
            return 'FRONT'
        other_x = other[0]
        if self.x > other_x + 0.05:
            return 'FRONT'
        elif self.x < other_x - 0.05:
            return 'BACK'
        return 'UNKNOWN'

    def formation_control(self):
        role = self.determine_role()
        v = self.base_speed
        w = 0.0

        if role == 'BACK':
            other = self.get_other_bot_position()
            if other is not None:
                dist = other[0] - self.x
                error = dist - self.spacing
                v = self.base_speed + self.Kp * error
                v = max(0.0, min(self.max_v, v))
            else:
                if self.detected_bots_cam:
                    closest = min(self.detected_bots_cam)
                    if closest < self.spacing:
                        v = self.base_speed * 0.5
        return v, w, role

    def control_loop(self):
        now = time.time()
        cmd = Twist()

        if self.wall_detected:
            self.state = 'WALL_STOP'

        if self.state == 'WALL_STOP':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'FORMATION':
            v, w, role = self.formation_control()
            cmd.linear.x = v
            cmd.angular.z = w

        elif self.state == 'OVERTAKE_CURVE':
            elapsed = now - self.overtake_start_time
            role = self.determine_role()
            if role == 'FRONT':
                cmd.linear.x = self.overtake_slow
                cmd.angular.z = 0.0
            else:
                if elapsed < self.curve_dur:
                    cmd.linear.x = 0.25
                    cmd.angular.z = 0.8 * self.overtake_side
                else:
                    self.state = 'OVERTAKE_PASS'
                    self.overtake_start_time = now
                    cmd.linear.x = self.overtake_fast
                    cmd.angular.z = 0.0

        elif self.state == 'OVERTAKE_PASS':
            role = self.determine_role()
            if role == 'FRONT':
                cmd.linear.x = self.overtake_slow
                cmd.angular.z = 0.0
            else:
                other = self.get_other_bot_position()
                if other is not None:
                    if self.x > other[0] + self.spacing + 0.05:
                        self.state = 'OVERTAKE_MERGE'
                        self.overtake_start_time = now
                        cmd.linear.x = 0.25
                        cmd.angular.z = -0.8 * self.overtake_side
                    else:
                        cmd.linear.x = self.overtake_fast
                        cmd.angular.z = 0.0
                else:
                    elapsed = now - self.overtake_start_time
                    if elapsed > 3.0:
                        self.state = 'OVERTAKE_MERGE'
                        self.overtake_start_time = now
                        cmd.linear.x = 0.25
                        cmd.angular.z = -0.8 * self.overtake_side
                    else:
                        cmd.linear.x = self.overtake_fast
                        cmd.angular.z = 0.0

        elif self.state == 'OVERTAKE_MERGE':
            elapsed = now - self.overtake_start_time
            if elapsed < self.merge_dur:
                cmd.linear.x = 0.25
                cmd.angular.z = -0.8 * self.overtake_side
            else:
                self.get_logger().info("Overtake complete. Returning to FORMATION.")
                self.state = 'FORMATION'
                cmd.linear.x = self.base_speed
                cmd.angular.z = 0.0

        cmd.linear.x = max(-self.max_v, min(self.max_v, cmd.linear.x))
        cmd.angular.z = max(-self.max_w, min(self.max_w, cmd.angular.z))
        self.cmd_pub.publish(cmd)

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'odom'
        ps.pose.position.x = self.x
        ps.pose.position.y = self.y
        ps.pose.orientation.z = math.sin(self.yaw/2)
        ps.pose.orientation.w = math.cos(self.yaw/2)
        self.pose_pub.publish(ps)

def main(args=None):
    rclpy.init(args=args)
    node = SwarmCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
