#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Int32MultiArray
import math

class SimDetector(Node):
    def __init__(self):
        super().__init__('sim_detector')
        self.declare_parameter('bot_id', 0)
        self.declare_parameter('num_bots', 2)
        self.declare_parameter('camera_fov', 1.047)
        self.declare_parameter('max_range', 3.0)
        self.declare_parameter('wall_marker_ids', [10, 11])
        self.declare_parameter('wall_positions', [3.0, 6.0])

        self.bot_id = self.get_parameter('bot_id').value
        self.num_bots = self.get_parameter('num_bots').value
        self.fov = self.get_parameter('camera_fov').value
        self.max_range = self.get_parameter('max_range').value
        self.wall_ids = self.get_parameter('wall_marker_ids').value
        self.wall_xs = self.get_parameter('wall_positions').value

        self.create_subscription(ModelStates, '/gazebo/model_states', self.states_cb, 10)
        self.det_pub = self.create_publisher(PoseArray, 'detected_bots', 10)
        self.wall_pub = self.create_publisher(PoseArray, 'detected_walls', 10)
        self.bot_ids_pub = self.create_publisher(Int32MultiArray, 'detected_bot_ids', 10)
        self.wall_ids_pub = self.create_publisher(Int32MultiArray, 'detected_wall_ids', 10)

        self.my_pose = None
        self.my_yaw = 0.0

    def states_cb(self, msg):
        my_name = f'dna_bot_{self.bot_id}'
        poses = {}
        my_idx = -1
        for i, name in enumerate(msg.name):
            if name == my_name:
                my_idx = i
            for j in range(self.num_bots):
                if name == f'dna_bot_{j}':
                    poses[j] = msg.pose[i]

        if my_idx == -1 or self.bot_id not in poses:
            return

        my_pose = poses[self.bot_id]
        q = my_pose.orientation
        siny = 2.0*(q.w*q.z + q.x*q.y)
        cosy = 1.0 - 2.0*(q.y*q.y + q.z*q.z)
        self.my_yaw = math.atan2(siny, cosy)

        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'camera_link_optical'
        bot_ids = []

        for j in range(self.num_bots):
            if j == self.bot_id:
                continue
            if j not in poses:
                continue
            dx = poses[j].position.x - my_pose.position.x
            dy = poses[j].position.y - my_pose.position.y
            cos_y = math.cos(-self.my_yaw)
            sin_y = math.sin(-self.my_yaw)
            rx = dx * cos_y - dy * sin_y
            ry = dx * sin_y + dy * cos_y
            dist = math.hypot(rx, ry)
            if dist > self.max_range or rx <= 0:
                continue
            angle = math.atan2(ry, rx)
            if abs(angle) > self.fov/2:
                continue
            p = Pose()
            p.position.x = -ry
            p.position.y = -0.05
            p.position.z = rx
            pa.poses.append(p)
            bot_ids.append(j)

        self.det_pub.publish(pa)
        ids_msg = Int32MultiArray()
        ids_msg.data = bot_ids
        self.bot_ids_pub.publish(ids_msg)

        wall_pa = PoseArray()
        wall_pa.header = pa.header
        wall_ids = []
        for wall_x, wall_id in zip(self.wall_xs, self.wall_ids):
            dx = wall_x - my_pose.position.x
            dy = 0.0 - my_pose.position.y
            cos_y = math.cos(-self.my_yaw)
            sin_y = math.sin(-self.my_yaw)
            rx = dx * cos_y - dy * sin_y
            ry = dx * sin_y + dy * cos_y
            dist = math.hypot(rx, ry)
            if dist > self.max_range or rx <= 0:
                continue
            angle = math.atan2(ry, rx)
            if abs(angle) > self.fov/2:
                continue
            p = Pose()
            p.position.x = -ry
            p.position.y = 0.0
            p.position.z = rx
            wall_pa.poses.append(p)
            wall_ids.append(wall_id)

        self.wall_pub.publish(wall_pa)
        wall_ids_msg = Int32MultiArray()
        wall_ids_msg.data = wall_ids
        self.wall_ids_pub.publish(wall_ids_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
