#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import math
import numpy as np

class MarkerDetector(Node):
    def __init__(self):
        super().__init__('marker_detector')
        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('camera_topic', 'camera/image_raw')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('bot_detection_topic', 'detected_bots')
        self.declare_parameter('wall_detection_topic', 'detected_walls')
        self.declare_parameter('bot_marker_ids', [0, 1])
        self.declare_parameter('wall_marker_ids', [10, 11, 12])

        self.marker_size = self.get_parameter('marker_size').value
        self.bot_ids = set(self.get_parameter('bot_marker_ids').value)
        self.wall_ids = set(self.get_parameter('wall_marker_ids').value)

        self.bridge = CvBridge()
        self.bot_pub = self.create_publisher(PoseArray, self.get_parameter('bot_detection_topic').value, 10)
        self.wall_pub = self.create_publisher(PoseArray, self.get_parameter('wall_detection_topic').value, 10)
        self.bot_ids_pub = self.create_publisher(Int32MultiArray, 'detected_bot_ids', 10)
        self.wall_ids_pub = self.create_publisher(Int32MultiArray, 'detected_wall_ids', 10)

        self.camera_matrix = None
        self.dist_coeffs = None

        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value, self.info_cb, 10)
        self.create_subscription(Image, self.get_parameter('camera_topic').value, self.image_cb, 10)

        try:
            self.dictionary = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
            self.parameters = aruco.DetectorParameters()
        except AttributeError:
            self.dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
            self.parameters = aruco.DetectorParameters_create()

        self.get_logger().info("Marker detector started. Bot IDs: {}  Wall IDs: {}".format(self.bot_ids, self.wall_ids))

    def info_cb(self, msg):
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k).reshape(3,3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("Camera info received")

    def image_cb(self, msg):
        if self.camera_matrix is None:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"CV bridge error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        bot_poses = PoseArray()
        bot_poses.header = msg.header
        wall_poses = PoseArray()
        wall_poses.header = msg.header
        bot_id_list = []
        wall_id_list = []

        if ids is not None:
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                tvec = tvecs[i][0]
                rvec = rvecs[i][0]

                pose = Pose()
                pose.position.x = tvec[0]
                pose.position.y = tvec[1]
                pose.position.z = tvec[2]
                angle = np.linalg.norm(rvec)
                if angle > 0:
                    axis = rvec / angle
                    pose.orientation.x = axis[0] * math.sin(angle/2)
                    pose.orientation.y = axis[1] * math.sin(angle/2)
                    pose.orientation.z = axis[2] * math.sin(angle/2)
                    pose.orientation.w = math.cos(angle/2)
                else:
                    pose.orientation.w = 1.0

                if marker_id in self.bot_ids:
                    bot_poses.poses.append(pose)
                    bot_id_list.append(marker_id)
                elif marker_id in self.wall_ids:
                    wall_poses.poses.append(pose)
                    wall_id_list.append(marker_id)

        self.bot_pub.publish(bot_poses)
        self.wall_pub.publish(wall_poses)

        bot_ids_msg = Int32MultiArray()
        bot_ids_msg.data = bot_id_list
        self.bot_ids_pub.publish(bot_ids_msg)

        wall_ids_msg = Int32MultiArray()
        wall_ids_msg.data = wall_id_list
        self.wall_ids_pub.publish(wall_ids_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
