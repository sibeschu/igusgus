import rclpy
import cv2

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from message_filters import Subscriber, ApproximateTimeSynchronizer
from yolo_msgs.msg import DetectionArray
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class Pub3D(Node):
    """ A ROS" Node that will hopefully publish minimized sensor data """

    def __init__(self):
        super().__init__('pub_3d')

        # topics
        image_rect_raw = "/camera/camera/depth/image_rect_raw"
        detections = "/detections"
        camera_info = "/camera/camera/depth/camera_info"

        # params
        queue_size = 30
        max_delay = 0.5

        self.qos = QoSProfile(
        reliability = QoSReliabilityPolicy.BEST_EFFORT,
        history = QoSHistoryPolicy.KEEP_LAST,
        durability = QoSDurabilityPolicy.VOLATILE,
        depth = 1,
        )

        # pubs
        self.pub_3d = self.create_publisher(PointCloud2, '/pub_3d', 10)
        self.debug_pub = self.create_publisher(String, '/pub_3d_debug', 10)

        # subs
        self.detections_sub = Subscriber(self, DetectionArray, detections)
        self.depth_sub = Subscriber(self, Image, image_rect_raw, qos_profile = self.qos)
        self.depth_info = Subscriber(self, CameraInfo, camera_info, qos_profile = self.qos)

        # sync and callback
        self.sync = ApproximateTimeSynchronizer([self.detections_sub, self.depth_sub, self.depth_info], queue_size, max_delay)
        self.sync.registerCallback(self.calculate_3d)

    def calculate_3d(self, detections_msg: DetectionArray, depth_msg: Image, depth_info_msg: CameraInfo):
        self.get_logger().info(
        f"Got sync! detections={len(detections_msg.detections)}, "
        f"depth stamp={depth_msg.header.stamp.sec}, "
        f"info stamp={depth_info_msg.header.stamp.sec}, "
        # f"pixel_coords = {pixel_coords}, "
        )

        pixel_coords = []

        # get x and y coordinates from image plane
        # (from segmentation yolov8)
        for detection in detections_msg.detections:
            if (detection.mask is None) or (detection.mask.data is None) or (len(detection.mask.data) == 0):
                self.get_logger().info("mask empty")
                continue

            mask_points = detection.mask.data # list of Point2D positions
            self.get_logger().info(f"mask_points count = {len(mask_points)}")
            self.get_logger().info(f"first5 = {[(p.x, p.y) for p in mask_points[:5]]}")
            # pixel_coords = [(u1,v1), (u2,v2), ...]
            pixel_coords = [(int(pt.x), int(pt.y)) for pt in mask_points]


        # get according z coordinates for x,y,z
        self.get_logger().info(f"converting depth map..")
        self.convert_depth_image(depth_msg, pixel_coords)

        # xyz depth points to cloud

        dbg = String()
        dbg.data = "Sync triggered"
        self.debug_pub.publish(dbg)

        pass

    def convert_depth_image(self, ros_image: Image, pixel_coordinates):
        # convert to openc
        bridge = CvBridge()

        try:
            # convert depth image with default passthrough encoding
            depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32)

            if not pixel_coordinates or len(pixel_coordinates) < 3:
                self.get_logger().info("too few mask points")
                return

            poly = np.array(pixel_coordinates, dtype=np.int32)
            mask = np.zeros(depth_array.shape, np.uint8)
            cv2.fillPoly(mask, [poly], 255)
            Z = depth_array[mask>0]
            z_m = Z * 0.001  # if encoding is 16UC1; else 1.0 for 32FC1

            self.get_logger().info(f"first5 depths (m): {z_m[:5]}")

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")

def main():
        rclpy.init()
        node = Pub3D()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
