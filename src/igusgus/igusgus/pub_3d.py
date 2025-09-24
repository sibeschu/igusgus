import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from message_filters import Subscriber, ApproximateTimeSynchronizer
from yolo_msgs.msg import DetectionArray
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

class Pub3D(Node):
    """ A ROS" Node that will hopefully publish minimized sensor data """

    def __init__(self):
        super().__init__('pub_3d')

        self.qos = QoSProfile(
        reliability = QoSReliabilityPolicy.BEST_EFFORT,
        history = QoSHistoryPolicy.KEEP_LAST,
        durability = QoSDurabilityPolicy.VOLATILE,
        depth = 1,
        )

        image_rect_raw = "/camera/camera/depth/image_rect_raw"

        # pubs
        self.pub_3d = self.create_publisher(PointCloud2, '/pub_3d', 10)
        self.debug_pub = self.create_publisher(String, '/pub_3d_debug', 10)

        # subs
        self.detections_sub = Subscriber(self, DetectionArray, "/detections")
        self.depth_sub = Subscriber(self, Image, image_rect_raw, qos_profile = self.qos)
        self.depth_info = Subscriber(self, CameraInfo, "/camera/camera/depth/camera_info", qos_profile = self.qos)

        queue_size = 30
        max_delay = 0.5

        self.sync = ApproximateTimeSynchronizer([self.detections_sub, self.depth_sub, self.depth_info], queue_size, max_delay)
        self.sync.registerCallback(self.calculate_3d)

    def calculate_3d(self, detections_msg: DetectionArray, depth_msg: Image, depth_info_msg: CameraInfo):
        # detections_msg is DetectionArray
        # depth_msg is an Image
        # info_msg is a CameraInfo

        self.get_logger().info(
        f"Got sync! detections={len(detections_msg.detections)}, "
        f"depth stamp={depth_msg.header.stamp.sec}, "
        f"info stamp={depth_info_msg.header.stamp.sec}, "
        # f"pixel_coords = {pixel_coords}, "
        )

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

        # xyz depth points to cloud

        dbg = String()
        dbg.data = "Sync triggered"
        self.debug_pub.publish(dbg)

        pass

def main():
        rclpy.init()
        node = Pub3D()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
