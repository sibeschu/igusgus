import rclpy
import cv2
import numpy as np
import std_msgs.msg

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from builtin_interfaces.msg import Time
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs_py import point_cloud2
from yolo_msgs.msg import DetectionArray
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge, CvBridgeError

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_point


class Pub3D(Node):
    """ A ROS" Node that will hopefully publish sensor data """

    def __init__(self):
        super().__init__('pub_3d')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # topics (make params to pass different ones on startup)
        image_rect_raw = "/camera/camera/aligned_depth_to_color/image_raw" # "/camera/camera/depth/image_rect_raw"
        detections = "/detections"
        camera_info = "/camera/camera/color/camera_info" # "/camera/camera/depth/camera_info"

        # params
        self.declare_parameter('debug_enable', False)
        self.declare_parameter('topic_sync_queue_size', 30)
        self.declare_parameter('topic_sync_max_delay', 0.5)

        self.debug_enable = self.get_parameter("debug_enable").get_parameter_value().bool_value
        self.topic_sync_queue_size = self.get_parameter("topic_sync_queue_size").get_parameter_value().integer_value
        self.topic_sync_max_delay = self.get_parameter("topic_sync_max_delay").get_parameter_value().double_value

        self.qos = QoSProfile(
        reliability = QoSReliabilityPolicy.BEST_EFFORT,
        history = QoSHistoryPolicy.KEEP_LAST,
        durability = QoSDurabilityPolicy.VOLATILE,
        depth = 1,
        )

        # pubs
        self.pub_3d = self.create_publisher(PointCloud2, '/pub_3d', 10)
        self.tool_coord = self.create_publisher(PoseStamped, '/tool_coord', 10)

        # subs
        self.detections_sub = Subscriber(self, DetectionArray, detections)
        self.depth_sub = Subscriber(self, Image, image_rect_raw, qos_profile = self.qos)
        self.depth_info = Subscriber(self, CameraInfo, camera_info, qos_profile = self.qos)

        # sync and callback
        self.sync = ApproximateTimeSynchronizer([self.detections_sub, self.depth_sub, self.depth_info], self.topic_sync_queue_size, self.topic_sync_max_delay)
        self.sync.registerCallback(self.calculate_3d)

    def calculate_3d(self, detections_msg: DetectionArray, depth_msg: Image, depth_info_msg: CameraInfo):
        # self.get_logger().info(
        # f"Got sync! detections={len(detections_msg.detections)}, "
        # f"depth stamp={depth_msg.header.stamp.sec}, "
        # f"info stamp={depth_info_msg.header.stamp.sec}, "
        # )

        pixel_coords = []

        # get x and y coordinates from image plane
        # (from segmentation yolov8)
        for detection in detections_msg.detections:
            if (detection.mask is None) or (detection.mask.data is None) or (len(detection.mask.data) == 0):
                self.get_logger().info("calculate_3d error : mask empty")
                continue

            mask_points = detection.mask.data # list of Point2D positions
        #    self.get_logger().info(f"mask_points count = {len(mask_points)}")
        #    self.get_logger().info(f"first5 = {[(p.x, p.y) for p in mask_points[:5]]}")
            # pixel_coords = [(u1,v1), (u2,v2), ...]
            pixel_coords = [(int(pt.x), int(pt.y)) for pt in mask_points]


        # get according z coordinates for x,y,z
        if self.debug_enable:
            self.get_logger().info(f"converting depth map..")
        pts_uv, z_coord = self.convert_depth_image(depth_msg, pixel_coords)
        if pts_uv is None or z_coord is None or len(z_coord) == 0:
            return
        if self.debug_enable:
            self.get_logger().info(f"z_coord: {z_coord[:5]}")

        # xyz depth points to cloud
        xyz = self.back_project_pixels(pts_uv, z_coord, depth_info_msg)
        if self.debug_enable:
            self.get_logger().info(f"xyz: {xyz[:5]}")

        header = std_msgs.msg.Header()
        header.stamp = depth_msg.header.stamp
        header.frame_id = depth_msg.header.frame_id

        cloud_msg = self.make_cloud(header, xyz)
        self.pub_3d.publish(cloud_msg)

        self.calculate_tool_coord(header, xyz)

        if self.debug_enable:
            pass
            dbg = String()
            dbg.data = "Sync triggered"
            self.debug_pub.publish(dbg)


    def convert_depth_image(self, ros_image: Image, pixel_coordinates):
        # convert to openc
        bridge = CvBridge()

        try:
            # convert depth image with default passthrough encoding
            depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32)

            if not pixel_coordinates or len(pixel_coordinates) < 3:
                self.get_logger().info("convert_depth_image error : too few mask points")
                return None, None

            poly = np.array(pixel_coordinates, dtype=np.int32).reshape(-1, 1, 2)
            mask = np.zeros(depth_array.shape, np.uint8)
            cv2.fillPoly(mask, [poly], 255)

            V, U = np.where(mask > 0)
            Z = depth_array[V, U]
            z_m = Z * 0.001  # if encoding is 16UC1; else 1.0 for 32FC1

            if self.debug_enable:
                self.get_logger().info(f"first5 depths (m): {z_m[:5]}")

            return list(zip(U.astype(np.float32), V.astype(np.float32))), z_m

        except CvBridgeError as e:
            self.get_logger().error(f"CvBridgeError: {e}")

    def back_project_pixels(self, pixel_coordinates, z_m, cam_info):
            """pixel_coordinates: list[(u,v)], z_m: 1D array (meters), cam_info: CameraInfo"""
            # intrinsics
            fx, fy, cx, cy = cam_info.k[0], cam_info.k[4], cam_info.k[2], cam_info.k[5]
            if self.debug_enable:
                self.get_logger().info(f"fx, fy, cx, cy :{fx, fy, cx, cy}")
            # to arrays
            uv = np.asarray(pixel_coordinates, dtype=np.float32)  # Nx2
            Z  = np.asarray(z_m, dtype=np.float32).reshape(-1)    # N
            if self.debug_enable:
                self.get_logger().info(f"uv, Z: {uv, Z}")
            if uv.shape[0] == 0 or uv.shape[0] != Z.shape[0]:
                return np.empty((0,3), np.float32)

            U, V = uv[:,0], uv[:,1]
            # back-project
            X = (U - cx) / fx * Z
            Y = (V - cy) / fy * Z
            if self.debug_enable:
                self.get_logger().info(f"X, Y: {X, Y}")
            pts = np.stack([X, Y, Z], axis=1).astype(np.float32)  # Nx3
            if self.debug_enable:
                self.get_logger().info(f"pts: {pts}")
            return pts

    def make_cloud(self, header, xyz_points):
            """header: std_msgs.msg.Header, xyz_points: Nx3 array/list (float)"""
            pts = [(float(x), float(y), float(z)) for x,y,z in np.asarray(xyz_points)]
            return point_cloud2.create_cloud_xyz32(header, pts)

    def calculate_tool_coord(self, header, xyz_points):
            if xyz_points.shape[0] == 0:
                return None

            # extract Z
            z_values = xyz_points[:, 2]

            # lower / upper bounds (2.5th and 97.5th percentiles)
            lower_z, upper_z = np.percentile(z_values, [45, 55])

            # keep only points in 95% range
            valid_mask = (z_values >= lower_z) & (z_values <= upper_z)
            filtered_points = xyz_points[valid_mask]

            if filtered_points.shape[0] == 0:
                return None

            center = np.mean(filtered_points, axis=0)

            tool_pose_cam = PointStamped()
            tool_pose_cam.header = header
            tool_pose_cam.point.x, tool_pose_cam.point.y, tool_pose_cam.point.z = map(float, center)

            try:
                 tf = self.tf_buffer.lookup_transform("world", header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.2))
                 p_world = do_transform_point(tool_pose_cam, tf)

                 tool_pose = PoseStamped()
                 tool_pose.header = p_world.header
                 tool_pose.pose.position.x = p_world.point.x
                 tool_pose.pose.position.y = p_world.point.y
                 tool_pose.pose.position.z = p_world.point.z
                 tool_pose.pose.orientation.w = 1.0
                 self.tool_coord.publish(tool_pose)
                 return tool_pose

            except TransformException as e:
                self.get_logger().warn(f"TF transform failed: {e}")
                return None

def main():
        rclpy.init()
        node = Pub3D()
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()
