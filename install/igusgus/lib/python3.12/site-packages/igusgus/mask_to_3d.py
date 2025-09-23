#!/usr/bin/env python3
import rclpy, numpy as np, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs_py import point_cloud2

from yolo_msgs.msg import DetectionArray  # adjust if package name differs

class MaskTo3D(Node):
    def __init__(self):
        super().__init__('mask_to_3d')
        self.bridge = CvBridge()

        # params
        self.declare_parameter('depth_topic', '/camera/camera/depth/image_rect_raw')
        self.declare_parameter('info_topic',  '/camera/camera/depth/camera_info')
        self.declare_parameter('det_topic',   '/yolo/detections')
        self.declare_parameter('depth_scale', 0.001)  # 16UC1 mm->m
        self.declare_parameter('stride', 2)
        self.declare_parameter('min_z', 0.05)
        self.declare_parameter('max_z', 5.0)

        self.fx=self.fy=self.cx=self.cy=None
        self.scale=float(self.get_parameter('depth_scale').value)
        self.stride=int(self.get_parameter('stride').value)
        self.min_z=float(self.get_parameter('min_z').value)
        self.max_z=float(self.get_parameter('max_z').value)

        qos = rclpy.qos.QoSProfile(depth=1)
        qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        qos.durability  = rclpy.qos.QoSDurabilityPolicy.VOLATILE

        self.sub_depth = Subscriber(self, Image, self.get_parameter('depth_topic').value, qos_profile=qos)
        self.sub_info  = Subscriber(self, CameraInfo, self.get_parameter('info_topic').value,  qos_profile=qos)
        self.sub_det   = Subscriber(self, DetectionArray, self.get_parameter('det_topic').value, qos_profile=qos)

        self.sync = ApproximateTimeSynchronizer([self.sub_depth, self.sub_info, self.sub_det], 5, 0.15)
        self.sync.registerCallback(self.cb)

        self.centroid_pub = self.create_publisher(PointStamped, 'mask_centroid', 10)
        self.cloud_pub    = self.create_publisher(PointCloud2, 'mask_cloud', 10)

        self.get_logger().info('MaskTo3D with cloud publisher ready.')

    def cb(self, depth_msg: Image, info: CameraInfo, det_arr: DetectionArray):
        # intrinsics
        if self.fx is None:
            k = info.k
            self.fx, self.fy, self.cx, self.cy = k[0], k[4], k[2], k[5]

        # depth -> float32 m
        if depth_msg.encoding in ('16UC1','mono16'):
            d = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.uint16)
            depth = d.astype(np.float32) * self.scale
        elif depth_msg.encoding == '32FC1':
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)
        else:
            self.get_logger().warn(f'Unsupported depth encoding: {depth_msg.encoding}')
            return

        H, W = depth.shape
        s = max(1, self.stride)
        uu = np.arange(0, W, s, dtype=np.float32)
        vv = np.arange(0, H, s, dtype=np.float32)
        U, V = np.meshgrid(uu, vv)

        for det in det_arr.detections:
            m = det.mask
            if m is None or len(m.data) < 3:
                continue

            # polygon from mask.data (scale if normalized)
            poly = np.array([[p.x, p.y] for p in m.data], dtype=np.float32)
            if poly.max() <= 1.5:  # normalized [0,1]
                poly[:,0] *= (m.width or W)
                poly[:,1] *= (m.height or H)
            poly[:,0] = np.clip(poly[:,0], 0, W-1)
            poly[:,1] = np.clip(poly[:,1], 0, H-1)
            poly_i = poly.astype(np.int32)

            mask = np.zeros((H,W), dtype=np.uint8)
            cv2.fillPoly(mask, [poly_i], 255)
            mask_s = mask[::s, ::s] > 0

            Z = depth[::s, ::s][mask_s]
            valid = np.isfinite(Z) & (Z > self.min_z) & (Z < self.max_z)
            if not np.any(valid):
                continue
            Z = Z[valid]
            Uv = U[mask_s][valid]
            Vv = V[mask_s][valid]

            X = (Uv - self.cx)/self.fx * Z
            Y = (Vv - self.cy)/self.fy * Z
            pts = np.stack((X, Y, Z), axis=1)

            # publish centroid
            c = pts.mean(axis=0)
            out = PointStamped()
            out.header = depth_msg.header
            out.point.x, out.point.y, out.point.z = float(c[0]), float(c[1]), float(c[2])
            self.centroid_pub.publish(out)

            # publish cloud
            fields = [
    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
]
pts32 = pts.astype(np.float32)  # ensure float32
cloud = point_cloud2.create_cloud(depth_msg.header, fields, pts32.tolist())
self.cloud_pub.publish(cloud)

def main():
    rclpy.init()
    node = MaskTo3D()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
