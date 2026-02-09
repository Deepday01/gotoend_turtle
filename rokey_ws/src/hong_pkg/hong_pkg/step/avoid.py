#!/usr/bin/env python3
import math
import time
import numpy as np
import cv2
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CompressedImage, CameraInfo, LaserScan
from geometry_msgs.msg import PointStamped

from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point

from ultralytics import YOLO


class YoloDepthToScan(Node):
    """
    YOLO + Depth -> LaserScan
    - 최신 프레임만 처리
    - YOLO 중심점 기준 depth 취득
    - base_link 기준 LaserScan 생성
    - spread를 이용해 원형 충돌범위 느낌 구현
    """

    def __init__(self):
        super().__init__('yolo_depth_to_scan')

        # -----------------------------
        # Topics (namespace-aware)
        # -----------------------------
        ns = self.get_namespace()  # e.g. /robot5
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.rgb_topic   = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic  = f'{ns}/oakd/rgb/camera_info'
        self.scan_topic  = f'{ns}/yolo_scan'

        # -----------------------------
        # Parameters
        # -----------------------------
        self.model_path = '/home/rokey/Desktop/project/gotoend_turtle/rokey_ws/src/hong_pkg/hong_pkg/yolov8s.pt'
        self.conf_thres = 0.35
        self.max_det = 20
        self.imgsz = 416

        self.z_min = 0.20
        self.z_max = 6.00

        self.roi_half = 2

        # LaserScan config
        self.angle_min = -math.pi / 2
        self.angle_max =  math.pi / 2
        self.angle_inc = math.radians(1.0)
        self.range_min = self.z_min
        self.range_max = self.z_max

        # YOLO obstacle radius (costmap 두께)
        self.yolo_radius_m = 0.25
        self.min_spread_deg = 2.0
        self.max_spread_deg = 40.0

        # Frames
        self.base_frame = 'base_link'

        # Debug
        self.show_debug = True
        self.win_name = 'YOLO center debug'
        self.show_dt = 0.1
        self._last_show_t = 0.0

        self.min_infer_dt = 0.12
        self.last_infer_t = 0.0

        # -----------------------------
        # State
        # -----------------------------
        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)
        self.get_logger().info(f'Loaded YOLO model: {self.model_path}')

        self.K = None
        self.camera_frame = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_scan = self.create_publisher(LaserScan, self.scan_topic, 10)

        self.lock = Lock()
        self.last_rgb = None
        self.last_depth = None

        # Subscribers
        self.create_subscription(CameraInfo, self.info_topic, self.cb_info, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.cb_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.cb_depth, qos_profile_sensor_data)

        self.timer = self.create_timer(0.05, self.process_latest)

    # -----------------------------
    # Subscribers
    # -----------------------------
    def cb_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float32).reshape(3, 3)
        self.camera_frame = msg.header.frame_id

    def cb_rgb(self, msg: CompressedImage):
        with self.lock:
            self.last_rgb = msg

    def cb_depth(self, msg: Image):
        with self.lock:
            self.last_depth = msg

    # -----------------------------
    # Main processing
    # -----------------------------
    def process_latest(self):
        if self.K is None or self.camera_frame is None:
            return

        with self.lock:
            rgb_msg = self.last_rgb
            depth_msg = self.last_depth

        if rgb_msg is None or depth_msg is None:
            return

        now = time.time()
        if now - self.last_infer_t < self.min_infer_dt:
            return
        self.last_infer_t = now

        frame = cv2.imdecode(np.frombuffer(rgb_msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough')
        except Exception:
            return

        depth_is_mm = depth.dtype == np.uint16

        results = self.model.predict(
            frame,
            imgsz=self.imgsz,
            conf=self.conf_thres,
            max_det=self.max_det,
            verbose=False
        )

        boxes = results[0].boxes if results and results[0].boxes is not None else []

        n = int((self.angle_max - self.angle_min) / self.angle_inc) + 1
        ranges = [float('inf')] * n

        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        for b in boxes:
            x1, y1, x2, y2 = b.xyxy[0]
            u = int((x1 + x2) * 0.5)
            v = int((y1 + y2) * 0.5)

            z = self.get_depth_roi_median(depth, u, v, depth_is_mm)
            if z is None:
                continue

            Xc = (u - cx) * z / fx
            Yc = (v - cy) * z / fy

            pt = PointStamped()
            pt.header.frame_id = self.camera_frame
            pt.header.stamp = depth_msg.header.stamp
            pt.point.x = float(Xc)
            pt.point.y = float(Yc)
            pt.point.z = float(z)

            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    rclpy.time.Time()
                )
                ptb = do_transform_point(pt, tf)
            except Exception:
                continue

            xb, yb = ptb.point.x, ptb.point.y
            if xb <= 0.05:
                continue

            rng = math.hypot(xb, yb)
            if not (self.range_min <= rng <= self.range_max):
                continue

            theta = math.atan2(yb, xb)
            if self.angle_min <= theta <= self.angle_max:
                self.apply_spread(ranges, theta, rng)

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.base_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges

        self.pub_scan.publish(scan)

    # -----------------------------
    # Helpers
    # -----------------------------
    def get_depth_roi_median(self, depth, u, v, depth_is_mm):
        h, w = depth.shape[:2]
        x0 = max(0, u - 2)
        x1 = min(w - 1, u + 2)
        y0 = max(0, v - 2)
        y1 = min(h - 1, v + 2)

        roi = depth[y0:y1 + 1, x0:x1 + 1].astype(np.float32)
        roi = roi[roi > 0]

        if roi.size == 0:
            return None

        z = float(np.median(roi))
        return z * 0.001 if depth_is_mm else z

    def apply_spread(self, ranges, theta, rng):
        spread = 2.0 * math.atan2(self.yolo_radius_m, max(rng, 1e-3))
        spread = math.degrees(spread)
        spread = max(self.min_spread_deg, min(self.max_spread_deg, spread))
        spread = math.radians(spread)

        a0 = theta - spread * 0.5
        a1 = theta + spread * 0.5

        i0 = int((a0 - self.angle_min) / self.angle_inc)
        i1 = int((a1 - self.angle_min) / self.angle_inc)

        n = len(ranges)
        i0 = max(0, min(n - 1, i0))
        i1 = max(0, min(n - 1, i1))

        for i in range(min(i0, i1), max(i0, i1) + 1):
            ranges[i] = min(ranges[i], rng)


def main():
    rclpy.init()
    node = YoloDepthToScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
