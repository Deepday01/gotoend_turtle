import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Quaternion
from std_msgs.msg import Bool

from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

import numpy as np
import cv2
import threading
import time
import math

from .utils.nav_util import NavProcessor
from .utils.depth_util import DepthProcessor
from .utils.cv_util import CVProcessor
from .utils.math_util import MathProcessor 
from .utils.yolo_util import YOLOProcessor
#from .actions.rotation_manager import RotationManager

# AMCL QoS 설정
qos_amcl = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

class DepthToMap(Node):
    def __init__(self):
        super().__init__('depth_to_map_node')

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        self.nav = NavProcessor()
        
        #추가
        # self.started = False
        # self.delay_timer = None
        # self.main_timer = None
        self.nav.dock() 
        self.nav.nav_setup(0.0, 0.0, 0.0)
        self.nav.undock()

        self.depth_proc = DepthProcessor()
        self.cv = CVProcessor()
        self.math = MathProcessor()
        model_path = '/home/rokey/gotoend_ws2/rokey_ws/src/hong_pkg/hong_pkg/my_best.pt' 
        self.yolo = YOLOProcessor(model_path) 

        # 변수 초기화
        self.depth_image = None
        self.rgb_image = None
        self.camera_frame = None
        self.clicked_point = None
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        
        self.display_image = None
        self.gui_thread_stop = threading.Event()

        ns = self.get_namespace()
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'
        self.amcl_pose = '/robot4/amcl_pose'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose, self.amcl_callback, qos_amcl)
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 1)

        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        self.get_logger().info("TF Tree 안정화 중... (5초)")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    # def start_callback(self, msg):
    #     if (not msg.data) or self.started:
    #         return

    #     self.started = True
    #     self.get_logger().info("출발 신호 수신! 5초 후 시작")
    #     self.delay_timer = self.create_timer(5.0, self.start_after_delay)

    # def start_after_delay(self):
    #     if self.delay_timer is not None:
    #         self.delay_timer.cancel()
    #         self.delay_timer = None

    #     self.nav.dock()
    #     self.nav.nav_setup(0.0, 0.0, 0.0)
    #     self.nav.undock()

    #     if self.main_timer is None:
    #         self.main_timer = self.create_timer(0.1, self.process_loop)

    #     if self.start_sub is not None:
    #         self.destroy_subscription(self.start_sub)
    #         self.start_sub = None
    #         self.get_logger().info("start_topic 구독 해제 완료 (이후 신호 무시)")

            
    def start_transform(self):
        self.get_logger().info("시스템 준비 완료. 메인 루프 시작.")
        self.timer = self.create_timer(0.1, self.process_loop) # 0.1초마다 처리
        self.start_timer.cancel()

    def amcl_callback(self, msg):
        with self.lock:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.robot_yaw = self.math.quaternion_to_orientation_yaw(q)

    def camera_info_callback(self, msg):
        with self.lock:
            self.depth_proc.set_intrinsics(msg.k)

    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if depth is not None:
                with self.lock:
                    self.depth_image = depth
                    self.camera_frame = msg.header.frame_id
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if rgb is not None:
                with self.lock:
                    self.rgb_image = rgb
        except Exception as e:
            self.get_logger().error(f"RGB error: {e}")

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            with self.lock:
                self.clicked_point = (x, y)
                self.get_logger().info(f"👉 Clicked Pixel: ({x}, {y})")

    def process_loop(self):
        with self.lock:
            if self.rgb_image is None or self.depth_image is None:
                return

            rgb = self.rgb_image.copy()
            depth = self.depth_image.copy()
            click = self.clicked_point
            frame_id = self.camera_frame

        rgb_detected, detections = self.yolo.detect_tracking_box(rgb)

        if click is not None and frame_id:
            x, y = click
            click_check, data = self.yolo.is_bounding_box(detections, x, y)
            print(f"cneter : {data}")
            if click_check and data is not None:
                cx, cy = data
                pt_map = self.depth_proc.get_xy_transform(self.tf_buffer, depth, int(cx), int(cy), frame_id)

                if pt_map:
                    P_goal, yaw_face = self.math.get_standoff_goal_yaw(self.robot_x, self.robot_y, pt_map, distance=0.6)
                    self.get_logger().info(f"Goal Set: ({P_goal[0]:.2f}, {P_goal[1]:.2f})")
                    # goal_pose = PoseStamped()
                    # goal_pose.header.frame_id = 'map'
                    # goal_pose.header.stamp = self.get_clock().now().to_msg()
                    # goal_pose.pose.position.x = P_goal[0]
                    # goal_pose.pose.position.y = P_goal[1]
                    # goal_pose.pose.position.z = 0.0
                    # yaw = float(self.robot_yaw)
                    # qz = math.sin(yaw / 2.0)
                    # qw = math.cos(yaw / 2.0)
                    # goal_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
                    self.nav.go_to_pose_yaw(self.get_clock().now().to_msg(), P_goal, yaw_face)
                    
                else:
                    self.get_logger().warn("유효하지 않은 좌표거나 Depth 범위 밖입니다.")

        with self.lock:
            if click is not None:
                self.clicked_point = None
            self.display_image = rgb_detected
    
    def gui_loop(self):
        window_name = "Robot Control View (RGB)"
        
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(window_name, self.mouse_callback)

        while not self.gui_thread_stop.is_set():
            img = None
            with self.lock:
                if self.display_image is not None:
                    img = self.display_image.copy()
            if img is not None:
                self.cv.show_single(window_name, img, 704, 704)
                key = cv2.waitKey(1)
                if key == ord('q'):
                    self.gui_thread_stop.set()
                    rclpy.shutdown()
            else:
                cv2.waitKey(10)

def main():
    rclpy.init()
    node = DepthToMap()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.gui_thread_stop.set()
        node.gui_thread.join()
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()
