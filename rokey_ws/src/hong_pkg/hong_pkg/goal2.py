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

# AMCL QoS ì„¤ì •
qos_amcl = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

# í´ëž˜ìŠ¤ ìƒì„±
class DepthToMap(Node):
    def __init__(self):
        super().__init__('depth_to_map_node')

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        self.nav = NavProcessor()
        self.nav.dock() 
        self.nav.nav_setup(0.0, 0.0, 0.0)
        self.nav.undock()

        self.depth_proc = DepthProcessor()
        self.cv = CVProcessor()
        self.math = MathProcessor()
        model_path = '/home/rokey/Desktop/project/gotoend_turtle/rokey_ws/src/hong_pkg/hong_pkg/best.pt' 
        self.yolo = YOLOProcessor(model_path) 

        # ë³€ìˆ˜ ì´ˆê¸°í™”
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
        self.start_topic = '/start_topic'

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(PoseWithCovarianceStamped, self.amcl_pose, self.amcl_callback, qos_amcl)
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 1)
        self.create_subscription(Bool, self.start_topic, self.start_callback, 1)

        self.gui_thread = threading.Thread(target=self.gui_loop, daemon=True)
        self.gui_thread.start()

        self.get_logger().info("TF Tree ì•ˆì •í™” ì¤‘... (5ì´ˆ)")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def start_callback(self, msg):
        if msg.data is True:
            self.get_logger().info("ì¶œë°œ ì‹ í˜¸ ë°›ì•˜ìŠµë‹ˆë‹¤.")
        else :
            pass

    def start_transform(self):
        self.get_logger().info("ì‹œìŠ¤í…œ ì¤€ë¹„ ì™„ë£Œ. ë©”ì¸ ë£¨í”„ ì‹œìž‘.")
        self.timer = self.create_timer(0.1, self.process_loop) # 0.1ì´ˆë§ˆë‹¤ ì²˜ë¦¬
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
                self.get_logger().info(f"ðŸ‘‰ Clicked Pixel: ({x}, {y})")

    def process_loop(self):
        with self.lock:
            if self.rgb_image is None or self.depth_image is None:
                return
            
            # rgb ì´ë¯¸ì§€ ë³µì‚¬
            rgb = self.rgb_image.copy()
            # ê¹Šì´ ë°°ì—´ ë³µì‚¬
            depth = self.depth_image.copy()
            # í´ë¦­ í¬ì¸íŠ¸ ì¢Œí‘œ ë³µì‚¬ 
            click = self.clicked_point
            # ì¹´ë©”ë¼ ì¢Œí‘œê³„ ê°€ì ¸ì˜¤ê¸°
            frame_id = self.camera_frame

        rgb_detected, detections = self.yolo.detect_tracking_box(rgb)

        if click is not None and frame_id:
            target_pos = []
            for data in detections:
                target_pos.append(data['box_pos'])
            self.get_logger().info(f"detect : {target_pos}")
            x, y = click
                    
            pt_map = self.depth_proc.get_xy_transform(self.tf_buffer, depth, x, y, frame_id)

            if pt_map:
                P_goal, yaw_face = self.math.get_standoff_goal(self.robot_x, self.robot_y, pt_map, distance=0.3)
                self.get_logger().info(f"Goal Set: ({P_goal[0]:.2f}, {P_goal[1]:.2f})")
                ###
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
                ###
                clock = self.get_clock().now().to_msg()

                self.nav.go_to_pose2(clock, P_goal, yaw_face)
                
                #self.nav.go_to_pose(P_goal[0], P_goal[1], self.robot_yaw)
                with self.lock:
                    self.clicked_point = None
            else:
                self.get_logger().warn("ìœ íš¨í•˜ì§€ ì•Šì€ ì¢Œí‘œê±°ë‚˜ Depth ë²”ìœ„ ë°–ìž…ë‹ˆë‹¤.")

        with self.lock:
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