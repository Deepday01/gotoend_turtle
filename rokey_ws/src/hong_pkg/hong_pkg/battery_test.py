import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time

import numpy as np
import threading
import time
import math

from enum import Enum, auto

class RobotState(Enum):
    START = auto()
    ROBOT_READY = auto()
    SEARCHING = auto()
    WAITING_USER = auto()
    APPROACHING = auto()
    DONE = auto()

class Batterytest(Node):
    def __init__(self):
        super().__init__('depth_to_map_node')

        self.lock = threading.Lock()

        # 변수 초기화
        self.state = RobotState.START

        ns = self.get_namespace()
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)

    def process_loop(self):
        pass

def main():
    rclpy.init()
    node = Batterytest()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
