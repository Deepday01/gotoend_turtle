import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data

import numpy as np
from threading import lock, Thread
import time
import math
from sensor_msgs.msg import BatteryState

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

        self.lock = lock

        # 변수 초기화
        self.state = RobotState.START

        ns = self.get_namespace()
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'

        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            'battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data)

    def battery_state_callback(self):
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
