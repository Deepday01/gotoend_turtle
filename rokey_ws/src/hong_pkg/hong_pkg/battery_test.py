import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int32, Int32MultiArray # MultiArray 추가

from threading import Lock
from enum import Enum, auto

# 모듈 import
from .modules.battery_processor import BatteryProcessor

class RobotState(Enum):
    START = auto()
    ROBOT_READY = auto()
    SEARCHING = auto()
    WAITING_USER = auto()
    APPROACHING = auto()
    DONE = auto()

class Batterytest(Node):
    def __init__(self):
        super().__init__('battery_test_node')

        self.lock = Lock()

        self.state = RobotState.START
        self.battery_percent = 0.0
        
        self.line1_count = 0 
        self.line2_count = 0
        
        self.line_status = {1: False, 2: False} 

        self.my_robot_id = 1
        self.battery_proc = BatteryProcessor(self.my_robot_id)

        self.battery_state_subscriber = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_state_callback,
            qos_profile_sensor_data
        )
        
        self.line1_total_subscriber = self.create_subscription(
            Int32,
            '/qr/count_total',
            self.line1_total_callback,
            1
        )

        self.line_status_subscriber = self.create_subscription(
            Int32MultiArray,
            '/line_status',
            self.line_status_callback,
            1
        )

        self.timer = self.create_timer(0.1, self.main_controller)

    def line_status_callback(self, msg):
        with self.lock:
            if len(msg.data) >= 3:
                self.line_status[1] = bool(msg.data[1])
                self.line_status[2] = bool(msg.data[2])

    def line1_total_callback(self, msg):
        with self.lock:
            self.line1_count = msg.data
            self.get_logger().info(f'data: {self.line1_count}')

    def battery_state_callback(self, batt_msg: BatteryState):
        with self.lock:
            self.battery_percent = batt_msg.percentage
            if self.state == RobotState.START:
                self.get_logger().info(f'Battery: {self.battery_percent:.2f}%')
                self.state = RobotState.ROBOT_READY

    def main_controller(self):
        with self.lock:
            current_battery = self.battery_percent
            q1 = self.line1_count
            q2 = self.line2_count
            current_status = self.line_status.copy()

        self.battery_proc.pick_up_waiting(
            0.2, 
            q1, 
            q2, 
            current_status
        )

def main(args=None):
    rclpy.init(args=args)
    node = Batterytest()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()