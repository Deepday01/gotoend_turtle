from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Int32MultiArray

class DriveCommander:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(Twist, '/robot4/cmd_vel', 10)
        self.publisher = node.create_publisher(Int32MultiArray, '/line_status', 10)

    def send_velocity(self, linear_x, angular_z):
        msg = Twist()
        
        msg.linear.x = float(linear_x)
        msg.linear.y = 0.0              
        msg.linear.z = 0.0              

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(angular_z)

        self.publisher.publish(msg)

    def stop(self):
        self.send_velocity(0.0, 0.0)

    def send_line_status(self, robot_id):
        msg = Int32MultiArray()

        if robot_id == 4:
            msg.data = [0,1,0]
        else :
            msg.data = [0,0,1]

        self.publisher.publish(msg)