from irobot_create_msgs.action import DriveArc
from .action_base import ActionBase 

class DriveArcManager(ActionBase):
    def __init__(self, node):
        super().__init__(node, DriveArc, '/robot4/drive_arc')
        
    
    def drive_arc_step(self, angle=1.57, radius=0.3, translate_direction=1, max_translation_speed=0.3):
        goal = DriveArc.Goal()
        goal.angle = angle
        goal.radius = radius
        goal.translate_direction = translate_direction
        goal.max_translation_speed = max_translation_speed
        self.node.get_logger().info("물체를 피해 갑니다")
        self.send_goal_base(goal)

    def stop(self):
        self.node.get_logger().info("종료합니다.")
        self.cancel_base()