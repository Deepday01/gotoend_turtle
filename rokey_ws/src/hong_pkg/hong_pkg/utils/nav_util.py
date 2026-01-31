from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import math

class NavProcessor():
    def __init__(self):
        self.navigator = TurtleBot4Navigator()
        self.navigator.waitUntilNav2Active()

    def nav_setup(self, start_x, start_y, start_or):
        initial_pose = self.navigator.getPoseStamped([start_x, start_y], start_or)
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()
        self.navigator.info(f"초기 위치 설정 완료: ({start_x}, {start_y})")

    def go_to_pose(self, goal_x, goal_y, goal_or, start_x=None, start_y=None, start_or=None):
        if start_x is not None and start_y is not None:
            if start_or is None:
                start_or = TurtleBot4Directions.NORTH
            self.nav_setup(start_x, start_y, start_or)
        
        goal_pose = self.navigator.getPoseStamped([goal_x, goal_y], goal_or)
        self.navigator.startToPose(goal_pose)
    
    def go_to_through(self, goal_array, goal_or, start_x=None, start_y=None, start_or=None):
        if start_x is not None and start_y is not None:
            if start_or is None:
                start_or = TurtleBot4Directions.NORTH
            self.nav_setup(start_x, start_y, start_or)

        goal_pose = []

        for point in goal_array:
            goal_pose.append(self.navigator.getPoseStamped([point[0], point[1]], goal_or))

        self.navigator.startThroughPoses(goal_pose)

    def go_to_follow(self, goal_array, goal_or, start_x=None, start_y=None, start_or=None):
        if start_x is not None and start_y is not None:
            if start_or is None:
                start_or = TurtleBot4Directions.NORTH
            self.nav_setup(start_x, start_y, start_or)

        goal_pose = []

        for point in goal_array:
            goal_pose.append(self.navigator.getPoseStamped([point[0], point[1]], goal_or))

        self.navigator.startFollowWaypoints(goal_pose)

    
    def spin(self, angle_deg):
        angle_rad = math.radians(angle_deg)
        self.navigator.info(f"제자리 돌기{angle_rad}")
        self.navigator.spin(spin_dist=angle_rad, time_allowance=10)

    def stop(self):
        self.navigator.info("action stop")
        self.navigator.cancelTask()

    def dock(self):
        if not self.navigator.getDockedStatus():
            print('도킹을 시작합니다...')
            self.navigator.dock()
        else:
            print('이미 도킹되어 있습니다.')

    def undock(self):
        if self.navigator.getDockedStatus():
            print('언독킹을 시작합니다...')
            self.navigator.undock()
        else:
            print('이미 언독되어 있습니다 (로봇 위치 확인 필요).')