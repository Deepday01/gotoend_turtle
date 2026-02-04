from ..utils.nav_util import NavProcessor
import time

class MainProcessor:
    def __init__(self, my_robot_id):
        
        self.robot_id = my_robot_id

        # 로봇 아이디별로 라인 배정
        if my_robot_id == 4:
            self.my_line_id = 1
            self.other_line_id = 2
        else :
            self.my_line_id = 2
            self.other_line_id = 1
        self.nav = NavProcessor()

    # battery percent, 자신의 라인 박스 갯수, 다른 라인 박스 갯수, 각 라인 작업 상태
    def pick_up_waiting(self, battery_percent, my_queue_count, other_queue_count, line_status):
        battery = battery_percent * 100
        # 배터리가 30프로 미만일때 도킹하러감
        if battery < 30:
            print('Low Battery! Go to Dock')
            if self.robot_id == 4:
                goal = [[-1.59, -0.47]]
                self.move_and_wait(goal, 0.0)
            else:
                goal = [[-1.53, 0.85]]
                self.move_and_wait(goal, 0.0)

        elif my_queue_count > 0:
            if line_status.get(self.my_line_id) == True:
                print(f"내 라인({self.my_line_id}) 작업 대기 중 (Occupied)...")
                return
            
            print(f'내 라인({self.my_line_id}) 작업 시작')

        elif other_queue_count > 0:
            if line_status.get(self.other_line_id) == True:
                print(f"{self.other_line_id}번 지원 대기 중 (Occupied)...")
                return
            if self.robot_id == 4:
                goal = [[-2.90, -1.67]]
                self.move_and_wait(goal, 0.0)
            else :
                goal = [[-1.58, -1.45]]
                self.move_and_wait(goal, 0.0)
            
            print(f"{self.other_line_id}번 라인 지원 출발")
        else:
            pass
    # x 좌표, y 좌표, 로봇이 바라보는 방향
    def move_and_wait(self, goal_array, yaw):
        self.nav.go_to_follow(goal_array = goal_array, goal_or = yaw)
        # waitting
        while not self.nav.navigator.isTaskComplete():
            time.sleep(0.1)

        print("도착 완료 (Action Complete)")