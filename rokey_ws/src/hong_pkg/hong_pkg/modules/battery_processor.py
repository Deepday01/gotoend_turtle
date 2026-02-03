from ..utils.nav_util import NavProcessor
import time

class BatteryProcessor:
    def __init__(self, my_line_id):
        self.my_line_id = my_line_id
        self.nav = NavProcessor()
        self.other_line_id = 2 if my_line_id == 1 else 1

    def pick_up_waiting(self, battery_percent, my_queue_count, other_queue_count, line_status):
        battery = battery_percent * 100

        if battery < 30:
            print('⚠️ Low Battery! Go to Dock')
            self.move_and_wait(-0.74, 1.99, 0.0)
            
        elif my_queue_count > 0:
            if line_status.get(self.my_line_id) == True:
                print(f"✋ 내 라인({self.my_line_id}) 작업 대기 중 (Occupied)...")
                return
            
            print(f'🏭 내 라인({self.my_line_id}) 작업 시작')
            
        elif other_queue_count > 0:
            if line_status.get(self.other_line_id) == True:
                print(f"✋ {self.other_line_id}번 지원 대기 중 (Occupied)...")
                return

            print(f"🤝 {self.other_line_id}번 라인 지원 출발")
            self.move_and_wait(-0.56, -0.04, 0.0)
        else:
            pass

    def move_and_wait(self, x, y, yaw):
        self.nav.go_to_pose(x, y, yaw)
        
        while not self.nav.isTaskComplete():
            time.sleep(0.1)
        
        print("✅ 도착 완료 (Action Complete)")