from utils.nav_util import NavProcessor
import time


class BatteryProcessor():
    def __init__(self):
        self.nav = NavProcessor()

    def pick_up_waiting(self, battery_percent, my_queue_count, other_queue_count):
        battery = battery_percent * 100

        if battery < 30:
            self.nav.go_to_pose(-0.744339644908905, 1.9917999505996704, 0.0)
            print('go to dock')
            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                time.sleep(0.1)
        elif my_queue_count > 0:
            print('my line start')
            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                time.sleep(0.1)
        elif other_queue_count > 0:
            self.nav.go_to_pose(-0.5631656646728516, -0.0406402051448822, 0.0)
            print("other line start")
            while not self.nav.isTaskComplete():
                feedback = self.nav.getFeedback()
                time.sleep(0.1)
        else:
            print("idle")

