import math

class MathProcessor():
    def __init__(self):
        pass

    def quaternion_to_orientation_yaw(self, q):
        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_z = math.atan2(t3, t4)
        return yaw_z

    def get_standoff_goal(self, robot_x, robot_y, pt_map, distance=0.6):
        P_robot = np.array([robot_x, robot_y])
        P_point = np.array([pt_map.point.x, pt_map.point.y])

        v = P_robot - P_point
        dist = np.linalg.norm(v)
        if dist == 0: 
            return P_robot
        u = v / dist

        P_goal = P_point + u * 0.6
        return P_goal

    def quaternion_to_orientation_roll(self, q):
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll_x = math.atan2(t0, t1)
        return roll_x

    def quaternion_to_orientation_pitch(self, q):
        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        return pitch_y