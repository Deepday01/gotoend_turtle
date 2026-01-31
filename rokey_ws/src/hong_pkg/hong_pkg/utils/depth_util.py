import numpy as np
import math
import cv2
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.time import Time

class DepthProcessor():
    def __init__(self):
        self.camera_matrix = None
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0

    def set_intrinsics(self, k_list):
        self.camera_matrix = np.array(k_list).reshape(3, 3)
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]

    # 카메라 렌즈와 평면사이의 z축거리(실제거리랑 다름)
    def get_depth_z(self, depth_img, x, y):
        if depth_img is None:
            return None
        
        if x < 0 or x >= depth_img.shape[1] or y < 0 or y >= depth_img.shape[0]:
            return None
        
        z = float(depth_img[y, x]) / 1000.0

        if 0.2 < z < 5.0:
            return z

        return None

    # 카메라 렌즈 사이의 진짜거리
    def get_real_distance(self, x, y, z):
        if self.camera_matrix is None or z is None:
            return None

        X = (x - self.cx) * z / self.fx
        Y = (y - self.cy) * z / self.fy
        Z = z

        distance = math.sqrt(X**2 + Y**2 + Z**2)
        
        return distance

    # 카메라 기준 xyz 좌표
    # 카메라: Z가 앞, X가 오른쪽, Y가 아래.
    # 로봇 몸체(Base_link): X가 앞, Y가 왼쪽, Z가 위.
    def get_XYZ(self, x, y, z):
        if self.camera_matrix is None:
            print("카메라 행렬값이 없습니다.")
            return None

        X = (x - self.cx) * z / self.fx
        Y = (y - self.cy) * z / self.fy
        Z = z
        return (X, Y, Z)

    def get_map_xyz(self, tf_buffer, source_point_3d, source_frame, target_frame='map'):
        try:
            pt_camera = PointStamped()
            pt_camera.header.stamp = Time().to_msg()
            pt_camera.header.frame_id = source_frame
            pt_camera.point.x = source_point_3d[0]
            pt_camera.point.y = source_point_3d[1]
            pt_camera.point.z = source_point_3d[2]

            pt_map = tf_buffer.transform(
                pt_camera,
                target_frame,
                timeout=Duration(seconds=0.1)
            )
            return pt_map
        except Exception as e:
            print(f"TF Transform Error: {e}")
            return None

    def get_xy_transform(self, tf_buffer, depth_img, x, y, source_frame, target_frame='map'):
        try:
            z = self.get_depth_z(depth_img, x, y)
            if z is None:
                return None
            source_point_3d = self.get_XYZ(x, y, z)
            pt_map = self.get_map_xyz(tf_buffer, source_point_3d, source_frame, target_frame)
            return pt_map
        except Exception as e:
            print(f"TF Transform Error: {e}")
            return None
