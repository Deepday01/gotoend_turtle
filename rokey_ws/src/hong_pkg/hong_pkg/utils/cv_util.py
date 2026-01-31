import cv2
import numpy as np

class CVProcessor():
    def __init__(self):
        self.colors = {
            'red': (0, 0, 255), 'green': (0, 255, 0), 'blue': (255, 0, 0),
            'yellow': (0, 255, 255), 'white': (255, 255, 255), 'black': (0, 0, 0)
        }
        
    def show_single(self, window_name, img, width=640, height=480):
        if img is None: return

        if img.shape[1] != width or img.shape[0] != height:
            img_resized = cv2.resize(img, (width, height))
        else:
            img_resized = img

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, width, height)
        cv2.imshow(window_name, img_resized)

    def show_dual(self, window_name, img_left, img_right, total_width=1280, height=480):

        if img_left is None or img_right is None: return

        half_width = total_width // 2
        r1 = cv2.resize(img_left, (half_width, height))
        r2 = cv2.resize(img_right, (half_width, height))
        combined = np.hstack((r1, r2))

        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, total_width, height)
        cv2.imshow(window_name, combined)