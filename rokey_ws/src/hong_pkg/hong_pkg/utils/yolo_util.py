import cv2
import os
import sys
from ultralytics import YOLO

class YOLOProcessor():
    def __init__(self, model_path):
        if not os.path.exists(model_path):
            print(f'{model_path}가 없습니다.')
            sys.exit(1)
        self.model = YOLO(model_path)
        self.class_names = self.model.names if hasattr(self.model, 'names') else ['Object']

    def get_detect(self, img):
        results = self.model.track(source=img, stream=True, persist=True, verbose=False)
        detections = []

        for r in results:
            if not hasattr(r, 'boxes') or r.boxes is None:
                print("model이 있는지 확인해주세요")
                continue

            for box in r.boxes:
                # 좌표 및 정보 추출
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0
                track_id = int(box.id[0]) if box.id is not None else -1

                # 데이터 구조화 (필요시 로봇 제어에 사용)
                detections.append({
                    'id': track_id,
                    'class': self.class_names[cls],
                    'box_pos': (x1, y1, x2, y2),
                    'conf': conf
                })
        return detections
    
    def get_detect_info(self, img):
        detections = self.get_detect(img)
        return detections

    def detect_tracking_box(self, img):

        detections = self.get_detect(img)

        for data in detections:
            x1, y1, x2, y2 = data['box_pos']
            track_id = data['id']
            class_name = data['class']
            conf = data['conf']

            label = f"{class_name} ID:{track_id} {conf:.2f}"
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
        return img, detections
    
    def is_bounding_box(self, detections, x, y):
        for data in detections:
            x1, x2, y1, y2 = data['box_pos']
            if x1 <= x <= x2 and y1 <= y <= y2:
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                result = (center_x, center_y)
                return True, result
        return False, None

