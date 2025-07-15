# detector_mobilenet.py (YOLOv5 version)
# --------------------------------------
# Uses YOLOv5s for real-time object detection (e.g., person)
# --------------------------------------

import numpy as np
import cv2
import time
import torch
from modules import camera

# Load YOLOv5 model (local weights)
model = torch.hub.load('yolov5', 'custom', path='yolov5s.pt', source='local')
model.conf = 0.4  # Confidence threshold

def initialize_detector():
    print("[INFO] YOLOv5 detector initialized.")

def get_detections():
    """
    Returns list of detections, FPS, and the current frame.
    Each detection is an instance of Detection class (same format as before).
    """
    frame = camera.get_video(0)
    if frame is None:
        print("[ERROR] Camera frame is None. Skipping detection.")
        return [], 0, None

    start = time.time()
    results = model(frame)
    end = time.time()
    fps = 1 / (end - start)

    detections = []
    h, w = frame.shape[:2]

    for *box, conf, cls in results.xyxy[0]:
        class_id = int(cls)
        label = model.names[class_id]

        if conf > 0.5 and label == 'person':  # Track only 'person'
            left = int(box[0])
            top = int(box[1])
            right = int(box[2])
            bottom = int(box[3])
            detections.append(Detection(left, top, right, bottom))

    return detections, round(fps, 2), frame

class Detection:
    def __init__(self, left, top, right, bottom):
        self.Left = left
        self.Top = top
        self.Right = right
        self.Bottom = bottom
        self.Center = ((left + right) // 2, (top + bottom) // 2)

