# camera.py
# -------------------------------------------
# Uses camera_stream.py for Pi Camera feed
# -------------------------------------------

import requests
import numpy as np
import cv2

# URL of the running camera_stream.py server
STREAM_URL = "http://127.0.0.1:5001/video_feed"


# Persistent stream object to avoid reconnecting every frame
stream = None
bytes_ = bytes()

def create_camera(camera_id=0):
    global stream
    print("[INFO] Initializing camera stream from", STREAM_URL)
    try:
        stream = requests.get(STREAM_URL, stream=True, timeout=5)
        if stream.status_code != 200:
            raise RuntimeError("Failed to connect to camera stream")
        print("[INFO] Camera stream connected.")
    except Exception as e:
        print(f"[ERROR] Could not connect to camera stream: {e}")
        stream = None

def get_video(camera_id=0):
    """
    Reads a single frame from the Flask camera stream.
    """
    global stream, bytes_
    if stream is None:
        create_camera()

    if stream is None:
        print("[ERROR] Camera stream not available.")
        return None

    try:
        for chunk in stream.iter_content(chunk_size=1024):
            bytes_ += chunk
            a = bytes_.find(b'\xff\xd8')
            b = bytes_.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes_[a:b+2]
                bytes_ = bytes_[b+2:]
                img_array = np.frombuffer(jpg, dtype=np.uint8)
                frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                if frame is not None:
                    return frame
                else:
                    print("[ERROR] Failed to decode frame.")
                    return None
    except Exception as e:
        print(f"[ERROR] Exception while reading camera stream: {e}")
        stream = None
        return None

def get_image_size(camera_id=0):
    # Returns default stream resolution
    return 640, 480

def close_cameras():
    global stream
    if stream:
        stream.close()
    stream = None
    print("[INFO] Camera stream closed.")


