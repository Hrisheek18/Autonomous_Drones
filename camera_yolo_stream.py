from flask import Flask, Response
import subprocess
import threading
import sys
import cv2
import numpy as np

app = Flask(__name__)

process = None
data_buffer = b""

# ---------------------------------------
# Start libcamera-vid as MJPEG pipe
# ---------------------------------------
def start_camera_process():
    global process
    cmd = [
        'libcamera-vid', '--inline', '--nopreview', '-t', '0',
        '--framerate', '8',  # ✅ Reduce frame rate to ease buffering
        '--width', '640', '--height', '480',
        '--codec', 'mjpeg', '-o', '-'
    ]
    try:
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        print("[INFO] libcamera-vid process started")
        threading.Thread(target=check_process_errors, daemon=True).start()
    except Exception as e:
        print(f"[ERROR] Failed to start libcamera-vid: {e}")
        sys.exit(1)

# ---------------------------------------
# Watch for libcamera-vid errors
# ---------------------------------------
def check_process_errors():
    global process
    for line in process.stderr:
        decoded = line.decode('utf-8').strip()
        if "ERROR" in decoded:
            print(f"[libcamera-vid ERROR] {decoded}")
            print("[FATAL] Camera could not be acquired. Exiting.")
            process.kill()
            sys.exit(1)

# ---------------------------------------
# Continuously read video bytes
# ---------------------------------------
def read_camera_stream():
    global data_buffer
    while True:
        if process:
            chunk = process.stdout.read(4096)
            if chunk:
                data_buffer += chunk

start_camera_process()
threading.Thread(target=read_camera_stream, daemon=True).start()

# ---------------------------------------
# MJPEG frame generator for browser
# ---------------------------------------
def generate_frames():
    global data_buffer
    while True:
        start = data_buffer.find(b'\xff\xd8')
        end = data_buffer.find(b'\xff\xd9')
        if start != -1 and end != -1 and end > start:
            jpg = data_buffer[start:end+2]
            data_buffer = data_buffer[end+2:]
        else:
            if len(data_buffer) > 1_000_000:
                data_buffer = b""
            continue

        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpg + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    return '''
    <html>
    <head><title>Low-Latency Drone Camera Feed</title></head>
    <body style="text-align:center; background:#111; color:white;">
        <h2>Live Drone Camera (Raw Feed)</h2>
        <img src="/video_feed" width="640" height="480">
    </body>
    </html>
    '''

if __name__ == '__main__':
    print("[INFO] Starting Flask server at http://0.0.0.0:5001")
    app.run(host='0.0.0.0', port=5001, threaded=True)

