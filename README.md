# Embedded Systems for Autonomous Drones

This project implements an autonomous object-following drone system using embedded platforms like Raspberry Pi, Pixhawk/SpeedyBee flight controllers, and computer vision models. The drone detects a person using a camera, estimates distance using synthetic stereo vision, and follows the person using MAVLink commands.

Originally designed as a two-drone surveillance-and-response system, this implementation focuses on the **object-following drone**, due to no-fly zone constraints.


# Project Structure

 main.py                      # Main FSM controller for takeoff, tracking, landing
 camera_yolo_stream.py       # Starts MJPEG stream using libcamera-vid (Flask)
|---modules/
    camera.py               # Connects to MJPEG stream and returns OpenCV frames
    detector_mobilenet.py   # Runs YOLOv5 object detection
    control.py              # Handles PID + movement command logic
    drone.py                # Handles MAVLink connection + commands
    vision.py               # Estimates object distance using bbox height
 yolov5s.pt                  # YOLOv5 weights file (place here)
 README.md                   # Project documentation (this file)



# Features

- Real-time object detection (YOLOv5)
- Object tracking and following using PID
- Distance estimation using synthetic stereo vision
- MAVLink-based drone control via Raspberry Pi
- Live camera feed via Flask and libcamera
- Modular Python architecture (easy to extend)


# Hardware Requirements

- Raspberry Pi 4
- Pixhawk 2.4.8 or SpeedyBee F405 V3 Flight Controller
- Pi Camera (or USB webcam)
- ESCs + Brushless motors + GPS module
- LiPo Battery + Frame
- FlySky FS-i6X (for safety override)
- Optional: Ultrasonic sensor (for other modes)


# Software Requirements

Install dependencies:

-bash
-sudo apt update
-sudo apt install python3-pip libatlas-base-dev libopencv-dev
-pip3 install flask opencv-python numpy torch torchvision simple-pid pymavlink requests



# How to Run the Project


1. Start the Camera Stream

bash
python3 camera_yolo_stream.py

- Opens MJPEG stream at `http://<RaspberryPi_IP>:5001/video_feed`

2. Start the Object Following Drone

In a new terminal:

bash
python3 main.py --mode flight --control PID


- FSM will move through `takeoff -> search -> track -> land`
- Person detection will trigger drone to rotate and follow
- Press **q** to manually interrupt


## File Roles Summary

 `main.py` - FSM controller for mission stages 
 `camera_yolo_stream.py` - Streams MJPEG video from Pi camera 
 `camera.py` - Connects to Flask stream and decodes OpenCV frames 
 `detector_mobilenet.py`- Runs YOLOv5 to detect person class 
 `control.py` - Calculates PID output and sends movement commands 
 `drone.py` - Handles MAVLink setup, takeoff, land, YAW, XY commands 
 `vision.py` - Computes distance from bbox height 

`drone_protocols folder`- Different autonomous applications for the drone through python scripts
##  Enhancements

- Stereo Vision: Extend `vision.py` to compute depth from two images.
- GPS Localization: Convert image + depth to global coordinates.
- Drone-to-Drone Communication: Send detected object GPS via MAVLink to another drone.


## Acknowledgements

- [ArduPilot Dev Docs](https://ardupilot.org/dev/)
- [PyTorch YOLOv5](https://github.com/ultralytics/yolov5)
- [MAVProxy / pymavlink](https://github.com/ArduPilot/pymavlink)
