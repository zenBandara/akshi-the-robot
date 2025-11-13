
import time, signal, math
import cv2
import mediapipe as mp
import numpy as np
import pigpio
from picamera2 import Picamera2
from wheel_handoff import WheelHandoffController   # <-- INSERTED

# Camera
FRAME_W, FRAME_H = 1000, 480

WINDOW_NAME = "MP Face Tracking (GPIO23/24)"

# ===================== CAMERA =====================
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
    main={"size": (FRAME_W, FRAME_H), "format": "RGB888"},
    buffer_count=4))
picam2.start()

signal.signal(signal.SIGINT, lambda *_: globals().__setitem__('running', False))
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
