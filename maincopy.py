#!/usr/bin/env python3
import os
import time
import signal
import math
import cv2
import mediapipe as mp
import pigpio
from picamera2 import Picamera2
from wheel_handoff import WheelHandoffController
from web_socket import WebSocketServer
from calibration_status import CalibrationWindow
from PySide6.QtWidgets import QApplication
import sys


NO_FACE_MP3 = "no_face.mp3"
FACE_OK_MP3 = "ok_face.mp3"


#audio play

def play_sound(file):
    os.system(f"pkill -f mpg123")
    os.system(f"mpg123 -q {file} > /dev/null 2>&1 &")

# ===================== SETTINGS =====================
PIN_YAW = 24
PIN_PITCH = 25

SERVO_MIN_US, SERVO_MAX_US = 600, 2400
SERVO_MIN_DEG, SERVO_MAX_DEG = 5.0, 170.0
START_YAW_DEG, START_PITCH_DEG = 90.0, 90.0

# FRAME_W, FRAME_H = 640, 360
FRAME_W, FRAME_H = 1280, 720

CAM_FOV_X_DEG, CAM_FOV_Y_DEG = 62.0, 48.8

SMOOTH_ALPHA_CMD = 0.15
SMOOTH_ALPHA_LM = 0.6
STEP_DT = 0.025

MIN_STEP_YAW_DEG = 0.5
MAX_DELTA_YAW = 3.5

MIN_STEP_PITCH_DEG = 0.4
MAX_DELTA_PITCH = 3.5

LOCK_RADIUS_PIX = 75
REACQUIRE_RADIUS_PIX = 120

WINDOW_NAME = "Face Tracking"


# ===================== HELPERS =====================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def deg_to_us(angle_deg):
    return int(SERVO_MIN_US + (angle_deg/180.0) * (SERVO_MAX_US - SERVO_MIN_US))


# 🔥 NEW: quantization
def quantize_angle(angle, step=1.2):
    return round(angle / step) * step


class Servo:
    def __init__(self, pi, pin, start_deg):
        self.pi = pi
        self.pin = pin
        self.deg = start_deg
        self.pi.set_mode(pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(pin, deg_to_us(self.deg))

    def set_angle(self, deg):
        self.deg = clamp(deg, SERVO_MIN_DEG, SERVO_MAX_DEG)
        self.pi.set_servo_pulsewidth(self.pin, deg_to_us(self.deg))
        return self.deg

    def stop(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)


# ===================== ANGLE MAPPING =====================
def pixel_to_angle(cx, cy):
    nx = (cx / FRAME_W) - 0.5
    ny = (cy / FRAME_H) - 0.5

    nx = nx * 1.5
    ny = ny * 1.5

    yaw = nx * CAM_FOV_X_DEG
    pitch = ny * CAM_FOV_Y_DEG

    return yaw, pitch


# ===================== FACE DETECTION =====================
mp_fd = mp.solutions.face_detection
face_det = mp_fd.FaceDetection(0, 0.5)

# def get_face_center(frame):

def get_face_center(frame):

    if frame is None:
        return None

    if frame.size == 0:
        return None

    rgb = frame

    res = face_det.process(rgb)

    if not res or not res.detections:
        return None

    # ✅ ADD THIS PART (MISSING)
    detection = res.detections[0]
    bbox = detection.location_data.relative_bounding_box

    cx = int((bbox.xmin + bbox.width / 2) * FRAME_W)
    cy = int((bbox.ymin + bbox.height / 2) * FRAME_H)

    return cx, cy


# ===================== INIT =====================
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
    main={"size": (FRAME_W, FRAME_H), "format": "RGB888"}))
picam2.start()

pi = pigpio.pi()
servo_yaw = Servo(pi, PIN_YAW, START_YAW_DEG)
servo_pitch = Servo(pi, PIN_PITCH, START_PITCH_DEG)

yaw_deg = START_YAW_DEG
pitch_deg = START_PITCH_DEG

handoff = WheelHandoffController(pi)

ax_f, ay_f = FRAME_W/2, FRAME_H/2
moving = False

last_yaw_update = time.time()
YAW_UPDATE_INTERVAL = 0.15

# 🔥 NEW: last sent value
last_sent_yaw = None
last_face_state = True
no_face_start_time = None
NO_FACE_DELAY = 2.0
ws = WebSocketServer()
ws.start()

# calibration window
app = QApplication(sys.argv)
window = CalibrationWindow(ws)
window.show()


# ===================== LOOP =====================
running = True
signal.signal(signal.SIGINT, lambda *_: globals().__setitem__('running', False))

# cv2.namedWindow(WINDOW_NAME)

while running:

    frame = cv2.flip(picam2.capture_array(), -1)

    if frame is None or frame.size == 0:
        continue

    # center = get_face_center(frame)
    try:
        center = get_face_center(frame)
    except Exception as e:
        print("Face detection error:", e)
        center = None

    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    ws.update_frame(frame)

    if center:
        # ===== FACE DETECTED EVENT =====
        no_face_start_time = None

        if last_face_state == False:
            play_sound(FACE_OK_MP3)

        last_face_state = True
        cx, cy = center

        ax_f = SMOOTH_ALPHA_LM*ax_f + (1-SMOOTH_ALPHA_LM)*cx
        ay_f = SMOOTH_ALPHA_LM*ay_f + (1-SMOOTH_ALPHA_LM)*cy

        dx = ax_f - FRAME_W/2
        dy = ay_f - FRAME_H/2
        r = math.hypot(dx, dy)

        if abs(dx) < 8:
            dx = 0
        if abs(dy) < 6:
            dy = 0

        advice = handoff.update(
            yaw_deg=yaw_deg,
            yaw_err_deg=dx,
            face_r_pix=r,
            lock_radius_pix=LOCK_RADIUS_PIX,
            reacquire_radius_pix=REACQUIRE_RADIUS_PIX,
            face_present=True,
            dt=STEP_DT
        )

        if advice["mode"] == "chassis":
            yaw_deg = servo_yaw.set_angle(advice["yaw_hold_deg"])
            moving = False

        if not moving and r > REACQUIRE_RADIUS_PIX:
            moving = True
        elif moving and r <= LOCK_RADIUS_PIX:
            moving = False

        if moving and advice["mode"] != "chassis":

            yaw_offset, pitch_offset = pixel_to_angle(ax_f, ay_f)

            yaw_target = START_YAW_DEG + (1.2 * yaw_offset)
            pitch_target = START_PITCH_DEG + (1.2 * pitch_offset)

            target_yaw = yaw_deg * SMOOTH_ALPHA_CMD + yaw_target * (1 - SMOOTH_ALPHA_CMD)
            target_pitch = pitch_deg * SMOOTH_ALPHA_CMD + pitch_target * (1 - SMOOTH_ALPHA_CMD)

            # ================= YAW FINAL =================
            dyy = target_yaw - yaw_deg
            dyy = dyy * 0.6

            if abs(dyy) < 1.8:
                dyy = 0

            dyy = clamp(dyy, -MAX_DELTA_YAW, MAX_DELTA_YAW)

            current_time = time.time()

            if current_time - last_yaw_update > YAW_UPDATE_INTERVAL:
                if abs(dyy) >= MIN_STEP_YAW_DEG:

                    new_yaw = yaw_deg + dyy
                    new_yaw = quantize_angle(new_yaw, 1.2)

                    # 🔥 prevent repeated commands
                    if last_sent_yaw is None or abs(new_yaw - last_sent_yaw) >= 1.0:
                        yaw_deg = servo_yaw.set_angle(new_yaw)
                        last_sent_yaw = new_yaw
                        last_yaw_update = current_time

            # ================= PITCH =================
            dpp = target_pitch - pitch_deg

            if abs(dpp) < 0.2:
                dpp = 0

            dpp = clamp(dpp, -MAX_DELTA_PITCH, MAX_DELTA_PITCH)

            if abs(dpp) >= MIN_STEP_PITCH_DEG:
                pitch_deg = servo_pitch.set_angle(pitch_deg + dpp)

        #cv2.circle(frame, (int(ax_f), int(ay_f)), 5, (0,255,0), -1)

    else:
        handoff.update(
            yaw_deg=yaw_deg,
            yaw_err_deg=0,
            face_r_pix=9999,
            lock_radius_pix=LOCK_RADIUS_PIX,
            reacquire_radius_pix=REACQUIRE_RADIUS_PIX,
            face_present=False,
            dt=STEP_DT
        )

        # ===== NO FACE EVENT =====
        if no_face_start_time is None:
            no_face_start_time = time.time()

        if time.time() - no_face_start_time > NO_FACE_DELAY:
            if last_face_state == True:
                play_sound(NO_FACE_MP3)
                last_face_state = False

    # cv2.imshow(WINDOW_NAME, frame)

    if cv2.waitKey(1) == 27:
        break

    time.sleep(STEP_DT)

    app.processEvents()


# ===================== CLEANUP =====================
servo_yaw.stop()
servo_pitch.stop()
handoff.stop()
pi.stop()
picam2.stop()
cv2.destroyAllWindows()