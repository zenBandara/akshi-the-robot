#!/usr/bin/env python3
# Face position + pose from MediaPipe FaceMesh; servo centering on GPIO23/24 (pigpio)

import time, signal
import cv2
import mediapipe as mp
import numpy as np
import pigpio
from picamera2 import Picamera2

# ===================== USER SETTINGS =====================
# Servos (BCM pins)
PIN_YAW   = 23   # left-right
PIN_PITCH = 24   # up-down

# Servo pulse + angle limits (tune to your rig)
SERVO_MIN_US, SERVO_MAX_US = 600, 2400
SERVO_MIN_DEG, SERVO_MAX_DEG = 10.0, 170.0
START_YAW_DEG, START_PITCH_DEG = 90.0, 90.0

# Camera
FRAME_W, FRAME_H = 640, 480
PROC_SCALE       = 0.5   # run FaceMesh at 320x240, map back to 640x480

# Controller (PD + smoothing)
CAM_FOV_X_DEG, CAM_FOV_Y_DEG = 62.0, 48.8
K_YAW, K_PITCH   = 1.6, 1.6
D_YAW, D_PITCH   = 0.18, 0.18
USE_PD           = True
SMOOTH_ALPHA_CMD = 0.55       # lower = snappier
SMOOTH_ALPHA_LM  = 0.45       # landmark smoothing
DEADBAND_PIX     = 6          # ignore tiny jitter near center
STEP_DT          = 0.015      # ~66 Hz loop
MIN_STEP_YAW_DEG, MIN_STEP_PITCH_DEG = 0.25, 0.20
MAX_DELTA_YAW, MAX_DELTA_PITCH = 3.0, 3.0
INVERT_YAW, INVERT_PITCH = False, False

# Drawing toggles
DRAW_MESH = True
DRAW_ALL_DOTS = False   # set True to draw 468 tiny dots (slower)
DRAW_BOX  = True        # draw bbox from landmarks
SHOW_POSE = True        # compute & draw yaw/pitch/roll

WINDOW_NAME = "FaceMesh Face Position + Servo (GPIO23/24)"

# ===================== HELPERS =====================
def clamp(v, lo, hi): return max(lo, min(hi, v))
def deg_to_us(angle_deg: float) -> int:
    a = clamp(angle_deg, 0.0, 180.0)
    return int(SERVO_MIN_US + (a/180.0) * (SERVO_MAX_US - SERVO_MIN_US))

class Servo:
    def __init__(self, pi, pin, start_deg):
        self.pi = pi; self.pin = pin
        self.deg = clamp(start_deg, SERVO_MIN_DEG, SERVO_MAX_DEG)
        self.pi.set_mode(pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(pin, deg_to_us(self.deg))
    def set_angle(self, target_deg):
        self.deg = clamp(target_deg, SERVO_MIN_DEG, SERVO_MAX_DEG)
        self.pi.set_servo_pulsewidth(self.pin, deg_to_us(self.deg))
        return self.deg
    def stop(self): self.pi.set_servo_pulsewidth(self.pin, 0)

# ---- FaceMesh bootstrap ----
def init_facemesh(max_faces=1):
    mp_face_mesh = mp.solutions.face_mesh
    fm = mp_face_mesh.FaceMesh(
        static_image_mode=False,
        max_num_faces=max_faces,
        refine_landmarks=True,    # enables iris landmarks (468 & 473)
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    return fm, mp_face_mesh

# ---- 2D anchor (face position) from landmarks ----
def face_anchor_from_mesh(landmarks, W, H):
    lm = landmarks.landmark
    # Midpoint between pupils (stable face center)
    lx, ly = lm[468].x * W, lm[468].y * H
    rx, ry = lm[473].x * W, lm[473].y * H
    cx, cy = (lx + rx) * 0.5, (ly + ry) * 0.5

    # Bounding box from all normalized points
    xs = np.array([p.x for p in lm]) * W
    ys = np.array([p.y for p in lm]) * H
    x, y = int(xs.min()), int(ys.min())
    w, h = int(xs.max() - xs.min()), int(ys.max() - ys.min())
    return (int(cx), int(cy)), (x, y, w, h)

# ---- Head pose (yaw/pitch/roll) from a few key landmarks ----
def head_pose_from_mesh(landmarks, W, H, focal=800.0):
    lm = landmarks.landmark
    idx = {
        "nose_tip": 1,
        "left_eye_out": 33,
        "right_eye_out": 263,
        "mouth_left": 61,
        "mouth_right": 291,
        "chin": 199
    }
    pts2d = np.array([
        [lm[idx["nose_tip"]].x * W, lm[idx["nose_tip"]].y * H],
        [lm[idx["left_eye_out"]].x * W, lm[idx["left_eye_out"]].y * H],
        [lm[idx["right_eye_out"]].x * W, lm[idx["right_eye_out"]].y * H],
        [lm[idx["mouth_left"]].x * W, lm[idx["mouth_left"]].y * H],
        [lm[idx["mouth_right"]].x * W, lm[idx["mouth_right"]].y * H],
        [lm[idx["chin"]].x * W, lm[idx["chin"]].y * H],
    ], dtype=np.float32)
    pts3d = np.array([
        [0.0,   0.0,   0.0],   # nose
        [-30.0, 35.0, -30.0],  # left eye
        [ 30.0, 35.0, -30.0],  # right eye
        [-25.0,-35.0, -20.0],  # mouth left
        [ 25.0,-35.0, -20.0],  # mouth right
        [ 0.0,-70.0,  10.0],   # chin
    ], dtype=np.float32)

    cx, cy = W/2.0, H/2.0
    K = np.array([[focal, 0, cx],
                  [0, focal, cy],
                  [0,     0,  1]], dtype=np.float32)
    dist = np.zeros(5, dtype=np.float32)

    ok, rvec, tvec = cv2.solvePnP(pts3d, pts2d, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    pitch = np.degrees(np.arctan2(-R[2,0], sy))
    yaw   = np.degrees(np.arctan2(R[1,0], R[0,0]))
    roll  = np.degrees(np.arctan2(R[2,1], R[2,2]))
    return float(yaw), float(pitch), float(roll)

# ---- One-call helper: get face position + bbox + pose from a frame ----
def get_face_pose(face_mesh, frame_rgb, out_w, out_h, proc_scale=0.5):
    if proc_scale != 1.0:
        small = cv2.resize(frame_rgb, (int(out_w*proc_scale), int(out_h*proc_scale)),
                           interpolation=cv2.INTER_AREA)
    else:
        small = frame_rgb

    res = face_mesh.process(small)  # expects RGB
    if not res.multi_face_landmarks:
        return None, None, None, None

    lms = res.multi_face_landmarks[0]  # use first face
    center, bbox = face_anchor_from_mesh(lms, out_w, out_h)
    pose = head_pose_from_mesh(lms, out_w, out_h) if SHOW_POSE else None
    return center, bbox, pose, lms

# ===================== CAMERA =====================
picam2 = Picamera2()
picam2.configure(picam2.create_video_configuration(
    main={"size": (FRAME_W, FRAME_H), "format": "RGB888"},
    buffer_count=4))
picam2.start()

# ===================== pigpio (servos) =====================
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio daemon not running. Start it: sudo systemctl start pigpiod")
servo_yaw   = Servo(pi, PIN_YAW,   START_YAW_DEG)
servo_pitch = Servo(pi, PIN_PITCH, START_PITCH_DEG)
yaw_deg, pitch_deg = START_YAW_DEG, START_PITCH_DEG

# ===================== FaceMesh + drawing utils =====================
face_mesh, mp_face_mesh = init_facemesh()
mp_draw = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles

# Filtered anchor and PD memory
ax_f, ay_f = FRAME_W/2.0, FRAME_H/2.0
prev_yaw_err, prev_pitch_err = 0.0, 0.0

# ===================== LOOP =====================
running = True
signal.signal(signal.SIGINT, lambda *_: globals().__setitem__('running', False))
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

print("FaceMesh face position + servo centering. ESC/Ctrl+C to quit.")
try:
    while running:
        # Capture RGB
        frame_rgb = picam2.capture_array()
        center, bbox, pose, lms = get_face_pose(face_mesh, frame_rgb, FRAME_W, FRAME_H, PROC_SCALE)

        # Convert to BGR for drawing
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        # Draw landmarks/mesh
        if lms is not None and DRAW_MESH:
            mp_draw.draw_landmarks(
                image=frame_bgr,
                landmark_list=lms,
                connections=mp_face_mesh.FACEMESH_TESSELATION,
                landmark_drawing_spec=None,
                connection_drawing_spec=mp_styles.get_default_face_mesh_tesselation_style(),
            )
            mp_draw.draw_landmarks(
                image=frame_bgr,
                landmark_list=lms,
                connections=mp_face_mesh.FACEMESH_CONTOURS,
                landmark_drawing_spec=None,
                connection_drawing_spec=mp_styles.get_default_face_mesh_contours_style(),
            )
            mp_draw.draw_landmarks(
                image=frame_bgr,
                landmark_list=lms,
                connections=mp_face_mesh.FACEMESH_IRISES,
                landmark_drawing_spec=None,
                connection_drawing_spec=mp_styles.get_default_face_mesh_iris_connections_style(),
            )
            if DRAW_ALL_DOTS:
                for p in lms.landmark:
                    cx = int(p.x * FRAME_W); cy = int(p.y * FRAME_H)
                    if 0 <= cx < FRAME_W and 0 <= cy < FRAME_H:
                        cv2.circle(frame_bgr, (cx, cy), 1, (0, 255, 0), -1)

        # Control using the anchor (midpoint of irises)
        if center is not None:
            cx, cy = center

            # Deadband near image center
            if abs(cx - FRAME_W/2) <= DEADBAND_PIX: cx = FRAME_W/2
            if abs(cy - FRAME_H/2) <= DEADBAND_PIX: cy = FRAME_H/2

            # Smooth anchor
            ax_f = SMOOTH_ALPHA_LM*ax_f + (1.0 - SMOOTH_ALPHA_LM)*cx
            ay_f = SMOOTH_ALPHA_LM*ay_f + (1.0 - SMOOTH_ALPHA_LM)*cy

            # Angular error (right/down positive)
            ex = (ax_f - FRAME_W/2) / (FRAME_W/2)
            ey = (ay_f - FRAME_H/2) / (FRAME_H/2)
            yaw_err   = ex * (CAM_FOV_X_DEG / 2.0)
            pitch_err = ey * (CAM_FOV_Y_DEG / 2.0)
            if INVERT_YAW:   yaw_err   *= -1
            if INVERT_PITCH: pitch_err *= -1

            # PD controller
            if USE_PD:
                dyaw   = (yaw_err   - prev_yaw_err)   / STEP_DT
                dpitch = (pitch_err - prev_pitch_err) / STEP_DT
                cmd_yaw   = K_YAW*yaw_err   + D_YAW*dyaw
                cmd_pitch = K_PITCH*pitch_err + D_PITCH*dpitch
            else:
                cmd_yaw, cmd_pitch = K_YAW*yaw_err, K_PITCH*pitch_err

            # Smooth command + rate limit + apply
            target_yaw   = yaw_deg   * SMOOTH_ALPHA_CMD + (yaw_deg   + cmd_yaw)   * (1 - SMOOTH_ALPHA_CMD)
            target_pitch = pitch_deg * SMOOTH_ALPHA_CMD + (pitch_deg + cmd_pitch) * (1 - SMOOTH_ALPHA_CMD)
            dy = clamp(target_yaw   - yaw_deg,   -MAX_DELTA_YAW,   +MAX_DELTA_YAW)
            dp = clamp(target_pitch - pitch_deg, -MAX_DELTA_PITCH, +MAX_DELTA_PITCH)

            if abs(dy) >= MIN_STEP_YAW_DEG:   yaw_deg   = servo_yaw.set_angle(yaw_deg + dy)
            if abs(dp) >= MIN_STEP_PITCH_DEG: pitch_deg = servo_pitch.set_angle(pitch_deg + dp)

            prev_yaw_err, prev_pitch_err = yaw_err, pitch_err

            # Draw anchor and bbox
            cv2.circle(frame_bgr, (int(ax_f), int(ay_f)), 5, (0,255,0), -1)
            if DRAW_BOX and bbox is not None:
                x,y,w,h = bbox
                cv2.rectangle(frame_bgr, (x,y), (x+w, y+h), (0,255,0), 2)

        else:
            # No face: hold last position; overlay message
            cv2.putText(frame_bgr, "NO FACE", (FRAME_W-220, 40),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2)

        # Pose text (optional)
        if SHOW_POSE and (pose is not None):
            yaw, pitch, roll = pose
            cv2.putText(frame_bgr, f"Yaw:{yaw:5.1f}  Pitch:{pitch:5.1f}  Roll:{roll:5.1f}",
                        (10, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

        # HUD + window
        cv2.drawMarker(frame_bgr, (FRAME_W//2, FRAME_H//2), (255,255,255),
                       markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)
        cv2.putText(frame_bgr, f"SrvYaw:{yaw_deg:5.1f}  SrvPitch:{pitch_deg:5.1f}",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        cv2.imshow(WINDOW_NAME, frame_bgr)
        if (cv2.waitKey(1) & 0xFF) == 27:  # ESC
            break

        time.sleep(STEP_DT)

except Exception as e:
    print("Error:", e)

finally:
    try: cv2.destroyAllWindows()
    except: pass
    try: picam2.stop(); picam2.close()
    except: pass
    try: servo_yaw.stop(); servo_pitch.stop()
    except: pass
    try: pi.stop()
    except: pass
    print("Shutdown complete.")
