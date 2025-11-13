#!/usr/bin/env python3
# MediaPipe (FaceDetection or FaceMesh) + pigpio servos on GPIO23/24
# Adds a fair-radius hold zone with hysteresis so motors stop near center.

import time, signal, math
import cv2
import mediapipe as mp
import numpy as np
import pigpio
from picamera2 import Picamera2
from wheel_handoff import WheelHandoffController   # <-- INSERTED



# ===================== USER SETTINGS =====================
# Servos (BCM pins)
PIN_YAW   = 24   # left-right
PIN_PITCH = 23   # up-down

# Servo pulse + angle limits (tune to your rig)
SERVO_MIN_US, SERVO_MAX_US = 600, 2400
SERVO_MIN_DEG, SERVO_MAX_DEG = 5.0, 170.0
START_YAW_DEG, START_PITCH_DEG = 90.0, 90.0

# Camera
FRAME_W, FRAME_H = 1000, 480

# ---- SPEED MODE FLAGS ----
USE_FACE_DETECTION = True       # True = fast box center; False = FaceMesh tracking
REFINE_LANDMARKS   = False      # only used when USE_FACE_DETECTION=False
PROC_SCALE         = 0.5       # 0.35?0.5 recommended

# Controller (PD + smoothing)
CAM_FOV_X_DEG, CAM_FOV_Y_DEG = 62.0, 48.8
K_YAW, K_PITCH   = 1.6, 1.6
D_YAW, D_PITCH   = 0.18, 0.18
USE_PD           = True
SMOOTH_ALPHA_CMD = 0.01      # lower = snappier
SMOOTH_ALPHA_LM  = 0.4       # center smoothing
STEP_DT          = 0.015      # ~66 Hz loop 0.015
MIN_STEP_YAW_DEG, MIN_STEP_PITCH_DEG = 0.25, 0.20
MAX_DELTA_YAW, MAX_DELTA_PITCH = 3.0, 3.0
INVERT_YAW, INVERT_PITCH = False, False

# ---- HOLD-ZONE (what you asked for) ----
# Motors stop when face is inside LOCK_RADIUS; resume only if it leaves REACQUIRE_RADIUS
LOCK_RADIUS_PIX      = 75     # inner radius to STOP updates
REACQUIRE_RADIUS_PIX = 120    # outer radius to START updates (must be > LOCK)

# Drawing toggles
DRAW_MESH = False
DRAW_ALL_DOTS = False
DRAW_BOX  = True
SHOW_POSE = False
DRAW_RADII = True

WINDOW_NAME = "MP Face Tracking (GPIO23/24)"

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

# ---- Face init (Detection or Mesh) ----
mp_fd = mp.solutions.face_detection
mp_fm = mp.solutions.face_mesh
mp_draw = mp.solutions.drawing_utils
mp_styles = mp.solutions.drawing_styles

if USE_FACE_DETECTION:
    face_det = mp_fd.FaceDetection(model_selection=0, min_detection_confidence=0.5)
    face_mesh = None
else:
    face_mesh = mp_fm.FaceMesh(
        static_image_mode=False,
        max_num_faces=1,
        refine_landmarks=REFINE_LANDMARKS,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )
    face_det = None

def head_pose_from_mesh(landmarks, W, H, focal=800.0):
    lm = landmarks.landmark
    idx = {"nose_tip":1, "left_eye_out":33, "right_eye_out":263, "mouth_left":61, "mouth_right":291, "chin":199}
    pts2d = np.array([
        [lm[idx["nose_tip"]].x * W, lm[idx["nose_tip"]].y * H],
        [lm[idx["left_eye_out"]].x * W, lm[idx["left_eye_out"]].y * H],
        [lm[idx["right_eye_out"]].x * W, lm[idx["right_eye_out"]].y * H],
        [lm[idx["mouth_left"]].x * W, lm[idx["mouth_left"]].y * H],
        [lm[idx["mouth_right"]].x * W, lm[idx["mouth_right"]].y * H],
        [lm[idx["chin"]].x * W, lm[idx["chin"]].y * H],
    ], dtype=np.float32)
    pts3d = np.array([
        [0.0,0.0,0.0], [-30.0,35.0,-30.0], [30.0,35.0,-30.0],
        [-25.0,-35.0,-20.0], [25.0,-35.0,-20.0], [0.0,-70.0,10.0],
    ], dtype=np.float32)
    cx, cy = W/2.0, H/2.0
    K = np.array([[focal,0,cx],[0,focal,cy],[0,0,1]], dtype=np.float32)
    dist = np.zeros(5, dtype=np.float32)
    ok, rvec, _ = cv2.solvePnP(pts3d, pts2d, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok: return None
    R,_ = cv2.Rodrigues(rvec); sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    pitch = np.degrees(np.arctan2(-R[2,0], sy))
    yaw   = np.degrees(np.arctan2(R[1,0], R[0,0]))
    roll  = np.degrees(np.arctan2(R[2,1], R[2,2]))
    return float(yaw), float(pitch), float(roll)

def get_face_center(frame_rgb, out_w, out_h):
    if PROC_SCALE != 1.0:
        small = cv2.resize(frame_rgb, (int(out_w*PROC_SCALE), int(out_h*PROC_SCALE)), interpolation=cv2.INTER_AREA)
    else:
        small = frame_rgb

    if USE_FACE_DETECTION:
        res = face_det.process(small)
        if not res.detections:
            return None, None, None, None
        det = res.detections[0]
        rb = det.location_data.relative_bounding_box
        x, y = int(rb.xmin * out_w), int(rb.ymin * out_h)
        w, h = int(rb.width * out_w), int(rb.height * out_h)
        cx, cy = x + w//2, y + h//2
        return (cx, cy), (x, y, w, h), None, None
    else:
        res = face_mesh.process(small)
        if not res.multi_face_landmarks:
            return None, None, None, None
        lms = res.multi_face_landmarks[0]
        if REFINE_LANDMARKS:
            lx, ly = lms.landmark[468].x * out_w, lms.landmark[468].y * out_h
            rx, ry = lms.landmark[473].x * out_w, lms.landmark[473].y * out_h
        else:
            lx, ly = lms.landmark[33].x  * out_w, lms.landmark[33].y  * out_h
            rx, ry = lms.landmark[263].x * out_w, lms.landmark[263].y * out_h
        cx, cy = int((lx + rx) * 0.5), int((ly + ry) * 0.5)
        xs = np.array([p.x for p in lms.landmark]) * out_w
        ys = np.array([p.y for p in lms.landmark]) * out_h
        x, y = int(xs.min()), int(ys.min())
        w, h = int(xs.max() - xs.min()), int(ys.max() - ys.min())
        pose = head_pose_from_mesh(lms, out_w, out_h) if SHOW_POSE else None
        return (cx, cy), (x, y, w, h), pose, lms



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
handoff = WheelHandoffController(pi)   # <-- INSERTED

# Filtered anchor and PD memory
ax_f, ay_f = FRAME_W/2.0, FRAME_H/2.0
prev_yaw_err, prev_pitch_err = 0.0, 0.0
moving = False  # << hysteresis state (stop inside lock radius; move outside reacquire radius)

# ===================== LOOP =====================
running = True
signal.signal(signal.SIGINT, lambda *_: globals().__setitem__('running', False))
cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

print("Tracking with fair-radius hold. ESC/Ctrl+C to quit.")
try:
    while running:
        frame_rgb = picam2.capture_array()

        center, bbox, pose, lms = get_face_center(frame_rgb, FRAME_W, FRAME_H)

        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
         # Flip vertically (up-down)
        flipped = cv2.flip(frame_bgr, 0)   # 0 = vertical, 1 = horizontal, -1 = both
        # flipped = cv2.flip(flipped1, 1)

        # ---- Control ----
        if center is not None:
            cx, cy = center

            # Smooth center (light smoothing to reduce noise)
            ax_f = SMOOTH_ALPHA_LM*ax_f + (1.0 - SMOOTH_ALPHA_LM)*cx
            ay_f = SMOOTH_ALPHA_LM*ay_f + (1.0 - SMOOTH_ALPHA_LM)*cy

            # Distance from image center (pixels)
            dx = ax_f - FRAME_W/2.0
            dy = ay_f - FRAME_H/2.0
            r  = math.hypot(dx, dy)

            # ------ INSERT: wheel handoff update ------
            ex_handoff = dx / (FRAME_W/2.0)
            yaw_err_handoff = ex_handoff * (CAM_FOV_X_DEG / 2.0)

            advice = handoff.update(
                yaw_deg=yaw_deg,
                yaw_err_deg=yaw_err_handoff,
                face_r_pix=r,
                lock_radius_pix=LOCK_RADIUS_PIX,
                reacquire_radius_pix=REACQUIRE_RADIUS_PIX,
                face_present=True,
                dt=STEP_DT
            )

            if advice["mode"] == "chassis":
                # hold head at center while base turns
                yaw_deg = servo_yaw.set_angle(advice["yaw_hold_deg"])
                moving = False  # request to skip PD this frame
            # ------------------------------------------

            # Hysteresis: decide whether to move servos
            if not moving and r > REACQUIRE_RADIUS_PIX:
                moving = True
            elif moving and r <= LOCK_RADIUS_PIX:
                moving = False

            # Draw lock/reacquire rings
            if DRAW_RADII:
                cv2.circle(flipped, (FRAME_W//2, FRAME_H//2), LOCK_RADIUS_PIX, (0,255,0), 1)
                cv2.circle(flipped, (FRAME_W//2, FRAME_H//2), REACQUIRE_RADIUS_PIX, (0,165,255), 1)

            # If moving, run PD; otherwise, hold last servo angles
            if moving and advice["mode"] != "chassis":   # <-- ensure head PD is skipped during chassis turn
                ex = dx / (FRAME_W/2.0)
                ey = dy / (FRAME_H/2.0)
                yaw_err   = ex * (CAM_FOV_X_DEG / 2.0)
                pitch_err = ey * (CAM_FOV_Y_DEG / 2.0)
                if INVERT_YAW:   yaw_err   *= -1
                if INVERT_PITCH: pitch_err *= -1

                if USE_PD:
                    dyaw   = (yaw_err   - prev_yaw_err)   / STEP_DT
                    dpitch = (pitch_err - prev_pitch_err) / STEP_DT
                    cmd_yaw   = K_YAW*yaw_err   + D_YAW*dyaw
                    cmd_pitch = K_PITCH*pitch_err + D_PITCH*dpitch
                else:
                    cmd_yaw, cmd_pitch = K_YAW*yaw_err, K_PITCH*pitch_err

                target_yaw   = yaw_deg   * SMOOTH_ALPHA_CMD + (yaw_deg   + cmd_yaw)   * (1 - SMOOTH_ALPHA_CMD)
                target_pitch = pitch_deg * SMOOTH_ALPHA_CMD + (pitch_deg + cmd_pitch) * (1 - SMOOTH_ALPHA_CMD)
                dyy = clamp(target_yaw   - yaw_deg,   -MAX_DELTA_YAW,   +MAX_DELTA_YAW)
                dpp = clamp(target_pitch - pitch_deg, -MAX_DELTA_PITCH, +MAX_DELTA_PITCH)

                if abs(dyy) >= MIN_STEP_YAW_DEG:   yaw_deg   = servo_yaw.set_angle(yaw_deg - dyy)
                if abs(dpp) >= MIN_STEP_PITCH_DEG: pitch_deg = servo_pitch.set_angle(pitch_deg - dpp)

                prev_yaw_err, prev_pitch_err = yaw_err, pitch_err

            # Draw center/box
            if DRAW_BOX and bbox is not None:
                x,y,w,h = bbox
                cv2.rectangle(flipped, (x,y), (x+w, y+h), (0,255,0), 2)
            cv2.circle(flipped, (int(ax_f), int(ay_f)), 4, (0,255,0) if moving else (0,200,255), -1)
        else:
            cv2.putText(flipped, "NO FACE", (FRAME_W-220, 40),
                        cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2)

            # ------ INSERT: run failsafe timing while no face ------
            _ = handoff.update(
                yaw_deg=yaw_deg,
                yaw_err_deg=0.0,
                face_r_pix=9999.0,
                lock_radius_pix=LOCK_RADIUS_PIX,
                reacquire_radius_pix=REACQUIRE_RADIUS_PIX,
                face_present=False,
                dt=STEP_DT
            )
            # -------------------------------------------------------

        # HUD
        cv2.drawMarker(flipped, (FRAME_W//2, FRAME_H//2), (255,255,255),
                       markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)
        cv2.putText(flipped, f"SrvYaw:{yaw_deg:5.1f}  SrvPitch:{pitch_deg:5.1f}  {'MOVE' if moving else 'HOLD'}",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)

        cv2.imshow(WINDOW_NAME, flipped)
        if (cv2.waitKey(1) & 0xFF) == 27:
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
    try: handoff.stop()     # <-- INSERTED
    except: pass
    try: pi.stop()
    except: pass
    print("Shutdown complete.")
