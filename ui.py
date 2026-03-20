#!/usr/bin/env python3
import os
import sys
import json
import subprocess
import threading
import time
import math
import random

import pigpio
import pygame  # audio

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAIN_PATH = os.path.join(BASE_DIR, "main.py")
ANIMATION_FILE = os.path.join(BASE_DIR, "animation.json")
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200

# ================= USER SETTINGS (FROM CHILDLIKE SCRIPT) =================

# Servos (BCM pins)
PIN_YAW = 24      # left right
PIN_PITCH = 23    # up down

# Servo pulse range
SERVO_MIN_US, SERVO_MAX_US = 600, 2400
SERVO_MIN_DEG, SERVO_MAX_DEG = 5.0, 170.0

# Neutral pose
START_YAW_DEG = 90.0
START_PITCH_DEG = 90.0

# Childlike movement range (a bit wider)
IDLE_YAW_MIN = 55.0
IDLE_YAW_MAX = 125.0
IDLE_PITCH_MIN = 65.0
IDLE_PITCH_MAX = 115.0

# Control loop
STEP_DT = 0.015          # 15 ms loop

# Step size limits
MIN_STEP_DEG = 0.15
MAX_STEP_DEG = 1.6       # slightly faster than smooth calm version

# Pause times at each pose (shorter, more active)
PAUSE_MIN = 0.15         # seconds
PAUSE_MAX = 0.60         # seconds

# Micro jitter around pose (small fidgeting)
MICRO_JITTER_DEG = 0.35
MICRO_JITTER_PERIOD = 1.5  # seconds

# Probability of a big turn
BIG_TURN_PROB = 0.15

# ========================================================================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def deg_to_us(angle_deg: float) -> int:
    a = clamp(angle_deg, 0.0, 180.0)
    return int(SERVO_MIN_US + (a / 180.0) * (SERVO_MAX_US - SERVO_MIN_US))


class Servo:
    def __init__(self, pi, pin, start_deg):
        self.pi = pi
        self.pin = pin
        self.deg = clamp(start_deg, SERVO_MIN_DEG, SERVO_MAX_DEG)
        self.pi.set_mode(pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(pin, deg_to_us(self.deg))

    def set_angle(self, target_deg):
        self.deg = clamp(target_deg, SERVO_MIN_DEG, SERVO_MAX_DEG)
        self.pi.set_servo_pulsewidth(self.pin, deg_to_us(self.deg))
        return self.deg

    def stop(self):
        self.pi.set_servo_pulsewidth(self.pin, 0)


# ---------- Childlike pose selection and step (same logic as your script) ----------

def pick_pose(yaw_deg, pitch_deg):
    """
    Childlike pose selection.
    More small moves. Some left right. Some diagonals.
    Sometimes a big turn.
    """
    # Rare big turn
    if random.random() < BIG_TURN_PROB:
        new_yaw = random.uniform(50.0, 130.0)
        new_pitch = random.uniform(70.0, 110.0)
        return new_yaw, new_pitch

    mode = random.choices(
        population=["neutral", "left", "right", "up", "down", "diag", "small"],
        weights=[0.8, 2.5, 2.5, 1.8, 1.8, 2.2, 3.4],
        k=1
    )[0]

    if mode == "neutral":
        new_yaw = random.uniform(85.0, 95.0)
        new_pitch = random.uniform(85.0, 95.0)

    elif mode == "left":
        new_yaw = random.uniform(IDLE_YAW_MIN, 80.0)
        new_pitch = clamp(
            pitch_deg + random.uniform(-4, 4),
            IDLE_PITCH_MIN, IDLE_PITCH_MAX
        )

    elif mode == "right":
        new_yaw = random.uniform(100.0, IDLE_YAW_MAX)
        new_pitch = clamp(
            pitch_deg + random.uniform(-4, 4),
            IDLE_PITCH_MIN, IDLE_PITCH_MAX
        )

    elif mode == "up":
        new_pitch = random.uniform(95.0, IDLE_PITCH_MAX)
        new_yaw = clamp(
            yaw_deg + random.uniform(-7, 7),
            IDLE_YAW_MIN, IDLE_YAW_MAX
        )

    elif mode == "down":
        new_pitch = random.uniform(IDLE_PITCH_MIN, 85.0)
        new_yaw = clamp(
            yaw_deg + random.uniform(-7, 7),
            IDLE_YAW_MIN, IDLE_YAW_MAX
        )

    elif mode == "diag":
        new_yaw = random.uniform(IDLE_YAW_MIN, IDLE_YAW_MAX)
        new_pitch = random.uniform(IDLE_PITCH_MIN, IDLE_PITCH_MAX)

    else:  # "small"
        new_yaw = clamp(
            yaw_deg + random.uniform(-10, 10),
            IDLE_YAW_MIN, IDLE_YAW_MAX
        )
        new_pitch = clamp(
            pitch_deg + random.uniform(-6, 6),
            IDLE_PITCH_MIN, IDLE_PITCH_MAX
        )

    return new_yaw, new_pitch


def compute_step(dist):
    """
    Step size based on distance.
    Slightly more aggressive than calm.
    """
    step = 0.16 * dist
    step = clamp(step, MIN_STEP_DEG, MAX_STEP_DEG)
    return step


# ================== GLOBAL SERVO / ANIMACY STATE ==================

pi = None
servo_yaw = None
servo_pitch = None
yaw_deg = START_YAW_DEG
pitch_deg = START_PITCH_DEG

anim_running = False
anim_thread = None
state_lock = threading.Lock()

# ================================================================

def init_servos():
    global pi, servo_yaw, servo_pitch, yaw_deg, pitch_deg
    if pi is not None:
        return
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running. Use: sudo systemctl start pigpiod")
    yaw_deg = START_YAW_DEG
    pitch_deg = START_PITCH_DEG
    servo_yaw = Servo(pi, PIN_YAW, START_YAW_DEG)
    servo_pitch = Servo(pi, PIN_PITCH, START_PITCH_DEG)


def animacy_loop():
    """Background thread, same behaviour as your main() while-loop."""
    global yaw_deg, pitch_deg, anim_running

    # initial target
    with state_lock:
        cur_yaw = yaw_deg
        cur_pitch = pitch_deg
    target_yaw, target_pitch = pick_pose(cur_yaw, cur_pitch)
    last_micro_time = time.time()

    try:
        while anim_running:
            with state_lock:
                cur_yaw = yaw_deg
                cur_pitch = pitch_deg

            dyaw = target_yaw - cur_yaw
            dpitch = target_pitch - cur_pitch
            dist = math.hypot(dyaw, dpitch)

            # If close to target, short pause with fidgeting
            if dist < 0.5:
                pause_time = random.uniform(PAUSE_MIN, PAUSE_MAX)
                end_pause = time.time() + pause_time

                while anim_running and time.time() < end_pause:
                    now = time.time()
                    if now - last_micro_time > MICRO_JITTER_PERIOD:
                        j_y = random.uniform(-MICRO_JITTER_DEG, MICRO_JITTER_DEG)
                        j_p = random.uniform(-MICRO_JITTER_DEG, MICRO_JITTER_DEG)

                        with state_lock:
                            yaw_deg = servo_yaw.set_angle(cur_yaw + j_y)
                            pitch_deg = servo_pitch.set_angle(cur_pitch + j_p)
                            cur_yaw = yaw_deg
                            cur_pitch = pitch_deg

                        last_micro_time = now

                    time.sleep(0.05)

                if not anim_running:
                    break

                with state_lock:
                    cur_yaw = yaw_deg
                    cur_pitch = pitch_deg
                target_yaw, target_pitch = pick_pose(cur_yaw, cur_pitch)
                continue

            # Step size based on distance
            step_deg = compute_step(dist)

            # Move one step toward target
            step_yaw = clamp(dyaw, -step_deg, step_deg)
            step_pitch = clamp(dpitch, -step_deg, step_deg)

            with state_lock:
                yaw_deg = servo_yaw.set_angle(cur_yaw + step_yaw)
                pitch_deg = servo_pitch.set_angle(cur_pitch + step_pitch)

            time.sleep(STEP_DT)

    except Exception as e:
        print("Animacy loop error:", e)


def start_animacy():
    global anim_running, anim_thread
    if servo_yaw is None or servo_pitch is None:
        return
    if anim_running:
        return
    anim_running = True
    anim_thread = threading.Thread(target=animacy_loop, daemon=True)
    anim_thread.start()


def stop_animacy_and_center():
    """Stop childlike animacy and move head smoothly to centre (90, 90)."""
    global anim_running, yaw_deg, pitch_deg
    if servo_yaw is None or servo_pitch is None:
        return

    # stop background anim loop
    anim_running = False
    if anim_thread is not None:
        try:
            anim_thread.join(timeout=1.5)
        except Exception:
            pass

    target_yaw = START_YAW_DEG
    target_pitch = START_PITCH_DEG

    # move to centre using same compute_step()
    while True:
        with state_lock:
            cur_yaw = yaw_deg
            cur_pitch = pitch_deg
        dyaw = target_yaw - cur_yaw
        dpitch = target_pitch - cur_pitch
        dist = math.hypot(dyaw, dpitch)
        if dist < 0.5:
            with state_lock:
                yaw_deg = servo_yaw.set_angle(target_yaw)
                pitch_deg = servo_pitch.set_angle(target_pitch)
            break
        step_deg = compute_step(dist)
        step_yaw = clamp(dyaw, -step_deg, step_deg)
        step_pitch = clamp(dpitch, -step_deg, step_deg)
        with state_lock:
            yaw_deg = servo_yaw.set_angle(cur_yaw + step_yaw)
            pitch_deg = servo_pitch.set_angle(cur_pitch + step_pitch)
        time.sleep(STEP_DT)


# ================== AUDIO HELPER ==================

def play_intro_audio():
    """Play audio_intro.mp3 once, blocking until finished."""
    audio_path = os.path.join(BASE_DIR, "audio_intro.mp3")
    if not os.path.exists(audio_path):
        print(f"Audio file not found: {audio_path}")
        return
    try:
        pygame.mixer.init()
        pygame.mixer.music.load(audio_path)
        pygame.mixer.music.play()
        print("Playing audio_intro.mp3 ...")
        while pygame.mixer.music.get_busy():
            time.sleep(0.1)
    except Exception as e:
        print(f"Error playing audio: {e}")
    finally:
        try:
            pygame.mixer.music.stop()
        except Exception:
            pass
        try:
            pygame.mixer.quit()
        except Exception:
            pass
        print("Playback finished.")


# ================== EXISTING UI + SERIAL LOGIC ==================

def ensure_pywebview():
    try:
        import webview
        return True
    except Exception:
        try:
            subprocess.run([sys.executable, "-m", "pip", "install", "pywebview"], check=False)
            import webview
            return True
        except Exception:
            return False


def build_html():
    with open(ANIMATION_FILE, "r", encoding="utf-8") as f:
        data = json.load(f)
    anim_json = json.dumps(data)
    return (
        "<!DOCTYPE html><html><head><meta charset=\"utf-8\">"
        "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
        "<title>Opening...</title>"
        "<style>"
        "html,body{margin:0;height:100%;background:#000;overflow:hidden}"
        "#lottie{width:100vw;height:100vh}"
        "#subtitle{position:fixed;bottom:6vh;left:50%;transform:translateX(-50%);"
        "color:#fff;font-size:4vh;font-family:sans-serif;text-align:center;"
        "padding:10px 20px;border-radius:20px;background:rgba(0,0,0,0.6);"
        "display:none;max-width:90vw;}"
        "</style>"
        "</head><body>"
        "<div id=\"lottie\"></div>"
        "<div id=\"subtitle\"></div>"
        "<script src=\"https://cdnjs.cloudflare.com/ajax/libs/bodymovin/5.12.2/lottie.min.js\"></script>"
        f"<script>var animData={anim_json};"
        "lottie.loadAnimation({container:document.getElementById('lottie'),renderer:'svg',loop:true,autoplay:true,animationData:animData});"
        # typing subtitle JS: reveal words across 4 seconds
        "var subtitleText=\"Hello I'm Akshi! I'm Starting to analyze your learning behaviors now!\";"
        "function typeSubtitle(){"
        " var s=document.getElementById('subtitle');"
        " if(!s) return;"
        " s.style.display='block';"
        " var words=subtitleText.split(' ');"
        " s.innerText='';"
        " var idx=0;"
        " var total=words.length;"
        " var interval=4000/total;"   # 4 seconds total
        " var timer=setInterval(function(){"
        "   if(idx>=total){clearInterval(timer);return;}"
        "   if(idx===0){s.innerText=words[0];}"
        "   else{s.innerText+=' '+words[idx];}"
        "   idx++;"
        " }, interval);"
        "}"
        "</script>"
        "</body></html>"
    )


def start_serial(window):
    try:
        import serial
    except Exception:
        return
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    except Exception:
        ser = None

    def loop():
        main_started = False
        try:
            while True:
                if ser is None:
                    time.sleep(1)
                    continue
                raw = ser.readline().decode(errors='ignore').strip()
                if not raw:
                    continue
                if raw and raw[0].isdigit() and ',' in raw:
                    raw = raw.split(',', 1)[1].strip()
                parts = raw.split(',')
                vals = {}
                for p in parts:
                    p = p.strip()
                    if '=' in p:
                        k, v = p.split('=', 1)
                        k = k.strip()
                        v = v.strip()
                        try:
                            vals[k] = int(v)
                        except Exception:
                            vals[k] = None
                s1 = vals.get("S1")

                # Ultrasonic condition
                if s1 is not None and s1 <= 15 and not main_started:
                    main_started = True
                    try:
                        # stop animacy and centre head first
                        stop_animacy_and_center()

                        # start typing subtitle in the webview (word by word, 4 s)
                        try:
                            window.evaluate_js(
                                "if(typeof typeSubtitle==='function'){typeSubtitle();}"
                            )
                        except Exception as e:
                            print("Subtitle JS error:", e)

                        # play intro audio (blocking, in parallel user sees subtitle typing)
                        play_intro_audio()

                        # then launch main.py
                        subprocess.Popen(["python3", "main.py"], cwd=BASE_DIR)
                    finally:
                        try:
                            window.destroy()
                        except Exception:
                            pass
                        break
        finally:
            try:
                if ser is not None:
                    ser.close()
            except Exception:
                pass

    threading.Thread(target=loop, daemon=True).start()


def run():
    ok = ensure_pywebview()
    if not ok:
        return

    # Init servos and start childlike animacy
    init_servos()
    start_animacy()

    import webview
    html = build_html()
    window = webview.create_window("Opening...", html=html, fullscreen=True)
    start_serial(window)
    webview.start()

    # Cleanup after window closed
    global anim_running
    anim_running = False
    if anim_thread is not None:
        try:
            anim_thread.join(timeout=1.0)
        except Exception:
            pass
    try:
        if servo_yaw is not None:
            servo_yaw.stop()
        if servo_pitch is not None:
            servo_pitch.stop()
        if pi is not None:
            pi.stop()
    except Exception:
        pass


if __name__ == "__main__":
    run()
