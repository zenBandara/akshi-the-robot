#!/usr/bin/env python3
import time
import signal
import random
import math
import pigpio

# ================= USER SETTINGS =================

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

# =================================================


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


running = True


def handle_sigint(sig, frame):
    global running
    running = False


signal.signal(signal.SIGINT, handle_sigint)


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


def main():
    global running

    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running. Use: sudo systemctl start pigpiod")

    servo_yaw = Servo(pi, PIN_YAW, START_YAW_DEG)
    servo_pitch = Servo(pi, PIN_PITCH, START_PITCH_DEG)

    yaw_deg = START_YAW_DEG
    pitch_deg = START_PITCH_DEG

    target_yaw, target_pitch = pick_pose(yaw_deg, pitch_deg)
    last_micro_time = time.time()

    try:
        while running:
            dyaw = target_yaw - yaw_deg
            dpitch = target_pitch - pitch_deg
            dist = math.hypot(dyaw, dpitch)

            # If close to target, short pause with fidgeting
            if dist < 0.5:
                pause_time = random.uniform(PAUSE_MIN, PAUSE_MAX)
                end_pause = time.time() + pause_time

                while running and time.time() < end_pause:
                    now = time.time()
                    if now - last_micro_time > MICRO_JITTER_PERIOD:
                        j_y = random.uniform(-MICRO_JITTER_DEG, MICRO_JITTER_DEG)
                        j_p = random.uniform(-MICRO_JITTER_DEG, MICRO_JITTER_DEG)

                        yaw_deg = servo_yaw.set_angle(yaw_deg + j_y)
                        pitch_deg = servo_pitch.set_angle(pitch_deg + j_p)

                        last_micro_time = now

                    time.sleep(0.05)

                target_yaw, target_pitch = pick_pose(yaw_deg, pitch_deg)
                continue

            # Step size based on distance
            step_deg = compute_step(dist)

            # Move one step toward target
            step_yaw = clamp(dyaw, -step_deg, step_deg)
            step_pitch = clamp(dpitch, -step_deg, step_deg)

            yaw_deg = servo_yaw.set_angle(yaw_deg + step_yaw)
            pitch_deg = servo_pitch.set_angle(pitch_deg + step_pitch)

            time.sleep(STEP_DT)

    finally:
        try:
            servo_yaw.stop()
            servo_pitch.stop()
        except:
            pass
        try:
            pi.stop()
        except:
            pass
        print("Childlike head animacy stopped.")


if __name__ == "__main__":
    main()
