#!/usr/bin/env python3

import time
import pigpio

# ===================== SETTINGS =====================
PIN_YAW = 24     # left-right
PIN_PITCH = 25   # up-down

SERVO_MIN_US = 600
SERVO_MAX_US = 2400

# ===================== HELPERS =====================

def clamp(v, lo, hi):
    return max(lo, min(hi, v))

def deg_to_us(angle):
    angle = clamp(angle, 0, 180)
    return int(SERVO_MIN_US + (angle / 180.0) * (SERVO_MAX_US - SERVO_MIN_US))

# ===================== INIT =====================

pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpio not connected")

pi.set_mode(PIN_YAW, pigpio.OUTPUT)
pi.set_mode(PIN_PITCH, pigpio.OUTPUT)

# ===================== TEST LOOP =====================

try:
    print("Starting servo test...")

    while True:
        # LEFT
        print("LEFT")
        pi.set_servo_pulsewidth(PIN_YAW, deg_to_us(30))
        time.sleep(2)

        # CENTER
        print("CENTER")
        pi.set_servo_pulsewidth(PIN_YAW, deg_to_us(90))
        time.sleep(2)

        # RIGHT
        print("RIGHT")
        pi.set_servo_pulsewidth(PIN_YAW, deg_to_us(150))
        time.sleep(2)

        # UP
        print("UP")
        pi.set_servo_pulsewidth(PIN_PITCH, deg_to_us(60))
        time.sleep(2)

        # CENTER
        print("CENTER")
        pi.set_servo_pulsewidth(PIN_PITCH, deg_to_us(90))
        time.sleep(2)

        # DOWN
        print("DOWN")
        pi.set_servo_pulsewidth(PIN_PITCH, deg_to_us(130))
        time.sleep(2)

except KeyboardInterrupt:
    print("Stopping...")

finally:
    pi.set_servo_pulsewidth(PIN_YAW, 0)
    pi.set_servo_pulsewidth(PIN_PITCH, 0)
    pi.stop()