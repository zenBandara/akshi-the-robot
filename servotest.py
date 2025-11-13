
#!/usr/bin/env python3

from __future__ import annotations
import time
from dataclasses import dataclass
import RPi.GPIO as GPIO

# Set GPIO mode
GPIO.setmode(GPIO.BCM)

# Define the servo pin
servo_pin = 24
GPIO.setup(servo_pin, GPIO.OUT)

# Create PWM instance at 50Hz (standard for servos)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

def set_angle(angle):
    # Convert angle (0?180) to duty cycle (2?12)
    duty = 2 + (angle / 18)
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        angle = float(input("Enter angle (0 to 180): "))
        if 0 <= angle <= 180:
            set_angle(angle)
        else:
            print("Angle out of range!")

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
