#!/usr/bin/env python3
"""
robot_motion.py ? Library for head (2-axis) + wheels control on Raspberry Pi,
with smooth-stop transitions between wheel motions.

GPIO map (BCM):
  Head up/down (pitch) servo  : 24
  Head left/right (yaw) servo : 23

  Wheels:
    22 = Left  wheel forward
    27 = Left  wheel reverse
    26 = Right wheel forward
    17 = Right wheel reverse
"""

from __future__ import annotations
import time
from dataclasses import dataclass
import RPi.GPIO as GPIO

# ============================== CONFIG ===============================

# --- GPIO mode / warnings
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# --- Pins
PIN_YAW   = 23   # left/right
PIN_PITCH = 24   # up/down
PIN_L_FWD = 22
PIN_L_REV = 27
PIN_R_FWD = 26
PIN_R_REV = 17

# --- Servo timing (50 Hz -> 20 ms period). Tweak if your servos differ.
SERVO_FREQ_HZ   = 50
SERVO_MIN_PW_MS = 0.5   # pulse width at 0�   (?2.5% duty)
SERVO_MAX_PW_MS = 2.5   # pulse width at 180� (?12.5% duty)
SERVO_MIN_DEG   = 0.0
SERVO_MAX_DEG   = 180.0

# --- Wheels PWM / ramp (smooth transitions)
WHEEL_FREQ_HZ    = 1000
MAX_SPEED        = 100.0      # logical max; we clamp to [0..100] for PWM duty
DEFAULT_RAMP_STEP= 4.0        # % duty change per step
DEFAULT_RAMP_DT  = 0.04       # seconds between steps
COAST_SEC        = 0.30       # pause with all pins low between different motions

# ============================ UTILITIES ==============================

def _clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def _duty_from_ms(pulse_ms: float, period_ms: float = 20.0) -> float:
    """Convert pulse width (ms) to duty% for RPi.GPIO PWM."""
    return (pulse_ms / period_ms) * 100.0

def _servo_duty_from_angle(angle_deg: float) -> float:
    """Map angle in [SERVO_MIN_DEG, SERVO_MAX_DEG] -> duty% based on min/max pulse widths."""
    a = _clamp(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG)
    span_ms = SERVO_MAX_PW_MS - SERVO_MIN_PW_MS
    pw_ms   = SERVO_MIN_PW_MS + (a - SERVO_MIN_DEG) * (span_ms / (SERVO_MAX_DEG - SERVO_MIN_DEG))
    return _duty_from_ms(pw_ms)

# ============================== HEAD =================================

class Head:
    """2-axis head control (yaw=left/right, pitch=up/down) using standard hobby servos."""

    def __init__(self, pin_yaw: int = PIN_YAW, pin_pitch: int = PIN_PITCH, freq_hz: int = SERVO_FREQ_HZ):
        self._pin_yaw   = pin_yaw
        self._pin_pitch = pin_pitch

        GPIO.setup([pin_yaw, pin_pitch], GPIO.OUT, initial=GPIO.LOW)
        self._pwm_yaw   = GPIO.PWM(pin_yaw,   freq_hz)
        self._pwm_pitch = GPIO.PWM(pin_pitch, freq_hz)
        self._pwm_yaw.start(0)
        self._pwm_pitch.start(0)
        print("Test complete.")

        # Track current angles (for smooth moves)
        self._yaw_angle   = 90.0
        self._pitch_angle = 90.0
        # Center on init
        self.set_yaw(90)
        self.set_pitch(90)

    def set_yaw(self, angle_deg: float) -> None:
        """Absolute left/right angle (0?180). 0 = far left, 180 = far right (conventional)."""
        self._yaw_angle = _clamp(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG)
        self._pwm_yaw.ChangeDutyCycle(_servo_duty_from_angle(self._yaw_angle))

    def set_pitch(self, angle_deg: float) -> None:
        """Absolute up/down angle (0?180). 0 = max up (depends on horn), 180 = max down."""
        self._pitch_angle = _clamp(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG)
        self._pwm_pitch.ChangeDutyCycle(_servo_duty_from_angle(self._pitch_angle))

    def move_yaw(self, delta_deg: float) -> None:
        """Relative move left/right by delta degrees (positive = right)."""
        self.set_yaw(self._yaw_angle + delta_deg)

    def move_pitch(self, delta_deg: float) -> None:
        """Relative move up/down by delta degrees (positive = down)."""
        self.set_pitch(self._pitch_angle + delta_deg)

    def center(self, yaw: float = 90.0, pitch: float = 90.0) -> None:
        self.set_yaw(yaw)
        self.set_pitch(pitch)

    def smooth_set(self, yaw: float | None = None, pitch: float | None = None,
                   secs: float = 0.5, steps: int = 50) -> None:
        """Ease both axes to target angles over time."""
        if yaw is None:   yaw   = self._yaw_angle
        if pitch is None: pitch = self._pitch_angle
        y0, p0 = self._yaw_angle, self._pitch_angle
        for i in range(1, steps + 1):
            u = i / steps
            self.set_yaw(y0 + (yaw - y0) * u)
            self.set_pitch(p0 + (pitch - p0) * u)
            time.sleep(secs / steps)

    def shutdown(self) -> None:
        self._pwm_yaw.stop()
        self._pwm_pitch.stop()

# ============================== WHEELS ===============================

@dataclass
class WheelPins:
    left_fwd:  int = PIN_L_FWD
    left_rev:  int = PIN_L_REV
    right_fwd: int = PIN_R_FWD
    right_rev: int = PIN_R_REV

class Wheels:
    """
    Differential drive with 4 PWM lines (FWD/REV for each side).
    Public API ensures SMOOTH transitions between commands:
      - If the new command changes which pins are active (e.g., forward -> reverse or pivot),
        we ramp down to 0, coast briefly, then ramp up into the new motion.
      - If it?s the same pins (e.g., speed tweak in same direction), we ramp directly.
    Speeds are specified per side in range [-100..100]. Sign = direction.
    """

    def __init__(self, pins: WheelPins = WheelPins(), freq_hz: int = WHEEL_FREQ_HZ):
        self.pins = pins
        GPIO.setup([pins.left_fwd, pins.left_rev, pins.right_fwd, pins.right_rev],
                   GPIO.OUT, initial=GPIO.LOW)
        self._pwm = {
            pins.left_fwd:  GPIO.PWM(pins.left_fwd,  freq_hz),
            pins.left_rev:  GPIO.PWM(pins.left_rev,  freq_hz),
            pins.right_fwd: GPIO.PWM(pins.right_fwd, freq_hz),
            pins.right_rev: GPIO.PWM(pins.right_rev, freq_hz),
        }
        for p in self._pwm.values():
            p.start(0)

        # Track current commanded speeds (logical, -100..100)
        self._cur_left  = 0.0
        self._cur_right = 0.0

    # -------- internals --------

    def _apply_side(self, speed: float, pin_fwd: int, pin_rev: int) -> None:
        """Set one side's direction and duty immediately (no ramp)."""
        s = _clamp(speed, -MAX_SPEED, MAX_SPEED)
        duty = abs(s)
        if s >= 0:
            self._pwm[pin_rev].ChangeDutyCycle(0)
            self._pwm[pin_fwd].ChangeDutyCycle(duty)
        else:
            self._pwm[pin_fwd].ChangeDutyCycle(0)
            self._pwm[pin_rev].ChangeDutyCycle(duty)

    def _set_immediate(self, left: float, right: float) -> None:
        self._apply_side(left,  self.pins.left_fwd,  self.pins.left_rev)
        self._apply_side(right, self.pins.right_fwd, self.pins.right_rev)
        self._cur_left, self._cur_right = left, right

    def _active_pins_for(self, left: float, right: float) -> tuple[int | None, int | None]:
        lp = self.pins.left_fwd  if left  > 0 else (self.pins.left_rev  if left  < 0 else None)
        rp = self.pins.right_fwd if right > 0 else (self.pins.right_rev if right < 0 else None)
        return (lp, rp)

    def _pins_change(self, left_t: float, right_t: float) -> bool:
        """Return True if moving to target would change which pins are active."""
        return self._active_pins_for(self._cur_left, self._cur_right) != self._active_pins_for(left_t, right_t)

    def _ramp_linear(self, left_t: float, right_t: float,
                     step: float = DEFAULT_RAMP_STEP, dt: float = DEFAULT_RAMP_DT) -> None:
        """Ramp from (self._cur_left/right) to (left_t/right_t) without stopping."""
        l0, r0 = self._cur_left, self._cur_right
        lt, rt = _clamp(left_t, -MAX_SPEED, MAX_SPEED), _clamp(right_t, -MAX_SPEED, MAX_SPEED)

        def sgn(x: float) -> float: return -1.0 if x < 0 else 1.0
        ls = step * sgn(lt - l0) if lt != l0 else 0.0
        rs = step * sgn(rt - r0) if rt != r0 else 0.0

        l, r = l0, r0
        while (ls != 0 and (l - lt) * sgn(ls) < 0) or (rs != 0 and (r - rt) * sgn(rs) < 0):
            if ls != 0:
                l_next = l + ls
                if (l_next - lt) * sgn(ls) > 0: l_next = lt
                l = l_next
            if rs != 0:
                r_next = r + rs
                if (r_next - rt) * sgn(rs) > 0: r_next = rt
                r = r_next
            self._set_immediate(l, r)
            time.sleep(dt)

    def _smooth_stop(self) -> None:
        """Ramp both sides to 0, coast, and ensure all PWMs are zero."""
        self._ramp_linear(0.0, 0.0)
        # Explicitly zero all channels and coast
        self._set_immediate(0.0, 0.0)
        for p in self._pwm.values():
            p.ChangeDutyCycle(0)
        time.sleep(COAST_SEC)

    # -------- public API (smooth by default) --------

    def go(self, left: float, right: float,
           step: float = DEFAULT_RAMP_STEP, dt: float = DEFAULT_RAMP_DT) -> None:
        """
        Command left/right speeds [-100..100] with automatic smooth-stop transitions.
        If the target changes active pins (direction change or pivot), we stop->coast->ramp.
        Otherwise we ramp directly to the new speed.
        """
        left  = _clamp(left,  -MAX_SPEED, MAX_SPEED)
        right = _clamp(right, -MAX_SPEED, MAX_SPEED)

        if self._pins_change(left, right):
            self._smooth_stop()
            # ramp up from 0 to target
            self._ramp_linear(left, right, step=step, dt=dt)
        else:
            # same pins: just ramp to new speed
            self._ramp_linear(left, right, step=step, dt=dt)

    def forward(self, speed: float = 40.0, duration: float | None = None) -> None:
        self.go(+abs(speed), +abs(speed))
        if duration:
            time.sleep(duration)
            self.stop()

    def reverse(self, speed: float = 40.0, duration: float | None = None) -> None:
        self.go(-abs(speed), -abs(speed))
        if duration:
            time.sleep(duration)
            self.stop()

    def pivot_left(self, speed: float = 40.0, duration: float | None = None) -> None:
        """Left wheel forward, right wheel reverse ? in-place left turn."""
        self.go(+abs(speed), -abs(speed))
        if duration:
            time.sleep(duration)
            self.stop()

    def pivot_right(self, speed: float = 40.0, duration: float | None = None) -> None:
        self.go(-abs(speed), +abs(speed))
        if duration:
            time.sleep(duration)
            self.stop()

    def stop(self) -> None:
        """Smooth stop with coast."""
        self._smooth_stop()

    # -------- low-level (immediate) if you really need it --------
    def set_speeds_immediate(self, left: float, right: float) -> None:
        """Immediate set without ramp/stop (expert use only)."""
        self._set_immediate(left, right)

    def shutdown(self) -> None:
        self.stop()
        for p in self._pwm.values():
            p.stop()

# ============================ TOP-LEVEL ==============================

class Robot:
    """Facade that bundles head + wheels. Use as a context manager for safe cleanup."""

    def __init__(self):
        # Clean any previous state
        GPIO.cleanup()
        GPIO.setmode(GPIO.BCM)
        self.head   = Head(PIN_YAW, PIN_PITCH)
        self.wheels = Wheels()

    def shutdown(self) -> None:
        try:
            self.head.shutdown()
        finally:
            try:
                self.wheels.shutdown()
            finally:
                GPIO.cleanup()

    # Context-manager sugar: with Robot() as bot: ...
    def __enter__(self) -> "Robot":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.shutdown()
