# handoff_controller.py
#!/usr/bin/env python3
"""
BaseTurnHandoff
---------------
Hand off from head yaw (GPIO 23) to chassis rotation (wheels) when yaw hits a soft limit.
While the base pivots, the head yaw recenters and holds; when the face is back inside
a fair radius around the image center, wheels stop and head-only tracking resumes.

Depends on:
  - pigpio (for wheel PWM)
  - Your existing Servo class instance for yaw (GPIO 23)

Wheel GPIO map (BCM):
  LF=22 (forward-left), LR=27 (reverse-left),
  RF=26 (forward-right), RR=17 (reverse-right)
"""

import time
import math
import pigpio

class BaseTurnHandoff:
    MODE_HEAD  = "HEAD_TRACK"
    MODE_BASE  = "BASE_TURN"

    def __init__(
        self,
        pi: pigpio.pi,
        pins: dict,
        yaw_servo,                        # your existing Servo instance for GPIO 23
        *,
        yaw_center_deg: float = 90.0,
        yaw_soft_limit_deg: float = 70.0, # trigger threshold (?�70� from center)
        fair_radius_px: int = 90,         # stop wheels if face within this radius
        reacquire_radius_px: int = 130,   # start wheels if face outside this radius
        pwm_freq_hz: int = 1000,
        turn_speed_dc: int = 150,         # 0..255 duty for pivot speed
        ramp_step: int = 20,              # duty increment per ramp step
        ramp_dt: float = 0.02,            # seconds between ramp steps
        max_turn_s: float = 2.5,          # safety timeout while pivoting
    ):
        self.pi = pi
        # Pins dict must contain: LF, LR, RF, RR
        self.p = pins
        self.yaw_servo = yaw_servo
        self.yaw_center_deg = yaw_center_deg
        self.yaw_soft_limit = float(yaw_soft_limit_deg)
        self.fair_r = int(fair_radius_px)
        self.reacq_r = int(reacquire_radius_px)
        self.freq = int(pwm_freq_hz)
        self.turn_dc = int(max(0, min(255, turn_speed_dc)))
        self.ramp_step = int(max(1, ramp_step))
        self.ramp_dt = float(max(0.0, ramp_dt))
        self.max_turn_s = float(max(0.0, max_turn_s))

        # Internal state
        self.mode = self.MODE_HEAD
        self.lock_yaw = False
        self._dc = {k: 0 for k in ("LF","LR","RF","RR")}
        self._turn_dir = 0   # -1 = left, +1 = right
        self._t_start = 0.0

        # Init pins
        for k,g in self.p.items():
            self.pi.set_mode(g, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(g, self.freq)
            self.pi.set_PWM_dutycycle(g, 0)

    # ------------- Public API ----------------
    def update(self, yaw_deg: float, yaw_err: float,
               face_center, frame_size, face_visible: bool) -> dict:
        """
        Call once per loop.

        Args:
          yaw_deg: current head yaw angle (deg)
          yaw_err: signed yaw error (deg) from your vision controller (right positive)
          face_center: (cx, cy) in pixels, or None if no face
          frame_size: (W, H)
          face_visible: bool

        Returns dict:
          {
            "mode": "HEAD_TRACK" | "BASE_TURN",
            "lock_yaw": bool,         # True => hold yaw at center & skip yaw updates
            "just_switched": bool
          }
        """
        just_switched = False
        W, H = frame_size

        if self.mode == self.MODE_HEAD:
            # Trigger handoff if yaw beyond soft limit and we still need more in same direction
            if abs(yaw_deg - self.yaw_center_deg) >= self.yaw_soft_limit and face_visible:
                # Decide turn direction from sign of yaw_err
                self._turn_dir = 1 if yaw_err > 0 else -1
                self._enter_base_turn()
                just_switched = True

        if self.mode == self.MODE_BASE:
            # Keep pivoting until face returns within fair radius or timeout
            now = time.monotonic()
            if (not face_visible) and (now - self._t_start > self.max_turn_s):
                # Lost target too long ? stop for safety
                self._exit_base_turn()
                just_switched = True
            else:
                if face_center is not None:
                    cx, cy = face_center
                    r = math.hypot(cx - W/2.0, cy - H/2.0)
                    if r <= self.fair_r:
                        self._exit_base_turn()
                        just_switched = True
                # else: keep pivoting a bit to try to reacquire (covered by timeout)

        return {"mode": self.mode, "lock_yaw": self.lock_yaw, "just_switched": just_switched}

    def stop(self):
        """Hard stop wheels and release yaw lock (does not move the yaw servo)."""
        self._ramp_all_to(0)
        self.mode = self.MODE_HEAD
        self.lock_yaw = False

    # ------------- Private helpers ----------------
    def _enter_base_turn(self):
        # Center and lock yaw for stability
        self.yaw_servo.set_angle(self.yaw_center_deg)
        self.lock_yaw = True
        # Start pivot with a smooth ramp
        if self._turn_dir < 0:
            self._pivot_left(self.turn_dc)
        else:
            self._pivot_right(self.turn_dc)
        self._t_start = time.monotonic()
        self.mode = self.MODE_BASE

    def _exit_base_turn(self):
        self._ramp_all_to(0)
        self.mode = self.MODE_HEAD
        self.lock_yaw = False
        # Head remains centered; your head controller will resume from center.

    # --- Wheel patterns ---
    def _pivot_right(self, target_dc: int):
        # Left wheel reverse (LR), Right wheel forward (RF)
        self._safe_zero("LR"); self._safe_zero("RF")
        self._ramp_to("LF", target_dc)
        self._ramp_to("RR", target_dc)
        self._safe_zero("LR"); self._safe_zero("RF")

    def _pivot_left(self, target_dc: int):
        # Left wheel forward (LF), Right wheel reverse (RR)
        self._safe_zero("LF"); self._safe_zero("RR")
        self._ramp_to("LR", target_dc)
        self._ramp_to("RF", target_dc)
        self._safe_zero("LF"); self._safe_zero("RR")

    def _ramp_all_to(self, target_dc: int):
        # Ramp all channels together to target_dc
        target_dc = int(max(0, min(255, target_dc)))
        more = True
        while more:
            more = False
            for k in self._dc.keys():
                more |= self._step_channel_towards(k, target_dc)
            if more:
                time.sleep(self.ramp_dt)

    def _ramp_to(self, key: str, target_dc: int):
        target_dc = int(max(0, min(255, target_dc)))
        while self._step_channel_towards(key, target_dc):
            time.sleep(self.ramp_dt)

    def _step_channel_towards(self, key: str, target_dc: int) -> bool:
        cur = self._dc[key]
        if cur == target_dc:
            return False
        step = self.ramp_step if target_dc > cur else -self.ramp_step
        nxt = cur + step
        if (step > 0 and nxt > target_dc) or (step < 0 and nxt < target_dc):
            nxt = target_dc
        self._set_dc(key, nxt)
        return nxt != target_dc

    def _set_dc(self, key: str, dc: int):
        dc = int(max(0, min(255, dc)))
        pin = self.p[key]
        self.pi.set_PWM_dutycycle(pin, dc)
        self._dc[key] = dc

    def _safe_zero(self, key: str):
        # Ensure opposite directions aren't driven together
        if self._dc[key] != 0:
            self._set_dc(key, 0)
