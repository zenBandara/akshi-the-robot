# distance_follower.py
import time
import pigpio

class DistanceFollowerController:
    """
    Keep a person at a target size by moving the robot forward/backward with the wheels.
    - If the face box gets smaller than a threshold (person moving away), go forward.
    - (Optional) If the face box gets too big (person too close), go backward.
    - Before driving straight, pivot the chassis to face the head's yaw direction.

    Wheel pins (BCM):
      Left  wheel: forward=22, reverse=27
      Right wheel: forward=26, reverse=17
    """

    def __init__(
        self,
        pi: pigpio.pi,
        left_fwd=22, left_rev=27,
        right_fwd=26, right_rev=17,
        pwm_freq_hz=400,
        # ---- distance (size) targets ----
        target_w_px=220,     # desired face-box width in pixels
        band_px=30,          # +/- tolerance (deadband) around target_w_px
        allow_reverse=False, # if True, back up when face too big
        # ---- heading alignment ----
        yaw_center_deg=90.0,     # head's "center" angle
        align_tol_deg=8.0,       # pivot until |yaw - center| <= this
        turn_sign=+1,            # flip if pivot direction feels inverted
        # ---- speed/ramping ----
        min_pwm=70,              # minimum wheel PWM when moving (0..255)
        max_pwm=200,             # maximum wheel PWM (0..255)
        k_speed=0.9,             # scale from size error -> PWM
        ramp_step=12,            # PWM step per update
        face_lost_timeout_s=1.0, # stop if face disappears for this long
        invert_left=True,        # set True if your left wheel wiring is reversed
        invert_right=True        # set True if your right wheel wiring is reversed
    ):
        self.pi = pi
        self.pins = {"lf": left_fwd, "lr": left_rev, "rf": right_fwd, "rr": right_rev}
        for p in self.pins.values():
            self.pi.set_mode(p, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(p, pwm_freq_hz)
            self.pi.set_PWM_range(p, 255)
            self.pi.set_PWM_dutycycle(p, 0)

        # config
        self.target_w_px = float(target_w_px)
        self.band_px     = float(band_px)
        self.allow_reverse = bool(allow_reverse)
        self.yaw_center  = float(yaw_center_deg)
        self.align_tol   = float(align_tol_deg)
        self.turn_sign   = -1 if turn_sign < 0 else 1
        self.min_pwm     = int(min_pwm)
        self.max_pwm     = int(max_pwm)
        self.k_speed     = float(k_speed)
        self.ramp_step   = int(ramp_step)
        self.face_lost_timeout_s = float(face_lost_timeout_s)
        self.invert_left  = bool(invert_left)
        self.invert_right = bool(invert_right)

        # state
        self.mode = "idle"          # "idle" | "align" | "drive"
        self._last_face_ts = 0.0
        self._cur_l = 0
        self._cur_r = 0
        self._tgt_l = 0
        self._tgt_r = 0

    # -------- public API --------
    def set_target_width(self, target_w_px: float, band_px: float = None):
        """Change the desired face-box width and (optionally) the deadband."""
        self.target_w_px = float(target_w_px)
        if band_px is not None:
            self.band_px = float(band_px)

    def stop(self):
        """Immediately command a stop (with ramp to 0 on next apply)."""
        self._tgt_l = 0
        self._tgt_r = 0
        self._ramp_to_targets()
        self._apply_pwm()
        self.mode = "idle"

    def update(
        self,
        *,
        yaw_deg: float,         # current head yaw angle (absolute, e.g., 0..180)
        bbox_w_px: float,       # current detected face-box width in pixels (or None)
        frame_w_px: float,      # (unused now, kept for future scaling needs)
        face_present: bool,
        dt: float
    ):
        """
        Call every loop.
        Returns a small dict for HUD/logging: {"mode": ..., "cur_l": ..., "cur_r": ...}
        """
        now = time.time()
        if face_present:
            self._last_face_ts = now

        # Face lost failsafe
        if not face_present or (now - self._last_face_ts) > self.face_lost_timeout_s:
            self._set_targets(0, 0)
            self._ramp_to_targets()
            self._apply_pwm()
            self.mode = "idle"
            return {"mode": self.mode, "cur_l": self._cur_l, "cur_r": self._cur_r}

        # 1) Heading alignment phase: pivot until head yaw is near center
        yaw_offset = yaw_deg - self.yaw_center
        if abs(yaw_offset) > self.align_tol:
            self.mode = "align"
            self._command_pivot_from_yaw(yaw_offset)
        else:
            # 2) Distance control phase: keep face width near target
            self.mode = "drive"
            if bbox_w_px is None:
                # No size signal -> just stop
                self._set_targets(0, 0)
            else:
                err = self.target_w_px - float(bbox_w_px)
                if err > self.band_px:
                    # face too small -> go forward
                    spd = self._pwm_from_error(err - self.band_px)
                    self._set_targets(+spd, +spd)
                elif err < -self.band_px and self.allow_reverse:
                    # face too big -> go backward (optional)
                    spd = self._pwm_from_error((-err) - self.band_px)
                    self._set_targets(-spd, -spd)
                else:
                    # within band -> stop
                    self._set_targets(0, 0)

        # Smooth & apply
        self._ramp_to_targets()
        self._apply_pwm()
        return {"mode": self.mode, "cur_l": self._cur_l, "cur_r": self._cur_r}

    # -------- internals --------
    def _pwm_from_error(self, e_pos: float) -> int:
        """Map a positive error -> PWM, with min/max limits."""
        pwm = int(self.min_pwm + self.k_speed * e_pos)
        return max(self.min_pwm, min(self.max_pwm, pwm))

    def _command_pivot_from_yaw(self, yaw_offset_deg: float):
        """
        Positive yaw_offset => head looks to the right.
        turn_sign selects which way the chassis should pivot to face that direction.
        """
        s = self.turn_sign * (1 if yaw_offset_deg > 0 else -1)
        if s > 0:
            # pivot right: left wheel forward, right wheel reverse
            self._set_targets(+self.max_pwm, -self.max_pwm)
        else:
            # pivot left: left wheel reverse, right wheel forward
            self._set_targets(-self.max_pwm, +self.max_pwm)

    def _set_targets(self, l_signed_pwm: int, r_signed_pwm: int):
        self._tgt_l = int(max(-self.max_pwm, min(self.max_pwm, l_signed_pwm)))
        self._tgt_r = int(max(-self.max_pwm, min(self.max_pwm, r_signed_pwm)))

    def _ramp_to_targets(self):
        self._cur_l = self._ramp(self._cur_l, self._tgt_l, self.ramp_step)
        self._cur_r = self._ramp(self._cur_r, self._tgt_r, self.ramp_step)

    @staticmethod
    def _ramp(cur, tgt, step):
        if cur < tgt:
            cur = min(cur + step, tgt)
        elif cur > tgt:
            cur = max(cur - step, tgt)
        return cur

    def _apply_pwm(self):
        # Left wheel
        self._drive_wheel(self.pins["lf"], self.pins["lr"], self._cur_l, is_left=True)
        # Right wheel
        self._drive_wheel(self.pins["rf"], self.pins["rr"], self._cur_r, is_left=False)

    def _drive_wheel(self, pin_fwd, pin_rev, signed_pwm, *, is_left: bool):
        inv = self.invert_left if is_left else self.invert_right
        fwd_pin, rev_pin = (pin_rev, pin_fwd) if inv else (pin_fwd, pin_rev)

        pwm = int(abs(signed_pwm))
        if signed_pwm > 0:
            # forward
            self.pi.set_PWM_dutycycle(rev_pin, 0)
            self.pi.set_PWM_dutycycle(fwd_pin, pwm)
        elif signed_pwm < 0:
            # reverse
            self.pi.set_PWM_dutycycle(fwd_pin, 0)
            self.pi.set_PWM_dutycycle(rev_pin, pwm)
        else:
            # stop
            self.pi.set_PWM_dutycycle(fwd_pin, 0)
            self.pi.set_PWM_dutycycle(rev_pin, 0)
