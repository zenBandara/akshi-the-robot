# wheel_handoff.py
import time
import pigpio

class WheelHandoffController:
    """
    Hand off from head yaw (GPIO 23) to chassis (wheels) when yaw nears limit.
    While the chassis pivots, request the head yaw to re-center and hold.

    Pins (BCM), fixed mapping:
      Left wheel:  forward=27, reverse=17
      Right wheel: forward=26, reverse=22
    """

    def __init__(
        self,
        pi: pigpio.pi,
        left_fwd=27, left_rev=17,
        right_fwd=26, right_rev=22,
        pwm_freq_hz=1000,
        max_pwm=255,            # 0..255
        ramp_step=15,           # PWM step per update
        yaw_soft_limit_deg=70,  # trigger handoff near this yaw
        yaw_center_deg=90,
        head_center_slew=0.98,  # 0..1 smoothing when recentring head
        face_lost_timeout_s=1.0,
        turn_sign=1,           # ?? FLIP DEFAULT so positive yaw => pivot LEFT->RIGHT matches your rig
        invert_left=True,      # set True if your left wheel wiring is reversed
        invert_right=True      # set True if your right wheel wiring is reversed
    ):
        self.pi = pi
        self.pins = {"lf": left_fwd, "lr": left_rev, "rf": right_fwd, "rr": right_rev}
        for p in self.pins.values():
            self.pi.set_mode(p, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(p, pwm_freq_hz)
            self.pi.set_PWM_range(p, 255)
            self.pi.set_PWM_dutycycle(p, 0)

        self.mode = "head"
        self._yaw_soft = float(yaw_soft_limit_deg)
        self._yaw_center = float(yaw_center_deg)
        self._head_center_slew = float(head_center_slew)
        self._max_pwm = int(max_pwm)
        self._ramp_step = int(ramp_step)
        self._cur_l = 0
        self._cur_r = 0
        self._tgt_l = 0
        self._tgt_r = 0
        self._yaw_hold_deg = self._yaw_center
        self._face_lost_timeout_s = float(face_lost_timeout_s)
        self._last_face_ts = 0.0

        self._turn_sign = -1 if turn_sign < 0 else 1
        self._invert_left = bool(invert_left)
        self._invert_right = bool(invert_right)

    def update(
        self,
        yaw_deg: float,
        yaw_err_deg: float,
        face_r_pix: float,
        lock_radius_pix: float,
        reacquire_radius_pix: float,
        face_present: bool,
        dt: float
    ):
        now = time.time()
        if face_present:
            self._last_face_ts = now

        if (not face_present) and self.mode == "chassis":
            if (now - self._last_face_ts) > self._face_lost_timeout_s:
                self._stop_wheels()
                self.mode = "head"

        if self.mode == "head":
            if face_present and abs(yaw_deg - self._yaw_center) >= self._yaw_soft:
                yaw_offset = yaw_deg - self._yaw_center
                self._begin_chassis_turn_from_yaw(yaw_offset)
        else:
            if face_present and (face_r_pix <= lock_radius_pix):
                self._stop_wheels()
                self.mode = "head"

        if self.mode == "chassis":
            self._ramp_to_targets()
            self._apply_pwm()
            self._yaw_hold_deg = (
                self._head_center_slew * self._yaw_hold_deg
                + (1.0 - self._head_center_slew) * self._yaw_center
            )
            yaw_hold = self._yaw_hold_deg
        else:
            self._stop_wheels()
            yaw_hold = None

        return {"mode": self.mode, "yaw_hold_deg": yaw_hold}

    def stop(self):
        self._stop_wheels()

    # ----------------- internals -----------------
    def _begin_chassis_turn_from_yaw(self, yaw_offset_deg: float):
        # positive yaw_offset => head to the right; choose pivot direction via turn_sign
        s = self._turn_sign * (1 if yaw_offset_deg > 0 else -1)

        if s > 0:
            #interchanged
            # pivot left: left wheel reverse, right wheel forward
            self._tgt_l = -self._max_pwm
            self._tgt_r = +self._max_pwm
        else:
            # pivot right: left wheel forward, right wheel reverse
            self._tgt_l = +self._max_pwm
            self._tgt_r = -self._max_pwm

        self.mode = "chassis"
        self._yaw_hold_deg = self._yaw_center

    def _ramp_to_targets(self):
        self._cur_l = self._ramp(self._cur_l, self._tgt_l, self._ramp_step)
        self._cur_r = self._ramp(self._cur_r, self._tgt_r, self._ramp_step)

    @staticmethod
    def _ramp(cur, tgt, step):
        if cur < tgt:
            cur = min(cur + step, tgt)
        elif cur > tgt:
            cur = max(cur - step, tgt)
        return cur

    def _apply_pwm(self):
        self._drive_wheel(self.pins["lf"], self.pins["lr"], self._cur_l, is_left=True)
        self._drive_wheel(self.pins["rf"], self.pins["rr"], self._cur_r, is_left=False)

    def _drive_wheel(self, pin_fwd, pin_rev, signed_pwm, is_left: bool):
        inv = self._invert_left if is_left else self._invert_right
        fwd_pin, rev_pin = (pin_rev, pin_fwd) if inv else (pin_fwd, pin_rev)

        pwm = int(abs(signed_pwm))
        if signed_pwm > 0:
            self.pi.set_PWM_dutycycle(rev_pin, 0)
            self.pi.set_PWM_dutycycle(fwd_pin, pwm)
        elif signed_pwm < 0:
            self.pi.set_PWM_dutycycle(fwd_pin, 0)
            self.pi.set_PWM_dutycycle(rev_pin, pwm)
        else:
            self.pi.set_PWM_dutycycle(fwd_pin, 0)
            self.pi.set_PWM_dutycycle(rev_pin, 0)

    def _stop_wheels(self):
        self._tgt_l = self._tgt_r = 0
        self._ramp_to_targets()
        self._apply_pwm()
