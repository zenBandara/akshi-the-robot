# test_wheel_gpio.py
# Simple GPIO mapping check for your robot wheels (pigpio required)
# Left wheel : forward=22, reverse=27
# Right wheel: forward=26, reverse=17

import time
import pigpio

# --- Pins ---
L_FWD, L_REV = 22, 27
R_FWD, R_REV = 26, 17

DUTY = 220      # 0..255 (set lower if it's too fast)
FREQ = 400      # PWM frequency Hz
HOLD = 4.0      # seconds per test

def drive(pi, l_fwd=0, l_rev=0, r_fwd=0, r_rev=0, label=""):
    # Ensure we never drive both directions on the same wheel
    if (l_fwd and l_rev) or (r_fwd and r_rev):
        raise RuntimeError("Both directions requested on the same wheel!")

    # Set all to 0 first
    for p in (L_FWD, L_REV, R_FWD, R_REV):
        pi.set_PWM_dutycycle(p, 0)

    # Apply requested sides
    if l_fwd: pi.set_PWM_dutycycle(L_FWD, DUTY)
    if l_rev: pi.set_PWM_dutycycle(L_REV, DUTY)
    if r_fwd: pi.set_PWM_dutycycle(R_FWD, DUTY)
    if r_rev: pi.set_PWM_dutycycle(R_REV, DUTY)

    print(f"Executing: {label}")
    time.sleep(HOLD)

def main():
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpio daemon not running. Start it: sudo systemctl start pigpiod")

    try:
        # Setup pins
        for p in (L_FWD, L_REV, R_FWD, R_REV):
            pi.set_mode(p, pigpio.OUTPUT)
            pi.set_PWM_frequency(p, FREQ)
            pi.set_PWM_range(p, 255)
            pi.set_PWM_dutycycle(p, 0)

        # ---- Tests (4s each) ----
        drive(pi, l_fwd=1, label=f"Left wheel : FORWARD  (GPIO {L_FWD})")
        # drive(pi, l_rev=1, label=f"Left wheel : REVERSE  (GPIO {L_REV})")
        # drive(pi, r_fwd=1, label=f"Right wheel: FORWARD  (GPIO {R_FWD})")
        # drive(pi, r_rev=1, label=f"Right wheel: REVERSE  (GPIO {R_REV})")

        print("Test complete.")

    finally:
        # Stop everything
        for p in (L_FWD, L_REV, R_FWD, R_REV):
            pi.set_PWM_dutycycle(p, 0)
        pi.stop()

if __name__ == "__main__":
    main()
