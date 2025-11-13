import serial
import time
import subprocess

# --- Serial setup ---
ser = serial.Serial('/dev/serial0', baudrate=9600, timeout=1)

print("Listening for VC-02 commands...")

while True:
    if ser.in_waiting >= 2:  # expecting 2-byte frame
        start = ser.read()
        cmd = ser.read()

        start_val = f"{start[0]:02X}"
        cmd_val = f"{cmd[0]:02X}"

        print(f"Received HEX: {start_val} {cmd_val}")

        # Check for AA 03
        if start_val == "AA" and cmd_val == "03":
            print("? Starting main.py...")
            subprocess.Popen(["python3", "main.py"])

    time.sleep(0.05)
