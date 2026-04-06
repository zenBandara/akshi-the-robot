import serial
import time

class EyeController:
    def __init__(self):
        self.port = '/dev/ttyACM0'
        self.baudrate = 115200
        self.timeout = 1
        self.ser = None

        self._connect()

    def _connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            time.sleep(2)  # wait for ESP32 reset
            print("Connected to ESP32")
        except Exception as e:
            print(f"Connection failed: {e}")

    def send(self, cmd):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd + '\n').encode())
            print(f"Sent: {cmd}")
        else:
            print("Serial not connected")

    # Emotion methods
    def happy(self):
        self.send('H')

    def sad(self):
        self.send('S')

    def cheer(self):
        self.send('C')

    def lovely(self):
        self.send('L')

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial closed")