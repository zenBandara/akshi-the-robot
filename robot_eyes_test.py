# import serial
# import time

# # Open serial connection
# ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
# time.sleep(2)   # wait for ESP32 reset

# def send(cmd):
#     ser.write((cmd + '\n').encode())
#     print(f"Sent: {cmd}")

# print("\nH: Happy")
# print("S: Sad")
# print("C: Cheer")
# print("L: Lovely")
# print("Q: Quit")

# send("S")


from EyeController import EyeController
import time

eyes = EyeController()

eyes.happy()
# time.sleep(2)

# eyes.sad()
# time.sleep(2)

# eyes.cheer()
# time.sleep(2)

# eyes.lovely()

eyes.close()