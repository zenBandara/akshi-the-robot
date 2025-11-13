import serial

# Open the serial port (adjust port name if needed)
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

readings = []  # will hold lists of six distances

try:
    while len(readings) < 100:            # read up to 100 samples
        line = ser.readline().decode().strip()
        if not line:
            continue
        # Expect: "S1=123, S2=456, S3=789, S4=101, S5=112, S6=131"
        parts = line.split(',')
        vals = []
        for p in parts:
            if '=' in p:
                _, v = p.split('=', 1)
                try:
                    vals.append(int(v))
                except ValueError:
                    pass
        if len(vals) == 6:
            readings.append(vals)

finally:
    ser.close()

print(readings)
