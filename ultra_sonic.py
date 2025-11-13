import serial

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

try:
    while True:
        raw = ser.readline().decode(errors='ignore').strip()
        if not raw:
            continue

        # Example raw values:
        # "0, S6=500"
        # "S1=119, S2=9, S3=3, S4=43, S5=500, S6=500"

        # Remove leading numbers like "0,"
        if raw[0].isdigit() and ',' in raw:
            raw = raw.split(',', 1)[1].strip()

        parts = raw.split(',')
        vals = {}

        for p in parts:
            p = p.strip()
            if '=' in p:
                key, value = p.split('=', 1)
                key = key.strip()
                value = value.strip()
                try:
                    vals[key] = int(value)
                except:
                    vals[key] = None

        # Only print when all six sensors exist
        required = ["S1","S2","S3","S4","S5","S6"]
        if all(k in vals for k in required):
            print(
                "S1:", vals["S1"],
                "S2:", vals["S2"],
                "S3:", vals["S3"],
                "S4:", vals["S4"],
                "S5:", vals["S5"],
                "S6:", vals["S6"],
            )

except KeyboardInterrupt:
    print("Stopped")

finally:
    ser.close()
