#!/usr/bin/env python3
import time, serial, os

candidates = ["/dev/ttyAMA0", "/dev/ttyS0", "/dev/serial0"]
port = next((p for p in candidates if os.path.exists(p)), None)
if not port:
    raise SystemExit("No UART device found. Expected one of: " + ", ".join(candidates))

print(f"Opening {port} at 115200...")
ser = serial.Serial(port, 115200, timeout=1)
i = 0
try:
    while True:
        msg = f"UART OK {i}\n"
        ser.write(msg.encode())
        print("Sent:", msg.strip())
        i += 1
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
