#!/usr/bin/env python3
import serial
import time

PORT = "/dev/serial0"   # Symbolic link to Pi’s primary UART
BAUD = 115200

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Opened {PORT} at {BAUD} baud")
except Exception as e:
    print(f"Failed to open UART: {e}")
    exit(1)

i = 0
try:
    while True:
        msg = f"UART test message {i}\n"
        ser.write(msg.encode("utf-8"))
        print(f"Sent: {msg.strip()}")
        i += 1
        time.sleep(1)
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
