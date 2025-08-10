# test/run_button_demo.py
import sys
import os
import time

# Add project root to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from src.button_handler import ButtonHandler

# Callback function when button is pressed
def on_button_press():
    print("✅ Callback: Button was pressed!")

# Create button handler instance
button = ButtonHandler(button_pin=18, callback=on_button_press, bouncetime_ms=300)

try:
    print("🟢 Waiting for button press... (CTRL+C to exit)")
    while True:
        time.sleep(0.1)  # Keeps CPU usage low
except KeyboardInterrupt:
    print("\n🛑 Exiting...")
finally:
    button.cleanup()
    print("🧹 GPIO cleaned up")
