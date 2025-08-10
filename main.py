import sys, os, time

# Ensure we can import from ./src
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), 'src')))

from button_handler import ButtonHandler  # no "src." now that we adjusted sys.path
# from lcd_display import LCDDisplay  # uncomment if you actually use it

def on_button_press():
    print("Hello world")

button = ButtonHandler(button_pin=18, callback=on_button_press, bouncetime_ms=500)
# If your ButtonHandler supports it, this helps prevent repeats:
# button = ButtonHandler(button_pin=18, callback=on_button_press, bouncetime_ms=500, require_release=True)

try:
    print("Waiting for button press")
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\nExiting...")
finally:
    button.cleanup()
    print("Cleaned up GPIO")
