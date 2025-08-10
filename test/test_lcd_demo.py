# test/test_lcd_demo.py

import sys
import os
import time

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.lcd_display import LCDDisplay

lcd = LCDDisplay()

try:
    print("🟢 Displaying messages on LCD...")
    lcd.clear() #Clear before first message
    lcd.show_message("Phase 5", "LCD Passed")
    time.sleep(4)

    lcd.clear() #Clear before second message
    time.sleep(1)
    lcd.show_message("Hello,", "Gonzalo Patino")
    time.sleep(4)

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    lcd.cleanup()
    print("🧹 Cleaned up GPIO and LCD")
