import time
import board
import digitalio
import adafruit_character_lcd.character_lcd as characterlcd

class LCDDisplay:
    def __init__(self):
        self.lcd_columns = 16
        self.lcd_rows = 2

        self.lcd_rs = digitalio.DigitalInOut(board.D17)
        self.lcd_en = digitalio.DigitalInOut(board.D27)
        self.lcd_d4 = digitalio.DigitalInOut(board.D5)
        self.lcd_d5 = digitalio.DigitalInOut(board.D6)
        self.lcd_d6 = digitalio.DigitalInOut(board.D13)
        self.lcd_d7 = digitalio.DigitalInOut(board.D26)

        self.lcd = characterlcd.Character_LCD_Mono(
            self.lcd_rs, self.lcd_en,
            self.lcd_d4, self.lcd_d5, self.lcd_d6, self.lcd_d7,
            self.lcd_columns, self.lcd_rows
        )
        self.clear()

    def show_message(self, line1: str, line2: str = ""):
        # Pad lines to exactly 16 characters to clear previous content
        line1 = line1[:16].ljust(16)
        line2 = line2[:16].ljust(16)
        message = f"{line1}\n{line2}"
        self.lcd.message = message


    def clear(self):
        self.lcd.clear()

    def cleanup(self):
        try:
            self.show_message("Shutting down", "")
            time.sleep(1)
            self.clear()
        except Exception as e:
            print(f"⚠️ LCD cleanup warning: {e}")
        finally:
            for pin in [self.lcd_rs, self.lcd_en, self.lcd_d4, self.lcd_d5, self.lcd_d6, self.lcd_d7]:
                try:
                    pin.deinit()
                except Exception:
                    pass
