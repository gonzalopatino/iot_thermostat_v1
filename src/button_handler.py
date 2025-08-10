# src/button_handler.py
import RPi.GPIO as GPIO

class ButtonHandler:
    def __init__(self, button_pin: int, callback=None, bouncetime_ms: int = 500):
        self.button_pin = button_pin
        self.callback = callback

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        try:
            GPIO.remove_event_detect(self.button_pin)  # clear stale registrations
        except Exception:
            pass

        GPIO.add_event_detect(
            self.button_pin,
            GPIO.FALLING,
            callback=self._internal_callback,
            bouncetime=bouncetime_ms
        )

    def _internal_callback(self, channel):
        print("🔘 Button pressed on GPIO", self.button_pin)
        if callable(self.callback):
            self.callback()

    def cleanup(self):
        try:
            GPIO.remove_event_detect(self.button_pin)
        except Exception:
            pass
        GPIO.cleanup(self.button_pin)  # only this pin
