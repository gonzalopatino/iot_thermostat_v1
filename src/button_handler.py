# src/button_handler.py
import time
import RPi.GPIO as GPIO

class ButtonHandler:
    def __init__(self, button_pin: int, callback=None,
                 bouncetime_ms: int = 800,
                 require_release: bool = True,
                 confirm_ms: int = 30):
        self.button_pin = button_pin
        self.callback = callback
        self._debounce_s = max(bouncetime_ms, 1) / 1000.0
        self._confirm_s = max(confirm_ms, 0) / 1000.0
        self._last_fire = 0.0
        self._require_release = require_release

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        try:
            GPIO.remove_event_detect(self.button_pin)
        except Exception:
            pass

        GPIO.add_event_detect(
            self.button_pin,
            GPIO.FALLING,                 # expecting press to pull to GND
            callback=self._internal_callback,
            bouncetime=bouncetime_ms
        )

    def _internal_callback(self, channel):
        now = time.monotonic()
        # software gap guard
        if now - self._last_fire < self._debounce_s:
            return

        # short settle, then confirm it is still LOW
        time.sleep(self._confirm_s)
        if GPIO.input(self.button_pin) != 0:
            return

        self._last_fire = now
        if callable(self.callback):
            self.callback()

        if self._require_release:
            # block until released so a long hold doesn't retrigger
            while GPIO.input(self.button_pin) == 0:
                time.sleep(0.01)

    def cleanup(self):
        try:
            GPIO.remove_event_detect(self.button_pin)
        except Exception:
            pass
        GPIO.cleanup(self.button_pin)
