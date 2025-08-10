# ===============================================================
#  Project: AHT20 + Button + 16x2 LCD demo (IDLE -> READING FSM)
#  File   : main.py
#  Author : gonzalo patino professional
#  Notes  : Single-class, minimal, production-style structure.
#           - Two states: "IDLE" and "READING"
#           - IDLE waits for a button press
#           - READING lasts 5 s, shows Temp + RH on line 1, timestamp on line 2
#           - While READING, a button press toggles C/F after an ignore window
#           - ISR never drives logic directly, it only sets a flag
#           - I2C reads are serialized to prevent bus contention
#           - LCD flicker minimized by updating only when content changes
#             (lcd_display.py caches last lines; keep that change in place)
#  Best practices used:
#    * Clear state model and transitions
#    * No cross-thread UI or I2C calls (ISR posts an event only)
#    * Timing constants centralized at the top
#    * Defensive coding around hardware access (locks, guards)
#    * Type hints and small, testable methods
# ===============================================================

from __future__ import annotations

import os
import sys
import time
import threading
from typing import Callable, Dict

# Make ./src importable
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "src")))

from aht20_sensor import AHT20Sensor, AHT20InitError  # type: ignore
from button_handler import ButtonHandler               # type: ignore
from lcd_display import LCDDisplay                     # type: ignore


class App:
    """Minimal two-state application for AHT20 + LCD with a debounced button."""

    # ---------- Tunables ----------
    READ_SECONDS: float = 5.0           # How long we stay in READING
    REFRESH_PERIOD: float = 0.25        # Main loop period and display refresh cadence
    ENTER_IGNORE_PRESS: float = 0.6     # Ignore button presses right after entering READING
    PRESS_GUARD: float = 0.5            # Global guard between ISR press events
    BOUNCETIME_MS: int = 700            # Hardware ISR bouncetime for ButtonHandler
    AVG_SAMPLES: int = 3                # Sensor averaging samples
    AVG_INTERVAL_S: float = 0.12        # Interval between averaged reads

    def __init__(self) -> None:
        print("🟢 Initializing application...")

        # Hardware setup
        self.lcd = LCDDisplay()
        try:
            self.sensor = AHT20Sensor(settle_s=0.2)
            print("✅ AHT20 sensor initialized successfully.")
        except AHT20InitError as ex:
            self.lcd.show_message("AHT20 init err", "Check wiring")
            print(f"❌ AHT20 init error: {ex}")
            raise

        # Button ISR -> event flag (no logic in ISR)
        self._button_event = threading.Event()
        self._last_press_ts = 0.0
        self.button = ButtonHandler(
            button_pin=18,
            callback=self._on_button_isr,
            bouncetime_ms=self.BOUNCETIME_MS,
        )
        print("✅ Button handler initialized.")

        # State variables
        self.state: str = "IDLE"         # "IDLE" or "READING"
        self._celsius: bool = True
        self._read_started: float = 0.0
        self._ignore_until: float = 0.0
        self._next_refresh: float = 0.0

        # Protect I2C transactions from overlap
        self._i2c_lock = threading.Lock()

        # "Switch-case" mapping for states
        self._handlers: Dict[str, Callable[[], None]] = {
            "IDLE": self._state_idle,
            "READING": self._state_reading,
        }

        # Enter initial state once
        self._enter_idle()

    # ---------- ISR ----------
    def _on_button_isr(self) -> None:
        """GPIO ISR. Only posts an event with a simple time guard."""
        now = time.monotonic()
        if now - self._last_press_ts >= self.PRESS_GUARD:
            self._last_press_ts = now
            self._button_event.set()

    # ---------- State transitions ----------
    def _enter_idle(self) -> None:
        self.state = "IDLE"
        self.lcd.show_message("Idle", "Press button")
        print("🔄 State changed -> IDLE")

    def _enter_reading(self) -> None:
        self.state = "READING"
        self._celsius = True
        self._read_started = time.monotonic()
        self._ignore_until = self._read_started + self.ENTER_IGNORE_PRESS
        self._next_refresh = 0.0
        print("🔄 State changed -> READING (°C)")
        self._update_display()  # Immediate first reading

    # ---------- State handlers ----------
    def _state_idle(self) -> None:
        """Wait for a debounced press, then enter READING."""
        if self._button_event.is_set():
            self._button_event.clear()
            self._enter_reading()

    def _state_reading(self) -> None:
        """Periodic display updates, toggle units on valid press, exit after timeout."""
        now = time.monotonic()

        # Handle a press as toggle after the ignore window
        if self._button_event.is_set():
            self._button_event.clear()
            if now >= self._ignore_until:
                self._celsius = not self._celsius
                unit = "°C" if self._celsius else "°F"
                print(f"🔁 Unit toggled -> {unit}")
                self._update_display()

        # Periodic update while reading
        if now >= self._next_refresh:
            self._update_display()

        # Exit back to idle after the time window
        if now - self._read_started >= self.READ_SECONDS:
            self._enter_idle()

    # ---------- Helpers ----------
    def _update_display(self) -> None:
        """Read sensor and refresh LCD. No-ops if another read is in progress."""
        if not self._i2c_lock.acquire(blocking=False):
            return
        try:
            reading = self.sensor.read_avg(samples=self.AVG_SAMPLES, interval_s=self.AVG_INTERVAL_S)
            if self._celsius:
                t_val = reading.temperature_c
                t_str = f"T:{t_val:5.1f} C"
            else:
                t_val = reading.temperature_c * 9.0 / 5.0 + 32.0
                t_str = f"T:{t_val:5.1f} F"

            h_str = f"RH:{reading.humidity_rh:5.1f} %"
            line1 = f"{t_str} {h_str}"[:16]  # enforce 16-char width
            ts = time.strftime("%H:%M:%S", time.localtime(reading.timestamp))
            self.lcd.show_message(line1, ts)  # lcd_display.py should cache to avoid flicker

            print(f"📊 Reading -> {line1} | {ts}")
            self._next_refresh = time.monotonic() + self.REFRESH_PERIOD
        finally:
            self._i2c_lock.release()

    # ---------- Main loop ----------
    def run(self) -> None:
        """Main cooperative loop. No busy waiting, predictable cadence."""
        print("🚀 Application started. Waiting for input...")
        try:
            while True:
                handler = self._handlers.get(self.state)
                if handler:
                    handler()
                time.sleep(self.REFRESH_PERIOD)
        except KeyboardInterrupt:
            pass
        finally:
            self._cleanup()

    # ---------- Cleanup ----------
    def _cleanup(self) -> None:
        """Orderly shutdown to leave hardware in a good state."""
        print("🛑 Shutting down...")
        try:
            self.lcd.show_message("Shutting down", "")
            time.sleep(0.4)
            self.lcd.cleanup()
        finally:
            self.button.cleanup()
            print("✅ GPIO cleaned up")


if __name__ == "__main__":
    App().run()