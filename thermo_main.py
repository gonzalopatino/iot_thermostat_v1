# ===============================================================
#  Project: Thermostat (AHT20 + Buttons + LEDs + 16x2 LCD)
#  States : INIT, OFF, HEAT_IDLE, HEAT_DEMAND, COOL_IDLE, COOL_DEMAND, FAULT
#  Notes  : - BTN_MODE cycles OFF → HEAT → COOL → OFF
#           - BTN_UP / BTN_DOWN adjust set point (SP) and re-evaluate demand
#           - LEDs: *_IDLE = solid, *_DEMAND = fade (PWM)
#           - LCD: L1 = time; L2 alternates between "T/RH" and "STATE, SP"
#           - UART CSV every 30 s; prints if /dev/serial0 unavailable
# ===============================================================

from __future__ import annotations

import os, sys, time, math, threading
from typing import Callable, Dict, Optional

# Make ./src importable
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "src")))

from aht20_sensor import AHT20Sensor, AHT20InitError     # type: ignore
from button_handler import ButtonHandler                  # type: ignore
from lcd_display import LCDDisplay                        # type: ignore

import RPi.GPIO as GPIO
try:
    import serial   # pyserial
except Exception:
    serial = None   # fallback to print

# ---------------- Pin plan (BCM) ----------------
PIN_BTN_MODE  = 18
PIN_BTN_UP    = 20
PIN_BTN_DOWN  = 21
PIN_LED_HEAT  = 23   # PWM0-capable
PIN_LED_COOL  = 24   # PWM1-capable

class LedDriver:
    """Tiny PWM LED driver: solid, fade, off."""
    def __init__(self, pin: int, freq_hz: int = 400) -> None:
        GPIO.setup(pin, GPIO.OUT)
        self.pin = pin
        self.pwm = GPIO.PWM(pin, freq_hz)
        self.pwm.start(0.0)  # duty 0..100
        self.mode = "off"

    def solid(self, duty: float = 40.0) -> None:
        self.mode = "solid"
        self.pwm.ChangeDutyCycle(max(0.0, min(100.0, duty)))

    def fade(self, t: float, period: float = 1.8, min_dc: float = 10.0, max_dc: float = 80.0) -> None:
        self.mode = "fade"
        # nice slow triangle
        phase = (t % period) / period
        tri = 2.0 * (0.5 - abs(phase - 0.5))  # 0..1..0
        dc = min_dc + (max_dc - min_dc) * tri
        self.pwm.ChangeDutyCycle(dc)

    def off(self) -> None:
        self.mode = "off"
        self.pwm.ChangeDutyCycle(0.0)

    def cleanup(self) -> None:
        try: self.pwm.stop()
        except Exception: pass


class App:
    # ---------- Tunables ----------
    HYST_F: float = 1.0                 # hysteresis (Fahrenheit)
    SP_MIN_F: float = 50.0
    SP_MAX_F: float = 86.0
    DEFAULT_SP_F: float = 72.0

    SENSOR_HZ: float = 1.0              # sample rate
    AVG_SAMPLES: int = 3
    AVG_INTERVAL_S: float = 0.12

    LOOP_PERIOD: float = 0.10
    LCD_ALT_SEC: float = 2.0
    UART_PERIOD: float = 30.0

    PRESS_GUARD: float = 0.18           # ISR global guard
    BOUNCETIME_MS: int = 50             # hardware debounce in ISR

    def __init__(self) -> None:
        print("🟢 Initializing application...")
        GPIO.setmode(GPIO.BCM)

        # LCD + Sensor
        self.lcd = LCDDisplay()
        try:
            self.sensor = AHT20Sensor(settle_s=0.2)
            print("AHT20 sensor initialized successfully.")
        except AHT20InitError as ex:
            self._fatal("AHT20 init err", str(ex))

        # LED PWM drivers
        self.led_heat = LedDriver(PIN_LED_HEAT)
        self.led_cool = LedDriver(PIN_LED_COOL)

        # --- Buttons: ISR only sets events ---
        self._ev_mode  = threading.Event()
        self._ev_up    = threading.Event()
        self._ev_down  = threading.Event()
        self._last_press_ts = 0.0

        self.btn_mode = ButtonHandler(PIN_BTN_MODE,  self._mk_isr(self._ev_mode),  bouncetime_ms=self.BOUNCETIME_MS)
        self.btn_up   = ButtonHandler(PIN_BTN_UP,    self._mk_isr(self._ev_up),    bouncetime_ms=self.BOUNCETIME_MS)
        self.btn_down = ButtonHandler(PIN_BTN_DOWN,  self._mk_isr(self._ev_down),  bouncetime_ms=self.BOUNCETIME_MS)

        # --- State variables ---
        self.state: str = "INIT"
        self.mode: str = "OFF"          # visible mode: OFF|HEAT|COOL
        self.SP_f: float = self.DEFAULT_SP_F
        self.T_f: Optional[float] = None
        self.RH: Optional[float] = None
        self.heat_demand: bool = False
        self.cool_demand: bool = False

        # timers
        now = time.monotonic()
        self._next_sample = now
        self._next_lcd = now
        self._lcd_page = 0              # 0: T/RH, 1: STATE/SP
        self._next_uart = now + self.UART_PERIOD

        # locks
        self._i2c_lock = threading.Lock()

        # dispatch
        self._handlers: Dict[str, Callable[[], None]] = {
            "INIT": self._state_init,
            "OFF": self._state_off,
            "HEAT_IDLE": self._state_heat_idle,
            "HEAT_DEMAND": self._state_heat_demand,
            "COOL_IDLE": self._state_cool_idle,
            "COOL_DEMAND": self._state_cool_demand,
            "FAULT": self._state_fault,
        }

        # UART (optional)
        self._ser = None
        if serial:
            try:
                self._ser = serial.Serial("/dev/serial0", 115200, timeout=0)
            except Exception:
                self._ser = None

    # --------- ISR helpers ---------
    def _mk_isr(self, ev: threading.Event):
        def _isr():
            now = time.monotonic()
            if now - self._last_press_ts >= self.PRESS_GUARD:
                self._last_press_ts = now
                ev.set()
        return _isr

    # --------- States ---------
    def _state_init(self) -> None:
        # First good sensor read → OFF; else FAULT
        if self._maybe_sample():
            if self.T_f is not None:
                self._enter_off()

    def _state_off(self) -> None:
        self.mode = "OFF"
        self.led_heat.off(); self.led_cool.off()
        self._lcd_if_due()
        self._uart_if_due()

        # mode cycle
        if self._take(self._ev_mode):
            self._enter_heat_idle()

        # SP adjustments still allowed while OFF
        if self._take(self._ev_up):   self._adjust_sp(+1)
        if self._take(self._ev_down): self._adjust_sp(-1)

        self._maybe_sample()  # keep current T/RH fresh for display

    def _state_heat_idle(self) -> None:
        self.mode = "HEAT"
        self.led_heat.solid(); self.led_cool.off()
        self._common_operational()

        # demand evaluation
        if self.T_f is not None and self.T_f <= self.SP_f - self.HYST_F:
            self._enter_heat_demand()

    def _state_heat_demand(self) -> None:
        self.mode = "HEAT"
        self.led_heat.fade(time.monotonic()); self.led_cool.off()
        self._common_operational()

        if self.T_f is not None and self.T_f >= self.SP_f:
            self._enter_heat_idle()

    def _state_cool_idle(self) -> None:
        self.mode = "COOL"
        self.led_cool.solid(); self.led_heat.off()
        self._common_operational()

        if self.T_f is not None and self.T_f >= self.SP_f + self.HYST_F:
            self._enter_cool_demand()

    def _state_cool_demand(self) -> None:
        self.mode = "COOL"
        self.led_cool.fade(time.monotonic()); self.led_heat.off()
        self._common_operational()

        if self.T_f is not None and self.T_f <= self.SP_f:
            self._enter_cool_idle()

    def _state_fault(self) -> None:
        # Blink both LEDs slowly; show FAULT
        t = time.monotonic()
        if int(t * 2) % 2 == 0:
            self.led_heat.solid(40); self.led_cool.off()
        else:
            self.led_cool.solid(40); self.led_heat.off()
        self.lcd.show_message("FAULT", "Sensor error")
        # allow retry by pressing MODE
        if self._take(self._ev_mode):
            self._enter_off()

    # --------- Common helpers ---------
    def _common_operational(self) -> None:
        """Read buttons, sensor, lcd, uart in HEAT/COOL."""
        # Mode cycle
        if self._take(self._ev_mode):
            if self.state.startswith("HEAT"):
                self._enter_cool_idle()
            elif self.state.startswith("COOL"):
                self._enter_off()
            return

        # SP adjust then immediately re-evaluate demand
        sp_changed = False
        if self._take(self._ev_up):   sp_changed = self._adjust_sp(+1) or sp_changed
        if self._take(self._ev_down): sp_changed = self._adjust_sp(-1) or sp_changed

        self._maybe_sample()

        # If SP changed, force LCD refresh
        if sp_changed:
            self._next_lcd = 0.0

        self._lcd_if_due()
        self._uart_if_due()

    def _lcd_if_due(self) -> None:
        now = time.monotonic()
        if now < self._next_lcd:
            return
        self._next_lcd = now + self.LCD_ALT_SEC
        self._lcd_page ^= 1

        line1 = time.strftime("%m/%d %H:%M:%S")
        if self._lcd_page == 0 and self.T_f is not None and self.RH is not None:
            line2 = f"T:{self.T_f:4.1f}F RH:{self.RH:4.1f}%"
        else:
            line2 = f"{self.mode}  SP:{self.SP_f:.0f}F"
        self.lcd.show_message(line1[:16], line2[:16])

    def _uart_if_due(self) -> None:
        now = time.monotonic()
        if now < self._next_uart:
            return
        self._next_uart = now + self.UART_PERIOD
        # Build snapshot
        heat = self.state.startswith("HEAT")
        cool = self.state.startswith("COOL")
        heat_dem = self.state == "HEAT_DEMAND"
        cool_dem = self.state == "COOL_DEMAND"
        temp = f"{self.T_f:.2f}" if self.T_f is not None else "nan"
        rh   = f"{self.RH:.2f}" if self.RH is not None else "nan"
        csv = f"{self.mode},{'1' if heat_dem else '0'},{'1' if cool_dem else '0'},{temp},{rh},{self.SP_f:.2f}\n"
        if self._ser:
            try: self._ser.write(csv.encode("utf-8"))
            except Exception: print(csv, end="")
        else:
            print(csv, end="")

    def _maybe_sample(self) -> bool:
        """Sample sensor at SENSOR_HZ with averaging. Returns True if a read happened."""
        now = time.monotonic()
        if now < self._next_sample:
            return False
        self._next_sample = now + (1.0 / self.SENSOR_HZ)
        if not self._i2c_lock.acquire(blocking=False):
            return False
        try:
            r = self.sensor.read_avg(self.AVG_SAMPLES, self.AVG_INTERVAL_S)
            self.T_f = r.temperature_c * 9.0 / 5.0 + 32.0
            self.RH = r.humidity_rh
            return True
        except Exception as ex:
            print(f"Sensor read error: {ex}")
            self._enter_fault()
            return False
        finally:
            self._i2c_lock.release()

    def _adjust_sp(self, delta: int) -> bool:
        new = max(self.SP_MIN_F, min(self.SP_MAX_F, self.SP_f + float(delta)))
        changed = (abs(new - self.SP_f) > 1e-6)
        self.SP_f = new
        return changed

    def _take(self, ev: threading.Event) -> bool:
        if ev.is_set():
            ev.clear()
            return True
        return False

    # --------- Transitions ---------
    def _enter_off(self) -> None:
        self.state = "OFF"
        self.lcd.show_message("OFF", "Mode=HEAT/COOL")
        self.led_heat.off(); self.led_cool.off()
        print("→ OFF")

    def _enter_heat_idle(self) -> None:
        self.state = "HEAT_IDLE"
        self.led_heat.solid(); self.led_cool.off()
        self._next_lcd = 0.0
        print("→ HEAT_IDLE")

    def _enter_heat_demand(self) -> None:
        self.state = "HEAT_DEMAND"
        self._next_lcd = 0.0
        print("→ HEAT_DEMAND")

    def _enter_cool_idle(self) -> None:
        self.state = "COOL_IDLE"
        self.led_cool.solid(); self.led_heat.off()
        self._next_lcd = 0.0
        print("→ COOL_IDLE")

    def _enter_cool_demand(self) -> None:
        self.state = "COOL_DEMAND"
        self._next_lcd = 0.0
        print("→ COOL_DEMAND")

    def _enter_fault(self) -> None:
        self.state = "FAULT"
        print("→ FAULT")

    # --------- Main loop ---------
    def run(self) -> None:
        print("🚀 Thermostat running...")
        try:
            while True:
                handler = self._handlers.get(self.state, None)
                if handler:
                    handler()
                time.sleep(self.LOOP_PERIOD)
        except KeyboardInterrupt:
            pass
        finally:
            self._cleanup()

    # --------- Utilities ---------
    def _fatal(self, l1: str, l2: str) -> None:
        self.lcd.show_message(l1, l2)
        raise SystemExit(l1 + " " + l2)

    def _cleanup(self) -> None:
        print("🛑 Shutting down...")
        try: self.lcd.cleanup()
        except Exception: pass
        try:
            self.btn_mode.cleanup(); self.btn_up.cleanup(); self.btn_down.cleanup()
        except Exception: pass
        self.led_heat.cleanup(); self.led_cool.cleanup()
        GPIO.cleanup()
        print("✅ GPIO cleaned up")

if __name__ == "__main__":
    App().run()
