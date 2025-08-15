# ===============================================================
#  Thermostat – single-file, modular refactor
#  HW: AHT20, 3 buttons, 2 LEDs, 16x2 LCD
#  States: INIT, OFF, HEAT_IDLE, HEAT_DEMAND, COOL_IDLE, COOL_DEMAND, FAULT
#  Buttons: MODE cycles OFF→HEAT→COOL→OFF; UP/DOWN adjust SP and re-evaluate
#  LCD: L1 time; L2 alternates T/RH and STATE/SP every 2 s
#  UART: CSV every 30 s to /dev/serial0 (falls back to print)
#  Notes: LEDs are on BCM 23/24 -> software PWM via RPi.GPIO (OK for demo).
#         For rock-solid fades, consider BCM 18/13 (hardware PWM) later.
# ===============================================================

from __future__ import annotations

import os, sys, time, threading
from typing import Optional, Callable, Dict
from dataclasses import dataclass
from enum import Enum

# Make ./src importable (uses your existing files)
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "src")))
from aht20_sensor import AHT20Sensor, AHT20InitError  # type: ignore
from button_handler import ButtonHandler                # type: ignore
from lcd_display import LCDDisplay                      # type: ignore

import RPi.GPIO as GPIO
try:
    import serial  # pyserial
except Exception:
    serial = None


# ------------------------- Config -------------------------

# Buttons (BCM)
PIN_BTN_MODE = 18
PIN_BTN_UP   = 20
PIN_BTN_DOWN = 21

# LEDs (BCM) — these pins use software PWM in RPi.GPIO
PIN_LED_HEAT = 23
PIN_LED_COOL = 24

# Thermostat tuning
HYST_F         = 1.0
SP_MIN_F       = 50.0
SP_MAX_F       = 86.0
DEFAULT_SP_F   = 72.0

# Sensor sampling / averaging
SENSOR_HZ      = 1.0
AVG_SAMPLES    = 3
AVG_INTERVAL_S = 0.12

# UI / cadence
LCD_ALT_SEC    = 2.0
UART_PERIOD    = 30.0
LOOP_PERIOD    = 0.10

# Buttons debounce/guard
BOUNCETIME_MS  = 50
PRESS_GUARD_S  = 0.18


# ------------------------- Types -------------------------

class Mode(str, Enum):
    OFF = "OFF"
    HEAT = "HEAT"
    COOL = "COOL"

class State(str, Enum):
    INIT = "INIT"
    OFF = "OFF"
    HEAT_IDLE = "HEAT_IDLE"
    HEAT_DEMAND = "HEAT_DEMAND"
    COOL_IDLE = "COOL_IDLE"
    COOL_DEMAND = "COOL_DEMAND"
    FAULT = "FAULT"

@dataclass(frozen=True)
class SensorReading:
    temp_f: float
    rh: float
    ts: float

@dataclass(frozen=True)
class SensorError:
    message: str
    ts: float

@dataclass(frozen=True)
class ButtonEvent:
    which: str  # "MODE" | "UP" | "DOWN"
    ts: float

@dataclass(frozen=True)
class LcdViewModel:
    line1: str
    line2: str

@dataclass(frozen=True)
class StateSnapshot:
    mode: Mode
    state: State
    sp_f: float
    temp_f: Optional[float]
    rh: Optional[float]
    heat_demand: bool
    cool_demand: bool
    ts: float


# ------------------------- Drivers (thin) -------------------------

class LedDriver:
    """Simple PWM LED driver with solid and fade. Software PWM is fine for a demo."""
    def __init__(self, pin: int, freq_hz: int = 400) -> None:
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, freq_hz)
        self._pwm.start(0.0)
        self._fade = False

    def solid(self, duty_pct: float = 40.0) -> None:
        self._fade = False
        self._pwm.ChangeDutyCycle(max(0.0, min(100.0, duty_pct)))

    def fade_tick(self) -> None:
        """Call periodically to animate a breathing effect if fade is enabled."""
        if not self._fade:
            return
        t = time.monotonic()
        # triangle wave 0..1..0 over ~1.8 s
        phase = (t % 1.8) / 1.8
        tri = 2.0 * (0.5 - abs(phase - 0.5))  # 0..1..0
        duty = 10.0 + (80.0 - 10.0) * tri
        self._pwm.ChangeDutyCycle(duty)

    def set_fade(self, enable: bool = True) -> None:
        self._fade = enable
        if not enable:
            self._pwm.ChangeDutyCycle(0.0)

    def off(self) -> None:
        self._fade = False
        self._pwm.ChangeDutyCycle(0.0)

    def cleanup(self) -> None:
        try: self._pwm.stop()
        except Exception: pass


class SensorDriver:
    """Serialized AHT20 access with averaging."""
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sensor = AHT20Sensor(settle_s=0.2)

    def read(self) -> SensorReading:
        with self._lock:
            r = self._sensor.read_avg(samples=AVG_SAMPLES, interval_s=AVG_INTERVAL_S)
        t_f = r.temperature_c * 9.0 / 5.0 + 32.0
        return SensorReading(temp_f=t_f, rh=r.humidity_rh, ts=time.monotonic())


class LcdDriver:
    def __init__(self) -> None:
        self._lcd = LCDDisplay()
        self._last: tuple[str, str] | None = None

    def show(self, vm: LcdViewModel) -> None:
        pair = (vm.line1, vm.line2)
        if pair != self._last:
            self._lcd.show_message(vm.line1, vm.line2)
            self._last = pair

    def show_fault(self) -> None:
        self._lcd.show_message("FAULT", "Sensor error")

    def cleanup(self) -> None:
        self._lcd.cleanup()


class ButtonsDriver:
    """ISR posts ButtonEvent via provided callback."""
    def __init__(self, publish: Callable[[ButtonEvent], None]) -> None:
        self._publish = publish
        self._last_ts = 0.0
        self._mk(PIN_BTN_MODE, "MODE")
        self._mk(PIN_BTN_UP,   "UP")
        self._mk(PIN_BTN_DOWN, "DOWN")

    def _mk(self, pin: int, name: str) -> None:
        def isr():
            now = time.monotonic()
            if now - self._last_ts >= PRESS_GUARD_S:
                self._last_ts = now
                self._publish(ButtonEvent(which=name, ts=now))
        ButtonHandler(pin, isr, bouncetime_ms=BOUNCETIME_MS)

    def cleanup(self) -> None:
        pass


class UartSink:
    """Write CSV snapshots to /dev/serial0 or print."""
    def __init__(self) -> None:
        self._ser = None
        if serial:
            try:
                self._ser = serial.Serial("/dev/serial0", 115200, timeout=0)
            except Exception:
                self._ser = None

    def write_snapshot(self, s: StateSnapshot) -> None:
        csv = f"{s.mode.value},{int(s.heat_demand)},{int(s.cool_demand)},{s.temp_f if s.temp_f is not None else 'nan'},{s.rh if s.rh is not None else 'nan'},{s.sp_f:.2f}\n"
        if self._ser:
            try:
                self._ser.write(csv.encode("utf-8")); return
            except Exception:
                pass
        print(csv, end="")


# ------------------------- FSM (pure logic) -------------------------

class ThermostatFSM:
    """Pure thermostat logic: no GPIO here."""
    def __init__(self) -> None:
        self.state: State = State.INIT
        self.mode: Mode = Mode.OFF
        self.sp_f: float = DEFAULT_SP_F
        self.t_f: Optional[float] = None
        self.rh: Optional[float] = None

        self._next_lcd: float = 0.0
        self._lcd_page: int = 0
        self._next_uart: float = time.monotonic() + UART_PERIOD

    # Inputs
    def on_sensor(self, r: SensorReading) -> None:
        self.t_f, self.rh = r.temp_f, r.rh
        if self.state == State.INIT:
            self._enter(State.OFF)
            return
        if self.state == State.HEAT_IDLE and self.t_f <= self.sp_f - HYST_F:
            self._enter(State.HEAT_DEMAND)
        elif self.state == State.HEAT_DEMAND and self.t_f >= self.sp_f:
            self._enter(State.HEAT_IDLE)
        elif self.state == State.COOL_IDLE and self.t_f >= self.sp_f + HYST_F:
            self._enter(State.COOL_DEMAND)
        elif self.state == State.COOL_DEMAND and self.t_f <= self.sp_f:
            self._enter(State.COOL_IDLE)

    def on_sensor_error(self, e: SensorError) -> None:
        self._enter(State.FAULT)

    def on_button(self, ev: ButtonEvent) -> None:
        if ev.which == "MODE":
            if self.state == State.OFF:
                self._enter(State.HEAT_IDLE)
            elif self.state.name.startswith("HEAT"):
                self._enter(State.COOL_IDLE)
            elif self.state.name.startswith("COOL"):
                self._enter(State.OFF)
        elif ev.which == "UP":
            self._adjust_sp(+1.0)
        elif ev.which == "DOWN":
            self._adjust_sp(-1.0)

    # Periodic tick: produce LED intent, LCD VM, optional snapshot
    def tick(self, now: float) -> tuple[str, Optional[LcdViewModel], Optional[StateSnapshot]]:
        # LED intent: "off" | "heat_solid" | "heat_fade" | "cool_solid" | "cool_fade" | "fault_blink"
        if self.state == State.OFF:
            led = "off"
        elif self.state == State.HEAT_IDLE:
            led = "heat_solid"
        elif self.state == State.HEAT_DEMAND:
            led = "heat_fade"
        elif self.state == State.COOL_IDLE:
            led = "cool_solid"
        elif self.state == State.COOL_DEMAND:
            led = "cool_fade"
        else:
            led = "fault_blink"

        vm: Optional[LcdViewModel] = None
        if now >= self._next_lcd:
            self._next_lcd = now + LCD_ALT_SEC
            self._lcd_page ^= 1
            l1 = time.strftime("%m/%d %H:%M:%S")
            if self._lcd_page == 0 and self.t_f is not None and self.rh is not None:
                l2 = f"T:{self.t_f:4.1f}F RH:{self.rh:4.1f}%"
            else:
                l2 = f"{self.mode.value}  SP:{self.sp_f:.0f}F"
            vm = LcdViewModel(l1[:16], l2[:16])

        snap: Optional[StateSnapshot] = None
        if now >= self._next_uart:
            self._next_uart = now + UART_PERIOD
            snap = StateSnapshot(
                mode=self.mode,
                state=self.state,
                sp_f=self.sp_f,
                temp_f=self.t_f,
                rh=self.rh,
                heat_demand=(self.state == State.HEAT_DEMAND),
                cool_demand=(self.state == State.COOL_DEMAND),
                ts=now,
            )
        return led, vm, snap

    # internals
    def _enter(self, s: State) -> None:
        self.state = s
        self.mode = (
            Mode.OFF if s == State.OFF
            else Mode.HEAT if s.name.startswith("HEAT")
            else Mode.COOL
        )
        self._next_lcd = 0.0  # force refresh on state/SP changes

    def _adjust_sp(self, delta: float) -> None:
        self.sp_f = max(SP_MIN_F, min(SP_MAX_F, self.sp_f + delta))
        self._next_lcd = 0.0


# ------------------------- App (wires everything) -------------------------

class App:
    def __init__(self) -> None:
        print("🟢 Init…")
        GPIO.setmode(GPIO.BCM)

        # Drivers
        self.lcd = LcdDriver()
        try:
            self.sensor = SensorDriver()
        except AHT20InitError as ex:
            self.lcd.show_fault()
            raise

        self.led_heat = LedDriver(PIN_LED_HEAT)
        self.led_cool = LedDriver(PIN_LED_COOL)

        # FSM + event wiring
        self.fsm = ThermostatFSM()

        # Button ISR → ButtonEvent → FSM
        self.buttons = ButtonsDriver(lambda ev: self.fsm.on_button(ev))

        # Sampling
        self._next_sample = time.monotonic()

        # UART
        self.uart = UartSink()

    # -- main loop --
    def run(self) -> None:
        print("🚀 Thermostat running…")
        try:
            while True:
                now = time.monotonic()

                # Sensor at fixed rate
                if now >= self._next_sample:
                    self._next_sample = now + (1.0 / SENSOR_HZ)
                    try:
                        self.fsm.on_sensor(self.sensor.read())
                    except Exception as ex:
                        self.fsm.on_sensor_error(SensorError(message=str(ex), ts=now))

                # FSM tick → outputs
                led_intent, vm, snap = self.fsm.tick(now)
                self._drive_leds(led_intent, now)
                if vm:   self.lcd.show(vm)
                if snap: self.uart.write_snapshot(snap)

                time.sleep(LOOP_PERIOD)
        except KeyboardInterrupt:
            pass
        finally:
            self.cleanup()

    # -- outputs --
    def _drive_leds(self, intent: str, now: float) -> None:
        # fault blink: alternate solid red/blue at ~1 Hz
        if intent == "off":
            self.led_heat.off(); self.led_cool.off()
        elif intent == "heat_solid":
            self.led_heat.solid(40); self.led_heat.set_fade(False)
            self.led_cool.off()
        elif intent == "heat_fade":
            self.led_heat.set_fade(True); self.led_cool.off()
        elif intent == "cool_solid":
            self.led_cool.solid(40); self.led_cool.set_fade(False)
            self.led_heat.off()
        elif intent == "cool_fade":
            self.led_cool.set_fade(True); self.led_heat.off()
        else:  # fault_blink
            if int(now * 2) % 2 == 0:
                self.led_heat.solid(40); self.led_cool.off()
            else:
                self.led_cool.solid(40); self.led_heat.off()

        # run fade animation step
        self.led_heat.fade_tick()
        self.led_cool.fade_tick()

    def cleanup(self) -> None:
        print("🛑 Shutting down…")
        try: self.lcd.cleanup()
        except Exception: pass
        try:
            self.led_heat.cleanup(); self.led_cool.cleanup()
        except Exception: pass
        GPIO.cleanup()
        print("✅ GPIO cleaned up")


# ------------------------- Entrypoint -------------------------

if __name__ == "__main__":
    App().run()
