# ===============================================================
#  Project : Embedded Systems – Final Project
#  File    : main.py
#  Author  : Gonzalo Patino
#  School  : Southern New Hampshire University (SNHU)
#  Program : Bachelor of Science in Computer Science
#  Focus   : Embedded Systems
#  Date    : August 2025
#  Notes   : 
#      - Developed as the final project for Embedded Systems course.
#      - Includes hardware interfacing, state machine logic, 
#        and real-time event handling.
# ===============================================================
#
#  Thermostat – single-file with MQTT (Subscribe + Publish)
#  HW: AHT20 (I2C), Buttons MODE/UP/DOWN (BCM 18/20/21), LEDs (BCM 23/24), 16x2 LCD
#  FSM: INIT, OFF, HEAT_IDLE, HEAT_DEMAND, COOL_IDLE, COOL_DEMAND, FAULT
#  Behavior:
#    - MODE cycles OFF → HEAT → COOL → OFF
#    - UP/DOWN adjust set point (SP) and immediately re-evaluate demand
#    - Hysteresis: Heat ON  T <= SP - HYST, OFF T >= SP
#                  Cool ON  T >= SP + HYST, OFF T <= SP
#    - LEDs: *_IDLE = solid, *_DEMAND = fade
#    - LCD: line1=time; line2 alternates {T/RH} and {STATE/SP} every 2 s
#    - UART CSV every 30 s to /dev/serial0 (falls back to stdout)
#    - MQTT:
#        * Subscribe  devices/<DEVICE_ID>/cmd
#          - {"type":"set_mode","value":"OFF|HEAT|COOL"}
#          - {"type":"set_sp","value":72}
#        * Publish    devices/<DEVICE_ID>/telemetry  (JSON)
# ===============================================================

from __future__ import annotations

import os, sys, time, threading, logging, json, datetime
from dataclasses import dataclass
from enum import Enum
from typing import Optional
import json, time, logging
# --------- Logging (human-friendly) ----------
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL, logging.INFO),
    format="%(asctime)s.%(msecs)03d | %(levelname)-5s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("thermostat")

# --------- Make ./src importable for your existing drivers ----------
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "src")))
from aht20_sensor import AHT20Sensor, AHT20InitError  # type: ignore
from button_handler import ButtonHandler                # type: ignore
from lcd_display import LCDDisplay                      # type: ignore

import RPi.GPIO as GPIO
try:
    import serial  # pyserial (optional)
except Exception:
    serial = None

# MQTT (optional). If paho isn't installed, MQTT will be disabled automatically.
try:
    import paho.mqtt.client as mqtt  # type: ignore
except Exception:
    mqtt = None


# =========================
# Configuration constants
# =========================
# Buttons (BCM)
PIN_BTN_MODE = 18
PIN_BTN_UP   = 20
PIN_BTN_DOWN = 21

# LEDs (BCM) — software PWM via RPi.GPIO for demo
PIN_LED_HEAT = 23
PIN_LED_COOL = 24

# Thermostat tuning
HYST_F         = 1.0
SP_MIN_F       = 50.0
SP_MAX_F       = 86.0
DEFAULT_SP_F   = 72.0
SP_STEP_F      = 1.0   # set-point increment per press

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
PRESS_GUARD_S  = 0.18  # additional guard across ISRs

# MQTT config (override with env vars if you like)
MQTT_ENABLED    = os.getenv("MQTT_ENABLED", "1") not in ("0", "false", "False")
MQTT_HOST       = os.getenv("MQTT_HOST", "broker.hivemq.com")   # or your broker
MQTT_PORT       = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USERNAME   = os.getenv("MQTT_USERNAME", "") or None
MQTT_PASSWORD   = os.getenv("MQTT_PASSWORD", "") or None
MQTT_TLS        = os.getenv("MQTT_TLS", "0") in ("1", "true", "True")
DEVICE_ID       = os.getenv("DEVICE_ID", "rpi-thermostat-001")
MQTT_TOPIC_CMD  = os.getenv("MQTT_TOPIC_CMD",  f"devices/{DEVICE_ID}/cmd")
MQTT_TOPIC_TEL  = os.getenv("MQTT_TOPIC_TEL",  f"devices/{DEVICE_ID}/telemetry")
MQTT_KEEPALIVE  = int(os.getenv("MQTT_KEEPALIVE", "45"))
MQTT_QOS        = int(os.getenv("MQTT_QOS", "1"))  # 0|1|2


# =========================
# Typed messages & enums
# =========================
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
class CloudCommand:
    type: str   # "set_mode" | "set_sp"
    value: object
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


# =========================
# Thin hardware drivers
# =========================
class LedDriver:
    """Simple PWM LED driver with 'solid' and 'fade' modes."""
    def __init__(self, pin: int, freq_hz: int = 400) -> None:
        GPIO.setup(pin, GPIO.OUT)
        self._pwm = GPIO.PWM(pin, freq_hz)
        self._pwm.start(0.0)
        self._fade = False

    def solid(self, duty_pct: float = 40.0) -> None:
        self._fade = False
        self._pwm.ChangeDutyCycle(max(0.0, min(100.0, duty_pct)))

    def set_fade(self, enable: bool = True) -> None:
        self._fade = enable
        if not enable:
            self._pwm.ChangeDutyCycle(0.0)

    def fade_tick(self) -> None:
        """Animate fade (call each loop). Triangle: 10%..80%..10% over ~1.8s."""
        if not self._fade:
            return
        t = time.monotonic()
        phase = (t % 1.8) / 1.8
        tri = 2.0 * (0.5 - abs(phase - 0.5))  # 0..1..0
        duty = 10.0 + (80.0 - 10.0) * tri
        self._pwm.ChangeDutyCycle(duty)

    def off(self) -> None:
        self._fade = False
        self._pwm.ChangeDutyCycle(0.0)

    def cleanup(self) -> None:
        try: self._pwm.stop()
        except Exception: pass


class SensorDriver:
    """Serialized access to AHT20 with averaging (thread-safe)."""
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sensor = AHT20Sensor(settle_s=0.2)

    def read(self) -> SensorReading:
        with self._lock:
            r = self._sensor.read_avg(samples=AVG_SAMPLES, interval_s=AVG_INTERVAL_S)
        t_f = r.temperature_c * 9.0 / 5.0 + 32.0
        return SensorReading(temp_f=t_f, rh=r.humidity_rh, ts=time.monotonic())


class LcdDriver:
    """LCD front-end that deduplicates updates to avoid flicker."""
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
    def __init__(self, publish_cb) -> None:
        self._publish = publish_cb
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
                log.debug("BTN %-5s @ %.3fs", name, now)
        ButtonHandler(pin, isr, bouncetime_ms=BOUNCETIME_MS)

    def cleanup(self) -> None:
        pass


class UartSink:
    """Write CSV snapshots to /dev/serial0 or log to console."""
    def __init__(self) -> None:
        self._ser = None
        if serial:
            try:
                self._ser = serial.Serial("/dev/serial0", 115200, timeout=0)
                log.info("UART ready at /dev/serial0 115200 8N1")
            except Exception as ex:
                log.warning("UART unavailable (%s); logging to console", ex)

    def write_snapshot(self, s: StateSnapshot) -> None:
        csv = (
            f"{s.mode.value},"
            f"{int(s.heat_demand)},"
            f"{int(s.cool_demand)},"
            f"{s.temp_f if s.temp_f is not None else 'nan'},"
            f"{s.rh if s.rh is not None else 'nan'},"
            f"{s.sp_f:.2f}\n"
        )
        if self._ser:
            try: self._ser.write(csv.encode("utf-8")); return
            except Exception as ex: log.warning("UART write failed: %s", ex)
        log.info("UART CSV  | %s", csv.strip())


# =========================
# MQTT client wrapper
# =========================
class MqttClient:
    """Paho client wrapper: runs loop in background, passes CloudCommand
    into a callback, and publishes telemetry snapshots as JSON."""
    def __init__(self, on_command_cb) -> None:
        self.enabled = bool(mqtt and MQTT_ENABLED)
        self.on_command_cb = on_command_cb
        self.client = None
        if not self.enabled:
            log.info("MQTT disabled (paho not installed or MQTT_ENABLED=0).")
            return

        cid = f"{DEVICE_ID}"
        self.client = mqtt.Client(client_id=cid, clean_session=True)
        if MQTT_USERNAME:
            self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        if MQTT_TLS:
            self.client.tls_set()  # default OS CA bundle

        # LWT so dashboards can show offline status
        self.client.will_set(MQTT_TOPIC_TEL, json.dumps({"status":"offline","ts":datetime.datetime.utcnow().isoformat()}), qos=MQTT_QOS, retain=True)

        # Callbacks
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message
        self.client.on_disconnect = self._on_disconnect

        # Connect & start network loop
        try:
            self.client.connect(MQTT_HOST, MQTT_PORT, keepalive=MQTT_KEEPALIVE)
            self.client.loop_start()
            log.info("MQTT connecting to %s:%s as %s", MQTT_HOST, MQTT_PORT, cid)
        except Exception as ex:
            self.enabled = False
            log.error("MQTT connect failed: %s", ex)

    # ---- callbacks ----
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            log.info("MQTT connected. Subscribing to %s (QoS=%d)", MQTT_TOPIC_CMD, MQTT_QOS)
            try:
                client.subscribe(MQTT_TOPIC_CMD, qos=MQTT_QOS)
                # Advertise online state
                client.publish(MQTT_TOPIC_TEL, json.dumps({"status":"online","ts":datetime.datetime.utcnow().isoformat()}), qos=MQTT_QOS, retain=True)
            except Exception as ex:
                log.error("MQTT subscribe error: %s", ex)
        else:
            log.error("MQTT connection failed rc=%s", rc)

    def _on_disconnect(self, client, userdata, rc):
        log.warning("MQTT disconnected (rc=%s)", rc)

    def _on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            payload_str = msg.payload.decode("utf-8")
            payload = json.loads(payload_str)

            # --- ThingsBoard RPC (dashboard widgets) ---
            # Topic: v1/devices/me/rpc/request/<reqId>
            # Payload: {"method":"set_sp","params":73} OR {"method":"set_mode","params":"HEAT"}
            if topic.startswith("v1/devices/me/rpc/request/"):
                req_id = topic.split("/")[-1]
                method = str(payload.get("method", "")).lower()
                params = payload.get("params")

                if method == "set_sp":
                    self.on_command_cb(CloudCommand(type="set_sp", value=float(params), ts=time.monotonic()))
                elif method == "set_mode":
                    self.on_command_cb(CloudCommand(type="set_mode", value=str(params).upper(), ts=time.monotonic()))
                else:
                    log.warning("MQTT RPC  | unknown method: %s payload=%s", method, payload)

                # acknowledge back to TB
                client.publish(f"v1/devices/me/rpc/response/{req_id}", json.dumps({"ok": True}), qos=MQTT_QOS)
                return

            # --- Legacy path (if you ever use a plain broker) ---
            # Topic: devices/<DEVICE_ID>/cmd
            # Payload: {"type":"set_sp","value":73} or {"type":"set_mode","value":"HEAT"}
            ctype = str(payload.get("type", "")).lower()
            if ctype == "set_sp":
                self.on_command_cb(CloudCommand(type="set_sp", value=float(payload.get("value")), ts=time.monotonic()))
            elif ctype == "set_mode":
                self.on_command_cb(CloudCommand(type="set_mode", value=str(payload.get("value")).upper(), ts=time.monotonic()))
            else:
                log.warning("MQTT CMD  | unrecognized payload on %s: %s", topic, payload)

        except Exception as ex:
            log.error("MQTT on_message error: %s", ex)


    # ---- publish telemetry ----
    def publish_snapshot(self, s: StateSnapshot) -> None:
        if not self.enabled or not self.client:
            return
        payload = {
            "mode": s.mode.value,
            "state": s.state.value,
            "heat_demand": s.heat_demand,
            "cool_demand": s.cool_demand,
            "sp_f": round(s.sp_f, 2),
            "temp_f": None if s.temp_f is None else round(s.temp_f, 2),
            "rh": None if s.rh is None else round(s.rh, 2),
            "ts": datetime.datetime.utcnow().isoformat()
        }
        try:
            self.client.publish(MQTT_TOPIC_TEL, json.dumps(payload), qos=MQTT_QOS, retain=False)
            log.debug("MQTT PUB  | %s", payload)
        except Exception as ex:
            log.error("MQTT publish failed: %s", ex)

    def close(self) -> None:
        if self.enabled and self.client:
            try:
                self.client.loop_stop()
                self.client.disconnect()
            except Exception:
                pass


# =========================
# Pure FSM (no GPIO calls)
# =========================
class ThermostatFSM:
    """Encapsulates thermostat policy (deterministic, testable)."""

    def __init__(self) -> None:
        self.state: State = State.INIT
        self.mode: Mode = Mode.OFF
        self.sp_f: float = DEFAULT_SP_F
        self.t_f: Optional[float] = None
        self.rh: Optional[float] = None

        self._next_lcd: float = 0.0
        self._lcd_page: int = 0
        self._next_uart: float = time.monotonic() + UART_PERIOD

    # ---- Inputs ----
    def on_sensor(self, r: SensorReading) -> None:
        self.t_f, self.rh = r.temp_f, r.rh
        log.debug("SENSOR    | T=%5.2f F  RH=%5.2f %%", self.t_f, self.rh)

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
        log.error("SENSORERR | %s", e.message)
        self._enter(State.FAULT)

    def on_button(self, ev: ButtonEvent) -> None:
        """Button presses mutate mode/SP; demand is reevaluated on next sample."""
        if ev.which == "MODE":
            if self.state == State.OFF:
                self._enter(State.HEAT_IDLE)
            elif self.state.name.startswith("HEAT"):
                self._enter(State.COOL_IDLE)
            elif self.state.name.startswith("COOL"):
                self._enter(State.OFF)
        elif ev.which == "UP":
            self._adjust_sp(+SP_STEP_F)
        elif ev.which == "DOWN":
            self._adjust_sp(-SP_STEP_F)

    def on_cloud(self, cmd: CloudCommand) -> None:
        """Cloud command handler (maps to same policy as buttons)."""
        if cmd.type == "set_mode":
            val = str(cmd.value).upper()
            if val == "OFF":  self._enter(State.OFF)
            elif val == "HEAT": self._enter(State.HEAT_IDLE)
            elif val == "COOL": self._enter(State.COOL_IDLE)
        elif cmd.type == "set_sp":
            try:
                target = float(cmd.value)
                delta = target - self.sp_f
                self._adjust_sp(delta)
            except Exception:
                log.warning("Bad set_sp value: %s", cmd.value)

    # ---- Periodic outputs ----
    def tick(self, now: float) -> tuple[str, Optional[LcdViewModel], Optional[StateSnapshot]]:
        """Return LED intent, optional LCD update, optional telemetry snapshot."""
        if self.state == State.OFF:            led = "off"
        elif self.state == State.HEAT_IDLE:    led = "heat_solid"
        elif self.state == State.HEAT_DEMAND:  led = "heat_fade"
        elif self.state == State.COOL_IDLE:    led = "cool_solid"
        elif self.state == State.COOL_DEMAND:  led = "cool_fade"
        else:                                   led = "fault_blink"

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

    # ---- Internal helpers ----
    def _enter(self, s: State) -> None:
        prev = self.state
        self.state = s
        self.mode = (
            Mode.OFF if s == State.OFF
            else Mode.HEAT if s.name.startswith("HEAT")
            else Mode.COOL
        )
        self._next_lcd = 0.0  # force LCD refresh on state/SP changes
        log.info("STATE     | %s  →  %s   (mode=%s)", prev.value, s.value, self.mode.value)

    def _adjust_sp(self, delta: float) -> None:
        old = self.sp_f
        self.sp_f = max(SP_MIN_F, min(SP_MAX_F, self.sp_f + delta))
        self._next_lcd = 0.0
        heat_on_at = self.sp_f - HYST_F
        cool_on_at = self.sp_f + HYST_F
        t = self.t_f
        log.info(
            "SETPOINT  | %.1f°F → %.1f°F   (T=%.2f°F  heat<=%.1f  cool>=%.1f)",
            old, self.sp_f, (t if t is not None else float('nan')), heat_on_at, cool_on_at
        )


# =========================
# Application wiring
# =========================
class App:
    """Owns drivers, FSM, MQTT, and the cooperative main loop."""

    def __init__(self) -> None:
        log.info("Init GPIO & drivers…")
        GPIO.setmode(GPIO.BCM)

        # Drivers
        self.lcd = LcdDriver()
        try:
            self.sensor = SensorDriver()
        except AHT20InitError as ex:
            self.lcd.show_fault()
            log.exception("AHT20 init failed: %s", ex)
            raise SystemExit(1)

        self.led_heat = LedDriver(PIN_LED_HEAT)
        self.led_cool = LedDriver(PIN_LED_COOL)

        # FSM and event sources
        self.fsm = ThermostatFSM()
        self.buttons = ButtonsDriver(lambda ev: self.fsm.on_button(ev))

        # MQTT client (optional)
        self.mqtt = MqttClient(self.fsm.on_cloud)

        # Sampling timer
        self._next_sample = time.monotonic()

        # UART sink
        self.uart = UartSink()

    # ---------- Main loop ----------
    def run(self) -> None:
        log.info("Thermostat running (log level: %s)…", LOG_LEVEL)
        try:
            while True:
                now = time.monotonic()

                # 1) Sample sensor at fixed cadence
                if now >= self._next_sample:
                    self._next_sample = now + (1.0 / SENSOR_HZ)
                    try:
                        self.fsm.on_sensor(self.sensor.read())
                    except Exception as ex:
                        self.fsm.on_sensor_error(SensorError(str(ex), now))

                # 2) Tick FSM → intents
                led_intent, vm, snap = self.fsm.tick(now)

                # 3) Drive actuators based on intents
                self._drive_leds(led_intent, now)
                if vm:
                    self.lcd.show(vm)
                if snap:
                    self.uart.write_snapshot(snap)
                    self.mqtt.publish_snapshot(snap)

                time.sleep(LOOP_PERIOD)
        except KeyboardInterrupt:
            log.info("KeyboardInterrupt: exiting")
        finally:
            self.cleanup()

    # ---------- Actuators ----------
    def _drive_leds(self, intent: str, now: float) -> None:
        """Render LED intent. Keep the animation step outside the FSM."""
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
        else:  # fault_blink → alternate red/blue ~1 Hz
            if int(now * 2) % 2 == 0:
                self.led_heat.solid(40); self.led_cool.off()
            else:
                self.led_cool.solid(40); self.led_heat.off()

        # animate fades (no-ops if not in fade mode)
        self.led_heat.fade_tick()
        self.led_cool.fade_tick()

    # ---------- Cleanup ----------
    def cleanup(self) -> None:
        log.info("Cleaning up…")
        try: self.lcd.cleanup()
        except Exception: pass
        try:
            self.led_heat.cleanup(); self.led_cool.cleanup()
        except Exception: pass
        try:
            self.mqtt.close()
        except Exception: pass
        GPIO.cleanup()
        log.info("GPIO cleaned up")


# =========================
# Entrypoint
# =========================
if __name__ == "__main__":
    App().run()
