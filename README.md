# Raspberry Pi Thermostat (AHT20 + LCD + Buttons + LEDs + MQTT + UART)

# CS350 Smart Thermostat – Final Project

## Project Overview  
This project implements a functional prototype of a smart thermostat using a Raspberry Pi, AHT20 temperature and humidity sensor, push buttons, LEDs, and a 16x2 LCD display. The thermostat solves the problem of **controlling heating and cooling systems based on a user-defined set point**. It simulates how a connected thermostat could monitor environmental conditions, switch between heating/cooling states, and communicate data via UART.  

Beyond the local prototype, this project also explores the **next phase of development**: connecting the thermostat to the cloud via MQTT, enabling remote monitoring, historical trend analysis, and remote setpoint/mode changes.  

---

## Reflection  

### 1. What did you do particularly well?  
I did particularly well in **structuring the system around a finite state machine (FSM)**. This design choice ensured that all operational states (OFF, HEAT_IDLE, HEAT_DEMAND, COOL_IDLE, COOL_DEMAND, FAULT) were logically separated and easy to test. I also integrated the LCD, UART output, and LEDs in a consistent way that kept the system synchronized, which improved usability and debugging.  

### 2. Where could you improve?  
I could improve in **code modularization** and **error handling**. While the prototype works reliably, some of the scripts could be broken down into smaller reusable modules to increase maintainability. Additionally, UART and I²C communication could benefit from more robust exception handling and recovery routines.  

### 3. What tools and/or resources are you adding to your support network?  
I added **MQTT and TLS security standards** to my support network, learning how lightweight protocols and secure connections are critical for IoT development. I also used **datasheets, online Raspberry Pi GPIO references, and MQTT documentation** to support design decisions.  

### 4. What skills from this project will be particularly transferable?  
- **Embedded systems design with FSMs** (widely applicable to IoT and control systems).  
- **UART communication and data formatting**, useful in debugging and device-to-device integration.  
- **Cloud integration concepts with MQTT**, directly transferable to future IoT coursework and real-world projects.  
- **Testing and validation strategies** (incremental builds, circuit testing with MultiButtonTest, UART verification).  

### 5. How did you make this project maintainable, readable, and adaptable?  
I ensured maintainability by:  
- Using **consistent naming conventions** for variables, functions, and states.  
- Documenting the system with a **System Requirements Specification (SRS)** and a **Final Report** that explains design choices.  
- Designing the FSM so that **new states or features (like adaptive mode or Wi-Fi integration)** could be added without rewriting the core logic.  
- Providing UART outputs in a **clear, delimited format**, making it easier for future dashboards to parse and integrate.
- I used object-oriented programming, a modular design, component-based approach.

---

## Installation and Usage  

### Requirements  
- Raspberry Pi 4 (or similar with GPIO + I²C + UART support)  
- AHT20 temperature & humidity sensor (I²C)  
- 3x push buttons + resistors (10kΩ pull-ups)  
- 2x LEDs (Red & Blue) with current-limiting resistors  
- 16x2 LCD with I²C backpack  
- Python 3.8+  
- Libraries: `smbus2`, `RPi.GPIO` (or `rpi-lgpio`), `Adafruit_CharLCD`, `datetime`  

### Setup  
1. Clone this repository:  
   ```bash
   git clone https://github.com/YOUR_USERNAME/CS350-Thermostat.git
   cd CS350-Thermostat
   ```

2. Install dependencies:  
   ```bash
   sudo apt-get install python3-smbus python3-dev
   pip install smbus2 RPi.GPIO adafruit-circuitpython-charlcd
   ```

3. Connect hardware according to the **Lab Guide circuit diagram**.  

4. Run the main thermostat program:  
   ```bash
   sudo python3 Thermostat.py
   ```

5. Verify UART output:  
   ```bash
   screen /dev/serial0 9600
   ```
   Output format (every 30 seconds):  
   ```
   state,current_temperature,set_point
   ```

---

## Future Work  
- Implement **adaptive mode** for energy efficiency.  
- Extend dashboard to include **historical trend analysis** and **remote control**.  

---

## License  
This project is released under the **MIT License**.  



A single-file thermostat for Raspberry Pi with:

- **AHT20** temperature/humidity over **I²C**
- **3 buttons** (MODE / UP / DOWN)
- **2 LEDs** (red = heat, blue = cool) with **fade/solid** behavior
- **16×2 LCD** (status + alternating pages)
- **UART CSV** output (every 30 s)
- **MQTT** telemetry + cloud control (ThingsBoard or plain broker)
- Clear **state machine** with **hysteresis** (Fahrenheit)

> Main app: `thermo_main.py` (drivers in `src/`)

---

## Features

- Default setpoint **72.0 °F**
- States: `INIT, OFF, HEAT_IDLE, HEAT_DEMAND, COOL_IDLE, COOL_DEMAND, FAULT`
- Hysteresis:  
  Heat **ON** when `T ≤ SP − HYST`, **OFF** when `T ≥ SP`  
  Cool **ON** when `T ≥ SP + HYST`, **OFF** when `T ≤ SP`
- LEDs: *_IDLE* = **solid**, *_DEMAND* = **fade**
- LCD:
  - Line 1: **date + time**
  - Line 2 (alternates every 2 s): **T/RH** ↔ **MODE + SP**
- UART CSV every **30 s** (3-field minimal or rich 6-field)
- MQTT:
  - Telemetry JSON to cloud
  - Remote commands: **setpoint** + **mode**
  - Optional TB RPC method `getState` for slider auto-sync

---

## Hardware

- **Raspberry Pi 4** (Ubuntu Server or Raspberry Pi OS)
- **AHT20** sensor (I²C, address **0x38**)
- **16×2 LCD** (your `lcd_display.py` decides pinout/I²C backpack)
- **Buttons**: MODE / UP / DOWN
- **LEDs**: red (heat), blue (cool) with current-limit resistors
- (Optional) **USB-to-TTL** for viewing UART on Windows

### GPIO / Pin Map (BCM → Header pin)

| Function | BCM | Header Pin |
|---|---:|---:|
| **MODE button** | 18 | 12 |
| **UP button** | 20 | 38 |
| **DOWN button** | 21 | 40 |
| **LED HEAT (red)** | 23 | 16 |
| **LED COOL (blue)** | 24 | 18 |
| **I²C SDA (AHT20)** | 2 | 3 |
| **I²C SCL (AHT20)** | 3 | 5 |
| **UART TXD0** | 14 | 8 |
| **UART RXD0** | 15 | 10 |
| **3V3 / 5V / GND** | — | 1/2, 6 |

> AHT20 breakouts typically include pull-ups; otherwise add 4.7 kΩ to 3.3 V.  
> **LCD wiring** depends on your `lcd_display.py` (HD44780 4-bit or I²C backpack).

---

## Software Setup

### OS Packages
```bash
sudo apt update
sudo apt install -y python3 python3-pip python3-venv i2c-tools mosquitto-clients
```

### Python deps
```bash
python3 -m venv venv
source venv/bin/activate
pip install RPi.GPIO smbus2 pyserial paho-mqtt
```

### Enable I²C
```bash
# Raspberry Pi OS: raspi-config → Interface Options → I2C=Enable
# Ubuntu Server on Pi:
echo 'dtparam=i2c_arm=on' | sudo tee -a /boot/firmware/config.txt
sudo reboot
```

### Enable UART (Ubuntu Server on Pi)
```bash
sudo usermod -aG dialout $USER
newgrp dialout

# Make UART available on GPIO14/15 and free from Bluetooth/console
sudo bash -c 'cat >> /boot/firmware/config.txt <<EOF
enable_uart=1
dtoverlay=disable-bt
EOF'
sudo sed -i -E 's/console=(serial0|ttyAMA0|ttyS0),[0-9]+ ?//g' /boot/firmware/cmdline.txt
sudo systemctl disable --now serial-getty@ttyAMA0.service 2>/dev/null || true
sudo systemctl disable --now serial-getty@ttyS0.service 2>/dev/null || true
sudo systemctl disable --now hciuart.service 2>/dev/null || true
sudo reboot
```

Verify:
```bash
ls -l /dev/serial0
```

---

## Project Layout

```
iot_thermostat_v1/
├─ thermo_main.py          # main app (single file)
└─ src/
   ├─ aht20_sensor.py      # AHT20 driver
   ├─ button_handler.py    # debounced button ISR helper
   └─ lcd_display.py       # 16x2 LCD driver (HD44780 / I²C backpack)
```

---

## Configuration (Environment Variables)

Common:
```bash
# Optional defaults (already set in code)
LOG_LEVEL=DEBUG            # DEBUG/INFO/WARN
DEVICE_ID=rpi-thermostat-001
UART_SIMPLE=0              # 1 = 3-field CSV (state,temp,sp); 0 = rich CSV
UART_PORT=/dev/serial0     # override if needed
UART_BAUD=115200
```

### Option A — ThingsBoard Cloud (recommended for dashboards)
```bash
DEVICE_ID=rpi-thermostat-001 \
MQTT_ENABLED=1 \
MQTT_HOST=thingsboard.cloud \
MQTT_PORT=1883 \
MQTT_USERNAME="<YOUR_TB_DEVICE_TOKEN>" \
MQTT_PASSWORD= \
MQTT_TOPIC_TEL="v1/devices/me/telemetry" \
MQTT_TOPIC_CMD="v1/devices/me/rpc/request/+" \
LOG_LEVEL=DEBUG \
python3 thermo_main.py
```

**Telemetry keys:** `temp_f, rh, sp_f, mode, state, heat_demand, cool_demand, ts`  
**RPC:**  
- `set_sp` *(number)*  
- `set_mode` *(“OFF” | “HEAT” | “COOL”)*  
- `getState` → `{"sp_f":72,"mode":"HEAT","state":"HEAT_IDLE","temp_f":..., "rh":...}`

> In TB, make a **Slider (RPC)** → On change: `set_sp` (param=Value).  
> Initial value: Execute RPC `getState` → converter `return Number(value.sp_f);` → repeat every 5 s.  
> Add **Buttons** calling `set_mode` with `OFF/HEAT/COOL`.

### Option B — Public Broker (HiveMQ) for quick CLI tests
```bash
DEVICE_ID=rpi-thermostat-001 \
MQTT_ENABLED=1 \
MQTT_HOST=broker.hivemq.com \
MQTT_PORT=1883 \
MQTT_QOS=1 \
LOG_LEVEL=DEBUG \
python3 thermo_main.py
```

**Plain broker topics**  
- Telemetry: `devices/<DEVICE_ID>/telemetry`  
- Commands: `devices/<DEVICE_ID>/cmd` with payload:
  - `{"type":"set_sp","value":73}`
  - `{"type":"set_mode","value":"HEAT"}`

---

## Running

```bash
source venv/bin/activate
LOG_LEVEL=DEBUG python3 thermo_main.py
```

Logs show `SENSOR`, `STATE`, `SETPOINT`, `MQTT PUB`, and `UART` lines.

**Systemd (auto-start)** — `/etc/systemd/system/thermostat.service`:
```ini
[Unit]
Description=Raspberry Pi Thermostat
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/iot_thermostat_v1
Environment="MQTT_ENABLED=1"
Environment="DEVICE_ID=rpi-thermostat-001"
Environment="MQTT_HOST=thingsboard.cloud"
Environment="MQTT_PORT=1883"
Environment="MQTT_USERNAME=<TOKEN>"
Environment="MQTT_PASSWORD="
Environment="MQTT_TOPIC_TEL=v1/devices/me/telemetry"
Environment="MQTT_TOPIC_CMD=v1/devices/me/rpc/request/+"
Environment="LOG_LEVEL=INFO"
ExecStart=/usr/bin/python3 /home/pi/iot_thermostat_v1/thermo_main.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now thermostat
journalctl -u thermostat -f
```

---

## UART Output

- Period: **30 s**

**Minimal (spec-compliant)** when `UART_SIMPLE=1`:
```
<state>,<temp_f>,<sp_f>
# e.g. "heat,71.88,72.00"
```

**Rich CSV** (default):
```
<mode>,<heat_demand>,<cool_demand>,<temp_f>,<rh>,<sp_f>
# e.g. "COOL,0,1,78.62,54.40,72.00"
```

### View on Windows via USB-TTL
- Wiring: Pi **TXD0 (pin 8)** → USB-TTL **RX**, Pi **GND (pin 6)** ↔ **GND**  
- PuTTY: **Serial**, COM#, **115200**, 8-N-1, **No** flow control

---

## State Machine & LED Rules

- `OFF` → both LEDs off  
- `HEAT_IDLE` → **red solid**  
- `HEAT_DEMAND` → **red fade**  
- `COOL_IDLE` → **blue solid**  
- `COOL_DEMAND` → **blue fade**  
- `FAULT` → red/blue alternate 1 Hz (sensor error)

Buttons:
- **MODE**: `OFF → HEAT → COOL → OFF`
- **UP/DOWN**: setpoint ±1 °F (bounded 50–86 °F), updates LCD and re-evaluates demand

---

## Architecture Options (for report)

### Executive summary
- We evaluated **Raspberry Pi**, **Microchip MCU**, and **NXP/Freescale i.MX RT**.  
- **Recommendation:** MVP/pilot on **Raspberry Pi (Pi 4 / CM4)**; prototype MCU (NXP i.MX RT + Wi-Fi module) in parallel for a lower-power SKU.

### Requirements mapping
| Requirement | Raspberry Pi | Microchip MCU | NXP i.MX RT |
|---|---|---|---|
| Peripherals (I²C AHT20, GPIO, LCD, UART) | **Meets** | **Meets** | **Meets** |
| Wi-Fi to cloud | Integrated (many SKUs) | On-chip/module | Module |
| Flash/RAM headroom | **Ample** (GB-class) | MB Flash / 100s KB RAM | ~1 MB SRAM + external Flash |

### Memory sufficiency
- **Linux (Pi):** ample for TLS/MQTT, dashboards, logs, OTA.  
- **MCU path:** target **≥1–2 MB Flash**, **≥512 KB–1 MB RAM** (+QSPI Flash) for TLS/MQTT/OTA.

### Risks & mitigations
- Supply variability (Pi) → Prefer **CM4** with eMMC, qualify alternates.  
- Power budget → MCU variant as Plan B; measure Pi draw.  
- Firmware complexity (MCU) → Use FreeRTOS + mbedTLS + vendor MQTT; keep payloads stable.

---

## Troubleshooting

- **No I²C device**: `sudo i2cdetect -y 1` (AHT20 at **0x38**).  
- **LCD blank/flicker**: verify `lcd_display.py` pinout/backpack address.  
- **UART “/dev/serial0 not found”**: enable UART (see above), reboot, verify symlink.  
- **MQTT no data**: check broker, topics, token (TB), and logs for `MQTT connected`.  
- **TB slider doesn’t update**: set **Initial value → Execute RPC → getState** with converter `return Number(value.sp_f);` and repeat every 5 s.

---

## All Rights Reserved
This project is provided for academic purposes only.  
All rights reserved. Unauthorized use, distribution, or modification is prohibited.  
