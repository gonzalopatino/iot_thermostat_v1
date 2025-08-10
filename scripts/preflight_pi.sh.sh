#!/usr/bin/env bash
set -euo pipefail

echo "[Preflight] Checking I2C bus"
if ! command -v i2cdetect >/dev/null 2>&1; then
  echo "i2c-tools not installed. Run scripts/setup_pi.sh" && exit 1
fi
sudo i2cdetect -y 1 || true
echo "Expected to see device 0x38 for AHT20"

echo "[Preflight] Python import smoke test"
source .venv/bin/activate
python - <<'PY'
import sys
print("Python:", sys.version)
try:
    import RPi.GPIO as GPIO
    print("GPIO backend:", GPIO.__name__)
except Exception as e:
    print("GPIO import error:", e)
try:
    import adafruit_ahtx0, adafruit_character_lcd.character_lcd as lcd
    print("Adafruit libs OK")
except Exception as e:
    print("Adafruit import issue:", e)
PY
