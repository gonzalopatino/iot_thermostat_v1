#!/usr/bin/env bash
set -euo pipefail

echo "[1/7] Update apt"
sudo apt-get update

echo "[2/7] System packages for GPIO/I2C and building wheels"
sudo apt-get install -y \
  swig build-essential python3-dev python3-venv python3-pip \
  liblgpio-dev python3-lgpio i2c-tools

echo "[3/7] Enable I2C (Pi OS). Safe no-op elsewhere."
if command -v raspi-config >/dev/null 2>&1; then
  sudo raspi-config nonint do_i2c 0 || true
fi

echo "[4/7] Add user to gpio and i2c groups (re-log to take effect)"
sudo usermod -aG gpio,i2c "$USER" || true

echo "[5/7] Create venv"
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip

echo "[6/7] Install Python dependencies"
pip install -r requirements-pi.txt

echo "[7/7] Quick note"
echo "Done. If this is the first time enabling I2C or changing groups, reboot the Pi."
echo "Then activate with: source .venv/bin/activate"
echo "Run with: python3 main.py"
