#!/usr/bin/env bash
export DEVICE_ID="rpi-thermostat-001"
export MQTT_ENABLED=1
export MQTT_HOST="broker.hivemq.com"
export MQTT_PORT=1883
export MQTT_QOS=1
export LOG_LEVEL=DEBUG
exec python3 thermo_main.py
