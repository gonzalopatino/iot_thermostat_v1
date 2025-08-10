# test/test_aht20_demo.py
import sys, os, time
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from src.aht20_sensor import AHT20Sensor

def main():
    sensor = AHT20Sensor(settle_s=0.2)

    r1 = sensor.read()
    ts1 = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r1.timestamp))
    print(f"[{ts1}] Single: {r1.temperature_c:.2f} °C, {r1.humidity_rh:.1f} %RH")

    r_avg = sensor.read_avg(samples=10, interval_s=0.1)
    ts_avg = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r_avg.timestamp))
    print(f"[{ts_avg}] Average: {r_avg.temperature_c:.2f} °C, {r_avg.humidity_rh:.1f} %RH")

if __name__ == "__main__":
    main()
