# src/aht20_sensor.py
from __future__ import annotations
import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    import board
    import busio
    import adafruit_ahtx0
except ImportError as e:
    # Helpful message if the driver is missing
    raise ImportError(
        "Missing dependency. Install with: pip install adafruit-circuitpython-ahtx0"
    ) from e


@dataclass(frozen=True)
class AHT20Reading:
    temperature_c: float
    humidity_rh: float
    timestamp: float


class AHT20InitError(RuntimeError):
    pass


class AHT20Sensor:
    """
    Simple, reusable AHT20 reader.
    - Uses I2C address 0x38 by default.
    - Provides single-shot read and averaged reads.
    """

    def __init__(
        self,
        address: int = 0x38,
        frequency: int = 100_000,
        settle_s: float = 0.0,
        i2c: Optional[busio.I2C] = None,
    ) -> None:
        """
        address: I2C address, default 0x38
        frequency: I2C clock, default 100 kHz
        settle_s: optional delay after init before first read
        i2c: pass an existing bus if you have one, else we create it
        """
        try:
            self._i2c = i2c or busio.I2C(board.SCL, board.SDA, frequency=frequency)
            # Wait until the bus is ready
            t0 = time.time()
            while not self._i2c.try_lock():
                if time.time() - t0 > 1.0:
                    raise AHT20InitError("Timed out acquiring I2C bus lock")
                time.sleep(0.01)
            self._i2c.unlock()

            self._sensor = adafruit_ahtx0.AHTx0(self._i2c, address=address)
        except Exception as ex:
            raise AHT20InitError(f"Failed to init AHT20 at 0x{address:02X}: {ex}") from ex

        if settle_s > 0:
            time.sleep(settle_s)

    def read(self) -> AHT20Reading:
        """
        Single reading.
        Returns temperature in Celsius and relative humidity in percent.
        """
        t_c = float(self._sensor.temperature)       # Celsius
        rh = float(self._sensor.relative_humidity)  # Percent
        return AHT20Reading(temperature_c=t_c, humidity_rh=rh, timestamp=time.time())

    def read_avg(self, samples: int = 5, interval_s: float = 0.2) -> AHT20Reading:
        """
        Average multiple readings to smooth noise.
        """
        assert samples >= 1
        t_sum = 0.0
        h_sum = 0.0
        ts: float = time.time()
        for i in range(samples):
            r = self.read()
            t_sum += r.temperature_c
            h_sum += r.humidity_rh
            ts = r.timestamp
            if i != samples - 1:
                time.sleep(interval_s)
        return AHT20Reading(temperature_c=t_sum / samples, humidity_rh=h_sum / samples, timestamp=ts)

    def read_f(self) -> Tuple[float, float]:
        """
        Convenience: temperature in Fahrenheit and RH in percent, single-shot.
        """
        r = self.read()
        f = r.temperature_c * 9.0 / 5.0 + 32.0
        return f, r.humidity_rh
