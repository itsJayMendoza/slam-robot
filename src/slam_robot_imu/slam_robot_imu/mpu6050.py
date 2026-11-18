"""MPU6050 register access and unit conversion. No ROS here, so it can be tested with a fake bus.

The bus object needs the smbus/smbus2 methods write_byte_data, read_byte_data and
read_i2c_block_data.
"""
import math
import time
from dataclasses import dataclass

ADDRESS = 0x68
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
ACCEL_XOUT_H = 0x3B
PWR_MGMT_1 = 0x6B
WHO_AM_I = 0x75

# WHO_AM_I values seen on MPU6050 boards; many cheap boards carry a compatible chip.
KNOWN_IDS = {0x68: "MPU6050", 0x70: "MPU6500", 0x71: "MPU9250", 0x73: "MPU9255",
             0x72: "MPU6050 clone", 0x98: "MPU6050 clone"}

GYRO_LSB_PER_DPS = {250: 131.0, 500: 65.5, 1000: 32.8, 2000: 16.4}
ACCEL_LSB_PER_G = {2: 16384.0, 4: 8192.0, 8: 4096.0, 16: 2048.0}
FS_BITS = {250: 0, 500: 1, 1000: 2, 2000: 3, 2: 0, 4: 1, 8: 2, 16: 3}
DLPF_HZ = {0: 260, 1: 184, 2: 94, 3: 44, 4: 21, 5: 10, 6: 5}
G = 9.80665


def int16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


@dataclass
class Sample:
    accel: tuple      # m/s^2, sensor axes
    gyro: tuple       # rad/s, sensor axes
    temp_c: float


def convert(block, gyro_range_dps=250, accel_range_g=2):
    """14 bytes from ACCEL_XOUT_H (accel xyz, temperature, gyro xyz; big-endian) -> Sample."""
    if len(block) != 14:
        raise ValueError(f"expected 14 bytes, got {len(block)}")
    v = [int16(block[i], block[i + 1]) for i in range(0, 14, 2)]
    a_scale = G / ACCEL_LSB_PER_G[accel_range_g]
    g_scale = math.radians(1.0) / GYRO_LSB_PER_DPS[gyro_range_dps]
    return Sample(accel=tuple(x * a_scale for x in v[0:3]),
                  gyro=tuple(x * g_scale for x in v[4:7]),
                  temp_c=v[3] / 340.0 + 36.53)


def sample_divider(rate_hz, dlpf):
    """SMPLRT_DIV for a wanted output rate. The internal rate is 1 kHz with the filter on, 8 kHz off."""
    base = 8000.0 if dlpf == 0 else 1000.0
    return max(0, min(255, round(base / rate_hz) - 1))


class MPU6050:
    def __init__(self, bus, address=ADDRESS, gyro_range_dps=250, accel_range_g=2, dlpf=3,
                 rate_hz=100.0):
        if gyro_range_dps not in GYRO_LSB_PER_DPS:
            raise ValueError(f"gyro_range_dps must be one of {sorted(GYRO_LSB_PER_DPS)}")
        if accel_range_g not in ACCEL_LSB_PER_G:
            raise ValueError(f"accel_range_g must be one of {sorted(ACCEL_LSB_PER_G)}")
        if dlpf not in DLPF_HZ:
            raise ValueError("dlpf must be 0..6")
        if rate_hz <= 0:
            raise ValueError("rate_hz must be > 0")
        self.bus, self.address = bus, address
        self.gyro_range_dps, self.accel_range_g = gyro_range_dps, accel_range_g
        self.dlpf, self.rate_hz = dlpf, rate_hz

    @property
    def chip_rate_hz(self):
        base = 8000.0 if self.dlpf == 0 else 1000.0
        return base / (1 + sample_divider(self.rate_hz, self.dlpf))

    def who_am_i(self):
        return self.bus.read_byte_data(self.address, WHO_AM_I)

    def configure(self, sleep=time.sleep):
        w = self.bus.write_byte_data
        w(self.address, PWR_MGMT_1, 0x80)      # reset
        sleep(0.1)
        w(self.address, PWR_MGMT_1, 0x01)      # wake, clock from the X gyro PLL
        sleep(0.05)
        w(self.address, CONFIG, self.dlpf)
        w(self.address, SMPLRT_DIV, sample_divider(self.rate_hz, self.dlpf))
        w(self.address, GYRO_CONFIG, FS_BITS[self.gyro_range_dps] << 3)
        w(self.address, ACCEL_CONFIG, FS_BITS[self.accel_range_g] << 3)

    def read(self):
        """One burst read of all axes, so accel and gyro come from the same instant."""
        block = self.bus.read_i2c_block_data(self.address, ACCEL_XOUT_H, 14)
        return convert(block, self.gyro_range_dps, self.accel_range_g)


def open_bus(number):
    """Open /dev/i2c-<number> with smbus2 if installed, else python3-smbus. Errors say what to fix."""
    try:
        from smbus2 import SMBus
    except ImportError:
        try:
            from smbus import SMBus
        except ImportError as e:
            raise RuntimeError("no I2C library: sudo apt install python3-smbus") from e
    try:
        return SMBus(number)
    except FileNotFoundError as e:
        raise RuntimeError(f"/dev/i2c-{number} missing: add 'dtparam=i2c_arm=on' to "
                           "/boot/firmware/config.txt and reboot") from e
    except PermissionError as e:
        raise RuntimeError(f"no permission on /dev/i2c-{number}: sudo usermod -aG i2c $USER, "
                           "then log out and in") from e


def explain_io_error(err, address):
    if getattr(err, "errno", None) in (121, 5):   # Remote I/O / I/O error: nothing answered
        return (f"no answer from 0x{address:02x}: check SDA to pin 3, SCL to pin 5, VCC to 3.3 V, "
                "GND, and run 'i2cdetect -y 1'")
    return str(err)
