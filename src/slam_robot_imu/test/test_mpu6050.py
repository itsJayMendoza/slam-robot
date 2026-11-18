import math
import struct

import pytest

from slam_robot_imu import mpu6050 as m


class FakeBus:
    def __init__(self, block=None, chip=0x68):
        self.writes, self.block, self.chip = [], block or [0] * 14, chip

    def write_byte_data(self, addr, reg, val):
        self.writes.append((addr, reg, val))

    def read_byte_data(self, addr, reg):
        assert reg == m.WHO_AM_I
        return self.chip

    def read_i2c_block_data(self, addr, reg, n):
        assert (reg, n) == (m.ACCEL_XOUT_H, 14)
        return list(self.block)


def block(ax=0, ay=0, az=0, temp=0, gx=0, gy=0, gz=0):
    return list(struct.pack(">7h", ax, ay, az, temp, gx, gy, gz))


def test_int16_sign():
    assert m.int16(0x7F, 0xFF) == 32767
    assert m.int16(0x80, 0x00) == -32768       # bumperbot's "> 32768" check gets this one wrong
    assert m.int16(0xFF, 0xFF) == -1


def test_gyro_scale_matches_range():
    # 131 LSB = 1 deg/s at +/-250; 16.4 LSB = 1 deg/s at +/-2000
    s = m.convert(block(gz=131), gyro_range_dps=250)
    assert s.gyro[2] == pytest.approx(math.radians(1.0))
    s = m.convert(block(gz=164), gyro_range_dps=2000)
    assert s.gyro[2] == pytest.approx(math.radians(10.0))


def test_accel_and_temperature():
    s = m.convert(block(az=16384, ax=-8192, temp=-521), accel_range_g=2)
    assert s.accel[2] == pytest.approx(m.G)
    assert s.accel[0] == pytest.approx(-m.G / 2)
    assert s.temp_c == pytest.approx(-521 / 340 + 36.53)


def test_mpu6500_temperature_formula_selected_by_chip_id():
    dev = m.MPU6050(FakeBus(block(temp=3339), chip=0x70))
    dev.who_am_i()
    assert dev.read().temp_c == pytest.approx(3339 / 333.87 + 21.0)
    dev = m.MPU6050(FakeBus(block(temp=-521), chip=0x68))
    dev.who_am_i()
    assert dev.read().temp_c == pytest.approx(-521 / 340 + 36.53)


def test_convert_rejects_short_block():
    with pytest.raises(ValueError):
        m.convert([0] * 12)


def test_sample_divider():
    assert m.sample_divider(100, 3) == 9          # 1 kHz / 10
    assert m.sample_divider(1000, 3) == 0
    assert m.sample_divider(1, 3) == 255           # clamped
    assert m.sample_divider(100, 0) == 79          # 8 kHz with the filter off


def test_configure_writes_registers_in_order():
    bus = FakeBus()
    dev = m.MPU6050(bus, gyro_range_dps=500, accel_range_g=4, dlpf=3, rate_hz=100)
    dev.configure(sleep=lambda s: None)
    regs = [(r, v) for _, r, v in bus.writes]
    assert regs[0] == (m.PWR_MGMT_1, 0x80)
    assert regs[1] == (m.PWR_MGMT_1, 0x01)
    assert dict(regs[2:]) == {m.CONFIG: 3, m.SMPLRT_DIV: 9,
                              m.GYRO_CONFIG: 1 << 3, m.ACCEL_CONFIG: 1 << 3}
    assert dev.chip_rate_hz == pytest.approx(100.0)


def test_read_uses_configured_range():
    dev = m.MPU6050(FakeBus(block(gz=-655)), gyro_range_dps=500)
    assert dev.read().gyro[2] == pytest.approx(math.radians(-10.0))


@pytest.mark.parametrize("kw", [{"gyro_range_dps": 300}, {"accel_range_g": 3}, {"dlpf": 7},
                                {"rate_hz": 0}])
def test_bad_settings_rejected(kw):
    with pytest.raises(ValueError):
        m.MPU6050(FakeBus(), **kw)


def test_io_error_message_points_at_wiring():
    msg = m.explain_io_error(OSError(121, "Remote I/O error"), 0x68)
    assert "0x68" in msg and "i2cdetect" in msg
