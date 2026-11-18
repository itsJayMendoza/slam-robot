"""imu_check: test the MPU6050 on the Pi without ROS.

    imu_check                # chip id, gravity, gyro bias and noise (keep the board still)
    imu_check --live         # then print the turn rate and heading; rotate the robot by hand
"""
import argparse
import math
import sys
import time

from .calibration import NotStill, apply, calibrate, collect
from .mpu6050 import KNOWN_IDS, MPU6050, explain_io_error, open_bus


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--address", type=lambda s: int(s, 0), default=0x68)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--gyro-range", type=int, default=250, choices=[250, 500, 1000, 2000])
    ap.add_argument("--seconds", type=float, default=3.0, help="stationary measurement length")
    ap.add_argument("--live", action="store_true", help="afterwards, print turn rate and heading")
    a = ap.parse_args(argv)
    try:
        dev = MPU6050(open_bus(a.bus), a.address, a.gyro_range, rate_hz=a.rate)
        chip = dev.who_am_i()
        dev.configure()
    except RuntimeError as e:
        sys.exit(str(e))
    except OSError as e:
        sys.exit(explain_io_error(e, a.address))
    print(f"chip: WHO_AM_I 0x{chip:02x} ({KNOWN_IDS.get(chip, 'unknown, may still work')})")
    print(f"settings: gyro +/-{a.gyro_range} deg/s, accel +/-2 g, filter 44 Hz, "
          f"{dev.chip_rate_hz:.0f} Hz")

    n = max(20, int(a.seconds * a.rate))
    print(f"keep it still: measuring {n} samples ...")
    t0 = time.monotonic()
    samples = collect(dev.read, n, 1.0 / a.rate, time.sleep)
    print(f"read rate: {n / (time.monotonic() - t0):.0f} Hz")
    try:
        cal = calibrate(samples)
    except NotStill as e:
        sys.exit(f"FAIL: {e}")
    deg = math.degrees
    print(f"accel mean (m/s^2): x {cal.accel_mean[0]:+.2f}  y {cal.accel_mean[1]:+.2f}  "
          f"z {cal.accel_mean[2]:+.2f}   |g| {cal.gravity:.2f} (expect 9.81)")
    print("gyro bias (deg/s):  " + "  ".join(f"{ax} {deg(b):+.3f}"
                                             for ax, b in zip("xyz", cal.gyro_bias)))
    print("gyro noise (deg/s): " + "  ".join(f"{ax} {deg(math.sqrt(v)):.3f}"
                                             for ax, v in zip("xyz", cal.gyro_var)))
    print(f"temperature: {samples[-1].temp_c:.1f} C")
    print("PASS")
    if not a.live:
        return

    print("\nlive: rotate the robot counterclockwise seen from above; z rate should go positive "
          "if the chip's Z axis points up. Ctrl+C to stop.")
    heading, last, shown = 0.0, time.monotonic(), 0.0
    try:
        while True:
            s = apply(dev.read(), cal)
            now = time.monotonic()
            heading += s.gyro[2] * (now - last)
            last = now
            if now - shown >= 0.1:
                shown = now
                print(f"\rz rate {deg(s.gyro[2]):+7.1f} deg/s   heading {deg(heading):+8.1f} deg ",
                      end="", flush=True)
            time.sleep(1.0 / a.rate)
    except KeyboardInterrupt:
        print()
