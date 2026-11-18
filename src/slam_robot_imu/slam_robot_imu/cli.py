"""imu_check: test the MPU6050 on the Pi without ROS.

    imu_check                # chip id, gravity, gyro bias and noise (keep the board still)
    imu_check --live         # then print the turn rate and heading; rotate the robot by hand
"""
import argparse
import math
import sys
import time

from .calibration import NotStill, apply, axis_stats, calibrate, collect
from .mpu6050 import KNOWN_IDS, MPU6050, explain_io_error, open_bus


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--bus", type=int, default=1)
    ap.add_argument("--address", type=lambda s: int(s, 0), default=0x68)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--gyro-range", type=int, default=250, choices=[250, 500, 1000, 2000])
    ap.add_argument("--seconds", type=float, default=3.0, help="stationary measurement length")
    ap.add_argument("--max-std", type=float, default=0.02,
                    help="pass limit for the gyro std, rad/s (default 0.02 = 1.15 deg/s)")
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
    samples = collect(dev.read, n, 1.0 / a.rate, time.sleep, time.monotonic)
    print(f"read rate: {n / (time.monotonic() - t0):.0f} Hz, temperature {samples[-1].temp_c:.1f} C")

    deg = math.degrees
    print("\n             mean      std   peak-to-peak   drift (last third - first third)")
    for name, which, conv, unit in [("gyro", "gyro", deg, "deg/s"), ("accel", "accel", float, "m/s^2")]:
        for ax, st in zip("xyz", axis_stats(samples, which)):
            print(f"{name} {ax}  {conv(st.mean):+8.3f} {conv(st.std):8.3f} {conv(st.peak_to_peak):10.3f}"
                  f" {conv(st.drift):+12.3f}   {unit}")
    g = math.sqrt(sum(st.mean ** 2 for st in axis_stats(samples, "accel")))
    print(f"|g| {g:.2f} m/s^2 (expect 9.81)\n")
    try:
        cal = calibrate(samples, a.max_std)
    except NotStill as e:
        print(f"FAIL: {e}")
        worst = max(axis_stats(samples, "gyro"), key=lambda st: st.std)
        if abs(worst.drift) > 2 * worst.std:
            print("hint: the gyro is drifting steadily (warming up, or the board slowly tilting); "
                  "run again after a minute")
        else:
            print("hint: noise or vibration; place the board flat on a solid surface, do not touch "
                  "the wires, and run again. If it stays this high the chip itself is noisy")
        sys.exit(1)
    print("PASS (gyro std below "
          f"{deg(a.max_std):.2f} deg/s on every axis; this bias is what imu_node will subtract)")
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
