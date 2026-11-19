"""Stationary calibration: gyro bias and the noise used as the message covariances."""
import math
import statistics
from dataclasses import dataclass

from .mpu6050 import G, Sample

VAR_FLOOR = 1e-6          # a covariance of exactly 0 would tell the EKF the sensor is perfect
GRAVITY_TOLERANCE = 0.25  # within 25% of g: this board reads |g| about 11.2 (+14%); accel is not used by the EKF


class NotStill(Exception):
    """The samples show motion (or no sensible gravity), so no bias can be taken from them."""


@dataclass
class Calibration:
    gyro_bias: tuple      # rad/s
    gyro_var: tuple       # (rad/s)^2
    accel_mean: tuple     # m/s^2 (includes gravity)
    accel_var: tuple      # (m/s^2)^2
    n: int

    @property
    def gravity(self):
        return math.sqrt(sum(a * a for a in self.accel_mean))


def calibrate(samples, max_gyro_std=0.02, min_samples=20):
    samples = list(samples)
    if len(samples) < min_samples:
        raise NotStill(f"only {len(samples)} samples, need {min_samples}")
    gyro = list(zip(*(s.gyro for s in samples)))
    accel = list(zip(*(s.accel for s in samples)))
    gyro_std = [statistics.pstdev(axis) for axis in gyro]
    if max(gyro_std) > max_gyro_std:
        raise NotStill(f"gyro std {max(gyro_std):.4f} rad/s > {max_gyro_std} (robot moving?)")
    cal = Calibration(gyro_bias=tuple(statistics.fmean(a) for a in gyro),
                      gyro_var=tuple(max(s * s, VAR_FLOOR) for s in gyro_std),
                      accel_mean=tuple(statistics.fmean(a) for a in accel),
                      accel_var=tuple(max(statistics.pvariance(a), VAR_FLOOR) for a in accel),
                      n=len(samples))
    if abs(cal.gravity - G) > GRAVITY_TOLERANCE * G:
        raise NotStill(f"accel magnitude {cal.gravity:.2f} m/s^2, expected about {G:.2f} "
                       "(wrong range setting or a bad sensor)")
    return cal


def apply(sample, cal):
    """Remove the gyro bias. Accel is left as measured: the EKF expects gravity in it."""
    return Sample(accel=sample.accel,
                  gyro=tuple(g - b for g, b in zip(sample.gyro, cal.gyro_bias)),
                  temp_c=sample.temp_c)


def collect(read, n, period_s, sleep, clock):
    """n samples from read() on a fixed schedule, one every period_s (blocking; for the CLI).
    Sleeps until the next slot instead of a fixed time, so the read time does not lower the rate."""
    out = []
    next_t = clock()
    for _ in range(n):
        out.append(read())
        next_t += period_s
        sleep(max(0.0, next_t - clock()))
    return out


@dataclass
class AxisStats:
    mean: float
    std: float
    peak_to_peak: float
    drift: float          # mean of the last third minus mean of the first third


def axis_stats(samples, which):
    """Per-axis statistics of samples' 'gyro' or 'accel', for telling noise from drift or motion."""
    out = []
    for axis in zip(*(getattr(s, which) for s in samples)):
        k = max(1, len(axis) // 3)
        out.append(AxisStats(statistics.fmean(axis), statistics.pstdev(axis),
                             max(axis) - min(axis),
                             statistics.fmean(axis[-k:]) - statistics.fmean(axis[:k])))
    return out
