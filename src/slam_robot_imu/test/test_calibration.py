import math
import random

import pytest

from slam_robot_imu.calibration import (VAR_FLOOR, NotStill, apply, axis_stats, calibrate,
                                        collect)
from slam_robot_imu.mpu6050 import G, Sample


def still(n=200, bias=(0.01, -0.02, 0.005), noise=0.003, seed=1):
    rng = random.Random(seed)
    return [Sample(accel=(rng.gauss(0, 0.05), rng.gauss(0, 0.05), G + rng.gauss(0, 0.05)),
                   gyro=tuple(b + rng.gauss(0, noise) for b in bias), temp_c=30.0)
            for _ in range(n)]


def test_bias_and_noise_estimated():
    cal = calibrate(still())
    assert cal.gyro_bias == pytest.approx((0.01, -0.02, 0.005), abs=0.001)
    assert all(math.sqrt(v) == pytest.approx(0.003, rel=0.2) for v in cal.gyro_var)
    assert cal.gravity == pytest.approx(G, abs=0.05)


def test_apply_removes_bias_only_from_gyro():
    cal = calibrate(still())
    s = Sample(accel=(1.0, 2.0, 3.0), gyro=(0.01, -0.02, 0.105), temp_c=25.0)
    out = apply(s, cal)
    assert out.accel == s.accel
    assert out.gyro[2] == pytest.approx(0.1, abs=0.001)


def test_motion_rejected():
    samples = still()
    for i, s in enumerate(samples):                 # robot turning during "calibration"
        samples[i] = Sample(s.accel, (s.gyro[0], s.gyro[1], 0.5 * math.sin(i / 10)), s.temp_c)
    with pytest.raises(NotStill, match="moving"):
        calibrate(samples)


def test_wrong_gravity_rejected():
    samples = [Sample((0.0, 0.0, G / 8), s.gyro, s.temp_c) for s in still()]
    with pytest.raises(NotStill, match="magnitude"):
        calibrate(samples)


def test_too_few_samples():
    with pytest.raises(NotStill):
        calibrate(still(5))


def test_covariance_never_zero():
    s = Sample((0.0, 0.0, G), (0.0, 0.0, 0.0), 25.0)
    cal = calibrate([s] * 50)                       # perfectly constant readings
    assert min(cal.gyro_var + cal.accel_var) == VAR_FLOOR


def test_collect_keeps_schedule_despite_slow_reads():
    t = [0.0]
    slept = []

    def read():
        t[0] += 0.004                      # each read takes 4 ms
        return round(t[0], 3)

    def sleep(d):
        slept.append(round(d, 6))
        t[0] += d

    out = collect(read, 3, 0.01, sleep, lambda: t[0])
    assert len(out) == 3
    assert slept == [0.006, 0.006, 0.006]  # 10 ms period, not 10 ms + read time
    assert t[0] == pytest.approx(0.03)


def test_axis_stats_separates_drift_from_noise():
    samples = [Sample((0.0, 0.0, G), (0.0, 0.0, i * 0.001), 25.0) for i in range(90)]
    z = axis_stats(samples, "gyro")[2]
    assert z.drift == pytest.approx(0.06, abs=0.001)     # steady ramp: large drift
    assert z.peak_to_peak == pytest.approx(0.089)
    x = axis_stats(samples, "gyro")[0]
    assert (x.std, x.drift) == (0.0, 0.0)
