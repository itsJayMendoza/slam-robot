import math

import pytest

from slam_robot_wifi_scanner.aggregate import aggregate, keep_mask
from slam_robot_wifi_scanner.beacon import Beacon


def beacon(rssi, bssid="aa:aa:aa:aa:aa:01", ssid="Net", ch=6, noise=None, util=None, t=0.0):
    return Beacon(t, bssid, ssid, ch, rssi, noise, util)


def test_stray_low_reading_is_rejected():
    # Real capture: a -100 dBm frame in the middle of a run of ~-40s.
    vals = [-40, -39, -41, -40, -100, -40, -38, -41]
    mask = keep_mask(vals)
    assert mask[4] is False
    assert all(m for i, m in enumerate(mask) if i != 4)


def test_below_receiver_floor_always_rejected():
    assert keep_mask([-112.0]) == [False]
    assert keep_mask([-97.0, -90.0]) == [False, True]


def test_small_spread_is_never_rejected():
    assert all(keep_mask([-40, -41, -42, -43, -40, -44]))


def test_average_excludes_outliers():
    readings = [beacon(r) for r in (-40, -40, -41, -39, -100)]
    stats, rejected = aggregate(readings)
    assert rejected == [False, False, False, False, True]
    s = stats[0]
    assert s.n_samples == 4 and s.n_rejected == 1
    assert math.isclose(s.rssi_mean, -40.0)
    assert s.rssi_min == -41 and s.rssi_max == -39


def test_groups_by_bssid_sorted_strongest_first():
    readings = [beacon(-80, bssid="bb"), beacon(-50, bssid="aa"), beacon(-82, bssid="bb")]
    stats, _ = aggregate(readings)
    assert [s.bssid for s in stats] == ["aa", "bb"]


def test_noise_absent_is_invalid_not_fabricated():
    s = aggregate([beacon(-40), beacon(-41)])[0][0]
    assert s.noise_valid is False
    assert math.isnan(s.noise_mean)


def test_noise_zero_is_not_a_measurement():
    s = aggregate([beacon(-40, noise=0), beacon(-41, noise=0)])[0][0]
    assert s.noise_valid is False


def test_noise_and_util_averaged_when_present():
    s = aggregate([beacon(-40, noise=-92, util=10.0), beacon(-41, noise=-90, util=20.0)])[0][0]
    assert s.noise_valid and math.isclose(s.noise_mean, -91.0)
    assert s.chan_util_valid and math.isclose(s.chan_util_mean, 15.0)


def test_util_absent_is_invalid():
    s = aggregate([beacon(-40)])[0][0]
    assert s.chan_util_valid is False and math.isnan(s.chan_util_mean)


def test_empty_input():
    assert aggregate([]) == ([], [])


def test_stats_are_floats_even_for_integer_input():
    s = aggregate([beacon(-40), beacon(-41)])[0][0]
    for v in (s.rssi_mean, s.rssi_std, s.rssi_median, s.rssi_min, s.rssi_max):
        assert isinstance(v, float)


def test_on_channel_only_drops_frames_heard_on_other_channels():
    on = [Beacon(0.0, "aa", "Net", 8, r, None, None, 8) for r in (-38, -39, -38, -40)]
    # Attenuated by 4-5 dB: inside the outlier filter's 5 dB dead band, so only the
    # on-channel filter can remove them.
    off = [Beacon(0.0, "aa", "Net", 8, -43, None, None, 3), Beacon(0.0, "aa", "Net", 8, -44, None, None, 11)]
    readings = on + off
    s_all = aggregate(readings)[0][0]
    stats, rejected = aggregate(readings, on_channel_only=True)
    s_on = stats[0]
    assert s_on.n_samples == 4 and s_on.rssi_mean == pytest.approx(-38.75)
    assert rejected == [False] * 4 + [True] * 2
    assert s_all.n_samples == 6 and s_all.rssi_std > 2 * s_on.rssi_std


def test_on_channel_only_keeps_frames_with_unknown_tuned_channel():
    r = [Beacon(0.0, "aa", "Net", 8, -40, None, None, None)] * 3
    assert aggregate(r, on_channel_only=True)[0][0].n_samples == 3


def test_ap_only_heard_off_channel_is_absent_when_filtering():
    r = [Beacon(0.0, "aa", "Net", 8, -90, None, None, 1)] * 3
    assert aggregate(r)[0][0].n_samples == 3
    assert aggregate(r, on_channel_only=True)[0] == []


def test_low_confidence_flag():
    few = aggregate([beacon(-40), beacon(-41)])[0][0]
    enough = aggregate([beacon(-40), beacon(-41), beacon(-40)])[0][0]
    assert few.low_confidence is True and enough.low_confidence is False
    assert aggregate([beacon(-40), beacon(-41)], min_samples=2)[0][0].low_confidence is False
