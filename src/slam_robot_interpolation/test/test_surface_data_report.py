import csv

import numpy as np
import pytest

from slam_robot_interpolation.data import read_waypoint_avg, samples
from slam_robot_interpolation.report import coverage_fraction, find_gaps, per_ap_summary
from slam_robot_interpolation.surface import Surface, free_space_mask, interpolate_surface


def test_free_space_mask_follows_floor_plan(room_occ):
    x, y, mask = free_space_mask(room_occ, 0.1)
    gx, gy = np.meshgrid(x, y)
    assert mask[(np.abs(gx) < 3.4) & (np.abs(gy) < 3.4)].all()
    assert not mask[(np.abs(gx) > 3.7) | (np.abs(gy) > 3.7)].any()


@pytest.mark.parametrize("method", ["idw", "kriging"])
def test_surface_nan_outside_free_space(lattice, room_occ, method):
    xy, z = lattice
    s = interpolate_surface(method, xy, z, room_occ, 0.25)
    _, _, mask = free_space_mask(room_occ, 0.25)
    assert np.isfinite(s.values[mask]).all()
    assert np.isnan(s.values[~mask]).all()
    assert (s.variance is not None) == (method == "kriging")
    if method == "kriging":
        assert np.isfinite(s.variance[mask]).all() and np.isnan(s.variance[~mask]).all()


def test_surface_close_to_truth(lattice, room_occ):
    xy, z = lattice
    s = interpolate_surface("kriging", xy, z, room_occ, 0.25)
    gx, gy = np.meshgrid(s.x, s.y)
    inside = np.isfinite(s.values) & (np.abs(gx) <= 3) & (np.abs(gy) <= 3)
    from conftest import field
    t = field(np.column_stack([gx[inside], gy[inside]]))
    assert np.sqrt(np.mean((s.values[inside] - t) ** 2)) < 2.0


HEADER = ["run_id", "waypoint_id", "x", "y", "yaw", "bssid", "ssid", "channel", "n_samples",
          "rssi_mean", "rssi_std", "noise_mean", "noise_std", "snr_mean", "chan_util_mean",
          "dwell_s", "spacing_m"]


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=HEADER)
        w.writeheader()
        for r in rows:
            w.writerow({**{h: "" for h in HEADER}, **r})


@pytest.fixture
def rows(tmp_path):
    p = tmp_path / "waypoint_avg.csv"
    write_csv(p, [
        dict(waypoint_id=1, x=0, y=0, bssid="AA", ssid="Corp", channel=1, rssi_mean=-50, snr_mean=30),
        dict(waypoint_id=1, x=0, y=0, bssid="BB", ssid="Guest", channel=6, rssi_mean=-60),
        dict(waypoint_id=2, x=1, y=0, bssid="AA", ssid="Corp", channel=1, rssi_mean=-70, snr_mean=15),
        dict(waypoint_id=2, x=1, y=0, bssid="BB", ssid="Guest", channel=6, rssi_mean=-65),
        dict(waypoint_id=3, x=2, y=0, bssid="AA", ssid="Corp", channel=1, rssi_mean=-80),
    ])
    return read_waypoint_avg(p)


def test_samples_filters_by_bssid_and_ssid(rows):
    xy, z, ids = samples(rows, bssid="aa")
    assert z.tolist() == [-50, -70, -80] and ids == [1, 2, 3]
    assert samples(rows, ssid="Guest")[1].tolist() == [-60, -65]


def test_samples_strongest_ap_per_waypoint(rows):
    _, z, ids = samples(rows, strongest=True)
    assert z.tolist() == [-50, -65, -80] and ids == [1, 2, 3]


def test_missing_snr_is_dropped_not_invented(rows):
    _, z, ids = samples(rows, "snr_mean", bssid="AA")
    assert z.tolist() == [30, 15] and ids == [1, 2]       # waypoint 3 has no SNR
    assert samples(rows, "snr_mean", bssid="BB")[1].size == 0


def test_per_ap_summary_pass_fail(rows):
    s = {r["bssid"]: r for r in per_ap_summary(rows)}
    assert s["aa"]["n_waypoints"] == 3
    assert s["aa"]["rssi_mean"] == pytest.approx(-200 / 3)
    assert s["aa"]["rssi_pass"] is True                  # -66.7 dBm is at or above -67
    assert {r["bssid"]: r["rssi_pass"] for r in per_ap_summary(rows, rssi_threshold=-65)}["aa"] is False
    assert (s["aa"]["rssi_min"], s["aa"]["rssi_max"]) == (-80, -50)
    assert s["aa"]["snr_mean"] == pytest.approx(22.5) and s["aa"]["snr_pass"] is True
    assert s["bb"]["snr_mean"] is None and s["bb"]["snr_pass"] is None   # never guessed
    assert [r["bssid"] for r in per_ap_summary(rows)] == ["bb", "aa"]   # strongest mean first


def _surface(values):
    v = np.array(values, float)
    return Surface(np.arange(v.shape[1]) * 0.5, np.arange(v.shape[0]) * 0.5, v, None, 0.5)


def test_find_gaps_area_location_and_mean():
    s = _surface([[-50, -50, -50, -50],
                  [-50, -80, -80, -50],
                  [-50, -80, np.nan, -50],
                  [-50, -50, -50, -75]])
    gaps = find_gaps(s, -67.0)
    assert [round(g["area_m2"], 3) for g in gaps] == [0.75, 0.25]   # 3 cells, then 1 cell
    big = gaps[0]
    assert big["mean_value"] == pytest.approx(-80) and big["worst_value"] == -80
    assert find_gaps(s, -67.0, min_area_m2=0.5) == [gaps[0]]


def test_coverage_fraction():
    s = _surface([[-50, -60], [-70, np.nan]])
    assert coverage_fraction(s, -67.0) == pytest.approx(2 / 3)
