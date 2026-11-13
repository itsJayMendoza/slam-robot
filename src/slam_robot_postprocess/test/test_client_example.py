"""The client's own worked example (T1 post-processing PDF) must come out exactly as printed."""
import json

import pytest

from slam_robot_postprocess.config import Config
from slam_robot_postprocess.pipeline import process
from conftest import A, B, G

# The example uses minimums of 2 (IDW) and 5 (Kriging) "chosen only so the example runs".
EXAMPLE_CFG = dict(idw_min_points=2, kriging_min_points=5)


@pytest.fixture
def result(example):
    session, map_yaml, store = example
    summary, out = process(session, store, map_yaml, Config.load(**EXAMPLE_CFG))
    load = lambda name: json.loads((out / name).read_text())
    return summary, load


def test_step6_accepts_21_of_23_and_rejects_the_two_placeholders(result):
    summary, load = result
    assert summary["records"] == 23 and summary["accepted"] == 21
    assert summary["rejected_by_reason"]["invalid_value"] == 2
    assert sum(summary["rejected_by_reason"].values()) == 2
    rej = load("cleaning.json")["rejected_records"]
    assert sorted(r["rssi"] for r in rej) == [-127, 0]
    assert all(r["bssid"] == "11:22:33:44:55:66" for r in rej)


def test_step7_medians_and_reading_counts(result):
    _, load = result
    merged = {(m["waypoint_id"], m["bssid"]): m for m in load("merged.json")["merged"]}
    expected = {(7, A): (-55.5, 4), (7, B): (-68, 3), (7, G): (-79, 3),
                (9, A): (-71, 4), (9, B): (-58, 4), (9, G): (-64, 3)}
    assert set(merged) == set(expected)                 # IoT-DEVICES has no row left
    for key, (rssi, n) in expected.items():
        assert merged[key]["rssi"] == rssi and merged[key]["readings"] == n


def test_step7_measured_positions(result):
    _, load = result
    pos = load("merged.json")["positions"]
    assert (pos["7"]["x"], pos["7"]["y"]) == (pytest.approx(2.02), pytest.approx(1.80))
    assert (pos["9"]["x"], pos["9"]["y"]) == (pytest.approx(4.01), pytest.approx(1.81))
    assert pos["7"]["offset_m"] == pytest.approx(0.02, abs=1e-6)
    assert "8" not in pos                               # WP-08 has no scans


def test_step8_points_per_network(result):
    _, load = result
    sets = load("sets.json")
    assert list(sets) == ["CORP-WIFI", "GUEST-NET"]
    corp = {p["waypoint_id"]: (p["rssi"], p["best_bssid"]) for p in sets["CORP-WIFI"]}
    assert corp == {7: (-55.5, A), 9: (-58, B)}
    guest = {p["waypoint_id"]: p["rssi"] for p in sets["GUEST-NET"]}
    assert guest == {7: -79, 9: -64}


def test_phase_c_grid_layout(result):
    summary, load = result
    grid = load("grid.json")
    assert (grid["cells_across"], grid["cells_up"]) == (80, 60)
    assert (grid["origin_x"], grid["origin_y"], grid["cell_size_m"]) == (-10.0, -7.5, 0.25)
    assert summary["waypoints_outside_grid"] == []


def test_phase_d_idw_at_cell_52_37(result):
    _, load = result
    idx = 37 * 80 + 52
    assert idx == 3012
    corp, guest, all_ = load("IDW_CORP-WIFI.json"), load("IDW_GUEST-NET.json"), load("IDW_ALL.json")
    assert corp["values"][idx] == pytest.approx(-57.0, abs=0.05)      # document: -57.0
    assert guest["values"][idx] == pytest.approx(-69.9, abs=0.05)     # document: -69.9
    assert all_["networks"] == ["CORP-WIFI", "GUEST-NET"]
    assert all_["values"][idx] == corp["values"][idx]
    assert all_["strongest"][idx] == 0                                 # CORP-WIFI
    assert len(all_["values"]) == 4800


def test_phase_d_kriging_skipped_with_not_enough_points(result):
    summary, load = result
    assert summary["methods"]["Kriging"] == {"CORP-WIFI": "not enough points (2 of 5)",
                                             "GUEST-NET": "not enough points (2 of 5)"}
    with pytest.raises(FileNotFoundError):
        load("Kriging_ALL.json")


def test_cell_on_a_waypoint_takes_its_value(example):
    """IDW is exact at a sample: a cell centred on a point returns that point's value."""
    from slam_robot_postprocess.interpolate import run_idw
    import numpy as np
    pts = [{"x": 0.125, "y": 0.125, "rssi": -50.0}, {"x": 2.0, "y": 2.0, "rssi": -70.0}]
    r = run_idw(pts, np.array([[0.125, 0.125]]), Config.load(idw_min_points=2))
    assert r["values"][0] == -50.0
