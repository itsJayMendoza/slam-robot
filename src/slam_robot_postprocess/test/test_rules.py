import json

import numpy as np
import pytest

from slam_robot_postprocess.clean import clean
from slam_robot_postprocess.config import Config
from slam_robot_postprocess.grid import MapError, build_layout, load_or_build_layout, read_map
from slam_robot_postprocess.ingest import SessionError, check_session
from slam_robot_postprocess.interpolate import run_kriging
from slam_robot_postprocess.merge import group_by_ssid, merge
from slam_robot_postprocess.pipeline import process
from conftest import EXAMPLE_WAYPOINTS, example_records, write_map, write_session


def rec(rssi, wid=7, bssid="AA:BB:CC:11:22:33", ssid="CORP-WIFI", ch=6, tuned=None, seq=1):
    return {"seq": seq, "timestamp": 1.0, "waypoint_id": wid, "x": 0, "y": 0, "bssid": bssid,
            "ssid": ssid, "channel": ch, "tuned_channel": tuned, "rssi": rssi}


def test_document_outlier_example_rejects_minus_75():
    rs = [rec(v, seq=i) for i, v in enumerate([-56, -55, -57, -54, -75])]
    out = clean(rs, [7], Config())
    assert out.counts["outlier"] == 1 and out.rejected[0]["rssi"] == -75


def test_spread_within_10_db_is_never_called_an_outlier():
    rs = [rec(v, seq=i) for i, v in enumerate([-79, -81, -70])]
    assert clean(rs, [7], Config()).counts["outlier"] == 0


def test_incomplete_records_and_empty_ssid():
    rs = [rec(-60, wid=None), rec(-62, bssid=""), rec(None), rec(-60, ssid="")]
    out = clean(rs, [7], Config())
    assert out.counts["incomplete"] == 3
    assert len(out.accepted) == 1                     # hidden network is NOT incomplete


def test_unknown_waypoint_is_flagged_not_silently_dropped():
    out = clean([rec(-60, wid=25)], [7], Config())
    assert out.counts["unknown_waypoint"] == 1 and out.flagged_waypoints == [25]


def test_adapter_placeholder_and_range():
    out = clean([rec(-239), rec(12), rec(-58), rec(-88), rec(-125)], [7], Config())
    assert out.counts["invalid_value"] == 3
    assert sorted(float(r["rssi"]) for r in out.accepted) == [-88, -58]


def test_weak_but_real_readings_are_kept_and_counted():
    out = clean([rec(-105), rec(-60)], [7], Config())
    assert len(out.accepted) == 2 and out.near_sensitivity == 1


def test_off_channel_frames_rejected():
    out = clean([rec(-50, tuned=11, ch=11), rec(-58, tuned=9, ch=11)], [7], Config())
    assert out.counts["off_channel"] == 1 and out.accepted[0]["rssi"] == -50
    keep = clean([rec(-58, tuned=9, ch=11)], [7], Config.load(reject_off_channel=False))
    assert len(keep.accepted) == 1


def test_hidden_networks_stay_separate_per_bssid():
    m = merge([rec(-60, ssid="", bssid="AA:AA:AA:AA:AA:01"), rec(-70, ssid="", bssid="AA:AA:AA:AA:AA:02")])
    sets = group_by_ssid(m, {7: {"x": 0, "y": 0}})
    assert sorted(sets) == ["<hidden> AA:AA:AA:AA:AA:01", "<hidden> AA:AA:AA:AA:AA:02"]


def test_target_ssids_limit_the_sets():
    m = merge([rec(-60, ssid="A", bssid="01"), rec(-70, ssid="B", bssid="02")])
    assert list(group_by_ssid(m, {7: {"x": 0, "y": 0}}, ["B"])) == ["B"]


def test_mean_merge_option():
    m = merge([rec(-60, seq=1), rec(-70, seq=2), rec(-71, seq=3)], how="mean")
    assert m[0]["rssi"] == pytest.approx(-67.0) and m[0]["readings"] == 3


def test_interrupted_session_waits_for_a_decision(tmp_path):
    s = write_session(tmp_path / "s", example_records(), EXAMPLE_WAYPOINTS, status="interrupted")
    with pytest.raises(SessionError, match="awaiting decision"):
        check_session(s)
    assert check_session(s, accept_interrupted=True)[0]["status"] == "interrupted"


def test_count_and_checksum_mismatch_ask_for_a_retry(tmp_path):
    s = write_session(tmp_path / "a", example_records(), EXAMPLE_WAYPOINTS, count=847)
    with pytest.raises(SessionError, match="count mismatch"):
        check_session(s)
    s = write_session(tmp_path / "b", example_records(), EXAMPLE_WAYPOINTS, sha="0" * 64)
    with pytest.raises(SessionError, match="checksum"):
        check_session(s)


def test_missing_map_stops_with_a_clear_message(tmp_path):
    s = write_session(tmp_path / "s", example_records(), EXAMPLE_WAYPOINTS)
    with pytest.raises(SessionError, match="no map for floor 1"):
        process(s, tmp_path / "store")


def test_rotated_map_origin_is_refused(tmp_path):
    y = write_map(tmp_path, origin=(-10.0, -7.5, 0.5))
    with pytest.raises(MapError, match="rotation"):
        read_map(y)


def test_cells_round_up_to_cover_the_floor(tmp_path):
    info = read_map(write_map(tmp_path, width_px=62, height_px=104))   # 3.10 m x 5.20 m
    lay = build_layout(info, "1", 0.25)
    assert (lay["cells_across"], lay["cells_up"]) == (13, 21)          # 12.4 -> 13, 20.8 -> 21


def test_grid_is_reused_until_the_map_or_cell_size_changes(tmp_path):
    y = write_map(tmp_path)
    assert load_or_build_layout(y, "1", 0.25)[1] is False
    assert load_or_build_layout(y, "1", 0.25)[1] is True
    assert load_or_build_layout(y, "1", 0.5)[1] is False
    (tmp_path / "map.pgm").write_bytes((tmp_path / "map.pgm").read_bytes()[:-1] + b"\x00")
    assert load_or_build_layout(y, "1", 0.25)[1] is False              # map changed


def test_kriging_runs_and_returns_variance_with_enough_points():
    rng = np.random.default_rng(0)
    pts = [{"x": float(x), "y": float(y), "rssi": float(-40 - 6 * np.hypot(x - 1, y - 1) + rng.normal(0, 1))}
           for x in range(6) for y in range(5)]
    q = np.array([[2.5, 2.5], [10.0, 10.0]])
    r = run_kriging(pts, q, Config())
    assert r["status"] == "ok" and r["variance"][1] > r["variance"][0]


def test_raw_copy_is_never_overwritten_by_a_different_one(example):
    session, map_yaml, store = example
    process(session, store, map_yaml)
    recs = example_records()
    recs[0]["rssi"] = -1
    write_session(session, recs, EXAMPLE_WAYPOINTS)
    with pytest.raises(SessionError, match="different copy"):
        process(session, store, map_yaml)
