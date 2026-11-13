import builtins
import hashlib
import json

import pytest

from slam_robot_wifi_scanner import walk_survey
from slam_robot_wifi_scanner.beacon import Beacon
from slam_robot_wifi_scanner.capture import Collector
from slam_robot_wifi_scanner.session import SessionWriter


def b(rssi, ch=11, tuned=11, ssid="Yumi 2.4G", bssid="84:3c:99:58:8d:5f"):
    return Beacon(1700000000.123, bssid, ssid, ch, rssi, None, 12.3, tuned, "WPA2")


def test_session_files_match_the_contract(tmp_path):
    w = SessionWriter(tmp_path, "001", "1", 0.5)
    w.add_waypoint(1, 1.55, 0.5)
    w.add_waypoint(2, 1.55, 1.0)
    w.add_beacons(1, 1.55, 0.5, 90.0, [b(-40), b(-41, tuned=6)])   # off-channel frame is kept raw
    w.add_beacons(2, 1.55, 1.0, 90.0, [b(-45)])
    note = w.finish()
    recs = json.loads((tmp_path / "records.json").read_text())
    assert note["status"] == "complete" and note["record_count"] == 3 == len(recs)
    assert note["records_sha256"] == hashlib.sha256((tmp_path / "records.json").read_bytes()).hexdigest()
    assert [r["seq"] for r in recs] == [1, 2, 3]
    r = recs[0]
    assert set(r) == {"session_id", "floor", "seq", "timestamp", "waypoint_id", "x", "y", "heading",
                      "bssid", "ssid", "channel", "tuned_channel", "rssi", "noise_floor",
                      "chan_util", "encryption"}
    assert r["bssid"] == "84:3C:99:58:8D:5F" and r["noise_floor"] is None and r["encryption"] == "WPA2"
    assert recs[1]["tuned_channel"] == 6                     # nothing filtered on the Pi
    wps = json.loads((tmp_path / "waypoints.json").read_text())
    assert wps["waypoints"] == [{"waypoint_id": 1, "x": 1.55, "y": 0.5},
                                {"waypoint_id": 2, "x": 1.55, "y": 1.0}]
    assert wps["spacing_m"] == 0.5


def test_interrupted_session_keeps_what_was_recorded(tmp_path):
    w = SessionWriter(tmp_path, "002", "1")
    w.add_beacons(1, 0, 0, 90, [b(-40)])
    assert w.finish("interrupted")["status"] == "interrupted"
    assert len(json.loads((tmp_path / "records.json").read_text())) == 1
    with pytest.raises(ValueError):
        SessionWriter(tmp_path / "x", "003", "1").finish("nope")


def test_line_points_and_labels():
    assert walk_survey.line_points(1.55, 0.5, "y", 0.5, 5) == [
        (1.55, 0.5), (1.55, 1.0), (1.55, 1.5), (1.55, 2.0), (1.55, 2.5)]
    assert walk_survey.line_points(0.5, 1.0, "x", 0.5, 2) == [(0.5, 1.0), (1.0, 1.0)]
    assert walk_survey.parse_points("1.0,1.0 2.0,1.5") == [(1.0, 1.0), (2.0, 1.5)]
    assert walk_survey.label(7, 5) == "WP-07" and walk_survey.label(7, 795) == "WP-007"


class FakeScanner:
    """Stands in for the adapter: each scan returns 3 beacons."""
    def __init__(self, *a, **k):
        self.collector = Collector()
    def start(self): pass
    def stop(self): pass
    def scan(self, dwell_s):
        return [b(-40), b(-41), b(-60, ch=6, tuned=6, ssid="Other", bssid="aa:bb:cc:dd:ee:ff")]


def _run(monkeypatch, tmp_path, keys, extra=()):
    monkeypatch.setattr(walk_survey, "LiveScanner", FakeScanner)
    monkeypatch.setattr(walk_survey.time, "sleep", lambda s: None)
    it = iter(keys)

    def fake_input(prompt=""):
        k = next(it)
        if k is KeyboardInterrupt:
            raise KeyboardInterrupt
        return k
    monkeypatch.setattr(builtins, "input", fake_input)
    walk_survey.main(["--session", "001", "--line", "1.55", "0.5", "--count", "3",
                      "--dwell", "1", "--out", str(tmp_path), "--target", "Yumi 2.4G", *extra])
    return json.loads((tmp_path / "notification.json").read_text())


def test_full_walk_completes(monkeypatch, tmp_path, capsys):
    note = _run(monkeypatch, tmp_path, ["", "", ""])
    assert note["status"] == "complete" and note["record_count"] == 9
    recs = json.loads((tmp_path / "records.json").read_text())
    assert {r["waypoint_id"] for r in recs} == {1, 2, 3}
    assert recs[0]["heading"] == 90.0 and recs[0]["x"] == 1.55
    assert "<- target" in capsys.readouterr().out


def test_ctrl_c_marks_session_interrupted(monkeypatch, tmp_path):
    note = _run(monkeypatch, tmp_path, ["", KeyboardInterrupt])
    assert note["status"] == "interrupted" and note["record_count"] == 3


def test_refuses_wlan0(tmp_path):
    with pytest.raises(SystemExit):
        walk_survey.main(["--session", "1", "--line", "0", "0", "--iface", "wlan0", "--out", str(tmp_path)])
