import time

from slam_robot_wifi_scanner.capture import Collector, plan_channels
from test_beacon import make_beacon

ALL = list(range(1, 14))


def test_full_sweep_visits_every_channel():
    assert plan_channels(ALL, {1, 6}, full_sweep=True) == ALL


def test_adaptive_round_visits_only_active_channels_in_order():
    assert plan_channels(ALL, {11, 1, 6}, full_sweep=False) == [1, 6, 11]


def test_adaptive_falls_back_to_all_when_nothing_seen():
    assert plan_channels(ALL, set(), full_sweep=False) == ALL


def test_ignores_seen_channels_outside_the_allowed_list():
    assert plan_channels([1, 6, 11], {6, 14}, full_sweep=False) == [6]


def test_collector_tracks_ap_channels_even_when_not_recording():
    c = Collector()
    c.on_packet(make_beacon(ds_channel=8))
    assert c.channels_seen(30) == {8}
    assert c.stop() == []                       # nothing was recorded


def test_collector_forgets_stale_channels():
    c = Collector()
    c.on_packet(make_beacon(ds_channel=8))
    time.sleep(0.05)
    assert c.channels_seen(0.01) == set()


def test_collector_records_only_while_recording():
    c = Collector()
    c.on_packet(make_beacon())
    c.start()
    c.on_packet(make_beacon())
    c.on_packet(make_beacon())
    assert len(c.stop()) == 2


def test_raw_csv_has_one_row_per_beacon_with_flags(tmp_path):
    from slam_robot_wifi_scanner.aggregate import aggregate
    from slam_robot_wifi_scanner.beacon import parse_beacon
    from slam_robot_wifi_scanner.cli import write_raw_csv
    import csv
    readings = [parse_beacon(make_beacon(rssi=r)) for r in (-40, -41, -40, -41, -100)]
    _, rej = aggregate(readings)
    path = tmp_path / "raw.csv"
    write_raw_csv(path, readings, rej, rej)
    rows = list(csv.DictReader(open(path)))
    assert len(rows) == 5
    assert rows[0]["bssid"] == "84:3c:99:10:78:3a" and rows[0]["tuned_channel"] == "6"
    assert [r["rejected_all_frames"] for r in rows] == ["0", "0", "0", "0", "1"]


def _run_hopper(monkeypatch, failing, seconds=0.5, channels=(1, 6, 11, 12, 13)):
    import subprocess
    import threading
    import types
    from slam_robot_wifi_scanner.capture import ChannelHopper
    calls = []

    def fake_run(cmd, **kw):
        ch = int(cmd[-1])
        calls.append(ch)
        return types.SimpleNamespace(returncode=1 if ch in failing else 0)

    monkeypatch.setattr(subprocess, "run", fake_run)
    h = ChannelHopper("wlx", channels, dwell_s=0.01)
    h.start()
    time.sleep(seconds)
    h.stop()
    h.join(1)
    return h, calls


def test_hopper_drops_channels_the_adapter_refuses(monkeypatch):
    h, calls = _run_hopper(monkeypatch, failing={12, 13})
    assert h.unsupported == {12, 13}
    assert calls.count(12) == 2 and calls.count(13) == 2   # tried twice, then given up
    assert calls.count(1) > 5                              # the rest keep being visited


def test_hopper_keeps_working_channels_only_when_nothing_fails(monkeypatch):
    h, calls = _run_hopper(monkeypatch, failing=set())
    assert h.unsupported == set() and set(calls) == {1, 6, 11, 12, 13}


def test_hopper_with_no_tunable_channel_does_not_spin(monkeypatch):
    h, calls = _run_hopper(monkeypatch, failing={1, 6, 11, 12, 13}, seconds=0.3)
    assert len(calls) < 100
