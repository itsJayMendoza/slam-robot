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
