from scapy.layers.dot11 import Dot11, Dot11Beacon, Dot11Elt, RadioTap

from slam_robot_wifi_scanner.beacon import freq_to_channel, parse_beacon

BSSID = "84:3c:99:10:78:3a"


def make_beacon(ssid="TestNet", ds_channel=8, freq=2437, rssi=-40, noise=None, bss_load=None):
    fields = "Channel+dBm_AntSignal" + ("+dBm_AntNoise" if noise is not None else "")
    kw = dict(present=fields, ChannelFrequency=freq, ChannelFlags="2GHz", dBm_AntSignal=rssi)
    if noise is not None:
        kw["dBm_AntNoise"] = noise
    frame = (RadioTap(**kw)
             / Dot11(type=0, subtype=8, addr1="ff:ff:ff:ff:ff:ff", addr2=BSSID, addr3=BSSID)
             / Dot11Beacon(cap="ESS+privacy")
             / Dot11Elt(ID=0, info=ssid.encode()))
    if ds_channel is not None:
        frame /= Dot11Elt(ID=3, info=bytes([ds_channel]))
    if bss_load is not None:
        frame /= Dot11Elt(ID=11, info=bytes([5, 0, bss_load]))
    return RadioTap(bytes(frame))  # round-trip so we test real parsing


def test_basic_fields():
    b = parse_beacon(make_beacon(rssi=-42))
    assert b.bssid == BSSID
    assert b.ssid == "TestNet"
    assert b.rssi_dbm == -42
    assert b.noise_dbm is None
    assert b.chan_util_pct is None


def test_channel_comes_from_ds_ie_not_tuned_frequency():
    # Tuned to channel 6 (2437 MHz) but the AP says it is on channel 8.
    assert parse_beacon(make_beacon(ds_channel=8, freq=2437)).channel == 8


def test_channel_falls_back_to_radiotap_frequency():
    assert parse_beacon(make_beacon(ds_channel=None, freq=2437)).channel == 6


def test_bss_load_channel_utilization():
    b = parse_beacon(make_beacon(bss_load=128))
    assert abs(b.chan_util_pct - 128 * 100 / 255) < 1e-6


def test_noise_reported_when_present():
    assert parse_beacon(make_beacon(noise=-92)).noise_dbm == -92


def test_non_beacon_returns_none():
    assert parse_beacon(RadioTap() / Dot11(type=2, subtype=0)) is None


def test_freq_to_channel():
    assert freq_to_channel(2412) == 1
    assert freq_to_channel(2472) == 13
    assert freq_to_channel(2484) == 14
    assert freq_to_channel(5180) == 36
    assert freq_to_channel(None) is None


def test_tuned_channel_recorded_separately_from_ap_channel():
    b = parse_beacon(make_beacon(ds_channel=8, freq=2437))
    assert b.channel == 8 and b.tuned_channel == 6
