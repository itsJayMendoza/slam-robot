from scapy.layers.dot11 import Dot11, Dot11Beacon, Dot11Elt, RadioTap

from slam_robot_wifi_scanner.beacon import classify_encryption, parse_beacon, rsn_akms

BSSID = "84:3c:99:10:78:3a"


def rsn_ie(*akm_types):
    body = b"\x01\x00" + b"\x00\x0f\xac\x04" + b"\x01\x00" + b"\x00\x0f\xac\x04"
    body += len(akm_types).to_bytes(2, "little") + b"".join(b"\x00\x0f\xac" + bytes([t]) for t in akm_types)
    return body + b"\x00\x00"


def beacon(cap, *ies):
    f = (RadioTap(present="Channel+dBm_AntSignal", ChannelFrequency=2437, ChannelFlags="2GHz", dBm_AntSignal=-40)
         / Dot11(type=0, subtype=8, addr1="ff:ff:ff:ff:ff:ff", addr2=BSSID, addr3=BSSID)
         / Dot11Beacon(cap=cap) / Dot11Elt(ID=0, info=b"Net") / Dot11Elt(ID=3, info=b"\x06"))
    for ie_id, info in ies:
        f /= Dot11Elt(ID=ie_id, info=info)
    return parse_beacon(RadioTap(bytes(f)))


def test_rsn_akm_parsing():
    assert rsn_akms(rsn_ie(2)) == [2]
    assert rsn_akms(rsn_ie(2, 8)) == [2, 8]
    assert rsn_akms(b"\x01") == []                      # truncated IE does not crash


def test_open_wep_wpa_wpa2_wpa3():
    assert beacon("ESS").encryption == "OPEN"
    assert beacon("ESS+privacy").encryption == "WEP"
    assert beacon("ESS+privacy", (221, b"\x00\x50\xf2\x01\x01\x00")).encryption == "WPA"
    assert beacon("ESS+privacy", (48, rsn_ie(2))).encryption == "WPA2"
    assert beacon("ESS+privacy", (48, rsn_ie(8))).encryption == "WPA3"
    assert beacon("ESS+privacy", (48, rsn_ie(2, 8))).encryption == "WPA2/WPA3"
    assert beacon("ESS+privacy", (48, rsn_ie(1))).encryption == "WPA2-Enterprise"


def test_classify_edge_cases():
    assert classify_encryption(False, [18], False) == "OWE"
    assert classify_encryption(True, [], False) == "WPA2"      # RSN present, unknown AKM
