import hashlib
import json

import pytest

T0 = 1727123000.0
A, B = "AA:BB:CC:11:22:33", "AA:BB:CC:77:88:99"
G, I = "CC:DD:EE:44:55:66", "11:22:33:44:55:66"
NET = {A: ("CORP-WIFI", 6), B: ("CORP-WIFI", 11), G: ("GUEST-NET", 1), I: ("IoT-DEVICES", 11)}

# The client's worked example (T1 post-processing PDF, pages 5-6): (time, waypoint, x, y, heading,
# [(bssid, rssi), ...]). WP-07/08/09 are waypoint IDs 7/8/9.
SCANS = [
    (456.3, 7, 2.05, 1.78, 90.1, [(A, -56), (B, -67), (G, -79), (I, 0)]),
    (459.3, 7, 2.03, 1.83, 91.0, [(A, -55), (B, -68), (G, -81), (I, -127)]),
    (462.3, 7, 1.98, 1.79, 89.5, [(A, -57), (B, -70), (G, -70)]),
    (465.3, 7, 2.02, 1.80, 90.4, [(A, -54)]),
    (520.3, 9, 4.03, 1.80, 88.7, [(A, -70), (B, -57), (G, -63)]),
    (523.3, 9, 4.00, 1.83, 89.2, [(A, -72), (B, -59), (G, -64)]),
    (526.3, 9, 4.02, 1.80, 90.0, [(A, -71), (B, -58), (G, -66)]),
    (529.3, 9, 3.99, 1.81, 89.6, [(A, -71), (B, -58)]),
]


def example_records():
    recs = []
    for t, wid, x, y, hd, aps in SCANS:
        for bssid, rssi in aps:
            ssid, ch = NET[bssid]
            recs.append({"session_id": "001", "seq": len(recs) + 1, "timestamp": T0 + t,
                         "waypoint_id": wid, "x": x, "y": y, "heading": hd, "bssid": bssid,
                         "ssid": ssid, "channel": ch, "rssi": rssi, "encryption": "WPA2"})
    return recs


def write_session(folder, records, waypoints, status="complete", session_id="001", floor="1",
                  count=None, sha=None):
    folder.mkdir(parents=True, exist_ok=True)
    body = json.dumps(records).encode()
    (folder / "records.json").write_bytes(body)
    (folder / "waypoints.json").write_text(json.dumps(
        {"session_id": session_id, "floor": floor, "spacing_m": 1.0, "waypoints": waypoints}))
    (folder / "notification.json").write_text(json.dumps({
        "session_id": session_id, "floor": floor, "status": status,
        "start_timestamp": T0, "end_timestamp": T0 + 600,
        "record_count": len(records) if count is None else count,
        "records_sha256": sha or hashlib.sha256(body).hexdigest()}))
    return folder


def write_map(folder, width_px=400, height_px=300, res=0.05, origin=(-10.0, -7.5, 0.0)):
    """The example map: 400 x 300 px, 0.05 m/px, origin (-10, -7.5), all free floor."""
    folder.mkdir(parents=True, exist_ok=True)
    (folder / "map.pgm").write_bytes(b"P5\n%d %d\n255\n" % (width_px, height_px)
                                     + bytes([254]) * (width_px * height_px))
    (folder / "map.yaml").write_text(
        f"image: map.pgm\nresolution: {res}\norigin: [{origin[0]}, {origin[1]}, {origin[2]}]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return folder / "map.yaml"


EXAMPLE_WAYPOINTS = [{"waypoint_id": 7, "x": 2.0, "y": 1.8}, {"waypoint_id": 8, "x": 3.0, "y": 1.8},
                     {"waypoint_id": 9, "x": 4.0, "y": 1.8}]


@pytest.fixture
def example(tmp_path):
    session = write_session(tmp_path / "in" / "001", example_records(), EXAMPLE_WAYPOINTS)
    return session, write_map(tmp_path / "robot_map"), tmp_path / "store"
