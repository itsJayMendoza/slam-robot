import csv

import numpy as np

from slam_robot_waypoints.generator import generate_waypoints
from slam_robot_waypoints.grid import FREE, OCCUPIED, UNKNOWN, load_map_yaml
from slam_robot_waypoints.io import load_waypoints, save_waypoints


def test_save_and_load_roundtrip(tmp_path, room):
    wps = generate_waypoints(room, 1.0, 0.25)
    jp, cp = save_waypoints(tmp_path / "run1", wps, 1.0, 0.25, room)
    loaded, meta = load_waypoints(tmp_path / "run1")
    assert loaded == wps                              # exact repeat of the same set
    assert meta["spacing_m"] == 1.0 and meta["frame"] == "map"
    assert load_waypoints(jp)[0] == wps


def test_csv_is_human_readable(tmp_path, room):
    wps = generate_waypoints(room, 2.0, 0.25)
    _, cp = save_waypoints(tmp_path, wps, 2.0, 0.25)
    rows = list(csv.reader(open(cp)))
    assert rows[0] == ["waypoint_id", "x_m", "y_m", "yaw_rad"]
    assert len(rows) == len(wps) + 1
    assert rows[1][0] == "1"


def _write_map(tmp_path, pixels, negate=0, extra=""):
    h, w = pixels.shape
    (tmp_path / "m.pgm").write_bytes(b"P5\n# comment\n%d %d\n255\n" % (w, h) + pixels.tobytes())
    (tmp_path / "m.yaml").write_text(
        f"image: m.pgm\nresolution: 0.05\norigin: [-1.0, -2.0, 0.0]\nnegate: {negate}\n"
        f"occupied_thresh: 0.65\nfree_thresh: 0.196\n{extra}")
    return tmp_path / "m.yaml"


def test_load_map_thresholds_and_orientation(tmp_path):
    # image row 0 is the TOP of the map. Column 0 = free (254), 1 = occupied (0), 2 = unknown (205)
    px = np.array([[254, 0, 205],
                   [205, 254, 0]], dtype=np.uint8)
    g = load_map_yaml(_write_map(tmp_path, px))
    assert (g.resolution, g.origin_x, g.origin_y) == (0.05, -1.0, -2.0)
    # grid row 0 is the lowest y, so it is the image's LAST row
    assert g.data.tolist() == [[UNKNOWN, FREE, OCCUPIED],
                               [FREE, OCCUPIED, UNKNOWN]]


def test_world_to_cell(tmp_path):
    px = np.full((10, 10), 254, dtype=np.uint8)
    g = load_map_yaml(_write_map(tmp_path, px))
    assert g.world_to_cell(-1.0, -2.0) == (0, 0)
    assert g.world_to_cell(-0.95, -1.95) == (1, 1)
    assert g.world_to_cell(-0.9, -1.9) == (2, 2)


def test_pgm_end_to_end_with_generator(tmp_path):
    px = np.full((200, 200), 254, dtype=np.uint8)        # 10 x 10 m free
    px[:5, :] = px[-5:, :] = px[:, :5] = px[:, -5:] = 0  # wall border
    g = load_map_yaml(_write_map(tmp_path, px))
    wps = generate_waypoints(g, 1.0, 0.25)
    assert len(wps) > 20
