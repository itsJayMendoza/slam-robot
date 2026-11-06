import math

import numpy as np
import pytest

from slam_robot_waypoints.generator import generate_waypoints, safe_mask
from slam_robot_waypoints.grid import FREE, OCCUPIED, UNKNOWN, OccGrid

CLEAR = 0.25


def keys(wps):
    return {(round(w.x, 6), round(w.y, 6)) for w in wps}


def test_every_waypoint_is_free_and_clear_of_obstacles(room):
    """Independent brute-force check: nothing blocked within the clearance radius."""
    wps = generate_waypoints(room, 0.5, CLEAR)
    assert wps
    r_cells = int(math.ceil(CLEAR / room.resolution))
    for w in wps:
        r, c = room.world_to_cell(w.x, w.y)
        assert room.data[r, c] == FREE
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                if math.hypot(dr, dc) * room.resolution < CLEAR - 1e-9:
                    assert room.data[r + dr, c + dc] == FREE, (w, dr, dc)


def test_points_lie_on_world_origin_lattice(room):
    for w in generate_waypoints(room, 0.5, CLEAR):
        assert abs(w.x / 0.5 - round(w.x / 0.5)) < 1e-9
        assert abs(w.y / 0.5 - round(w.y / 0.5)) < 1e-9


def test_spacings_are_nested_for_cross_validation(room):
    """Coarser surveys are subsets of the 0.25 m ground-truth set."""
    dense = keys(generate_waypoints(room, 0.25, CLEAR))
    for s in (0.5, 1.0, 1.5, 2.0):
        assert keys(generate_waypoints(room, s, CLEAR)) <= dense, s


def test_denser_spacing_gives_more_points(room):
    n = [len(generate_waypoints(room, s, CLEAR)) for s in (2.0, 1.5, 1.0, 0.5, 0.25)]
    assert n == sorted(n) and n[0] < n[-1]


def test_box_and_cylinder_excluded(room):
    wps = generate_waypoints(room, 0.25, CLEAR)
    for w in wps:
        assert not (1.0 - CLEAR < w.x < 2.0 + CLEAR and 0.5 - CLEAR < w.y < 1.5 + CLEAR)
        assert math.hypot(w.x + 1.5, w.y + 1.5) >= 0.4 + CLEAR - 0.05


def test_walls_and_unknown_space_excluded(room):
    for w in generate_waypoints(room, 0.25, CLEAR):
        assert abs(w.x) <= 4.0 - CLEAR and abs(w.y) <= 4.0 - CLEAR


def test_lawnmower_order(room):
    wps = generate_waypoints(room, 1.0, CLEAR)
    rows = {}
    for w in wps:
        rows.setdefault(round(w.y, 6), []).append(w)
    ys = [round(w.y, 6) for w in wps]
    assert ys == sorted(ys)                         # rows bottom to top
    for n, y in enumerate(sorted(rows)):
        xs = [w.x for w in rows[y]]                 # in visiting order
        assert xs == sorted(xs, reverse=bool(n % 2)), y


def test_ids_are_sequential_from_one(room):
    wps = generate_waypoints(room, 1.0, CLEAR)
    assert [w.id for w in wps] == list(range(1, len(wps) + 1))


def test_yaw_points_to_next_waypoint(room):
    wps = generate_waypoints(room, 1.0, CLEAR)
    for a, b in zip(wps, wps[1:]):
        assert a.yaw == pytest.approx(math.atan2(b.y - a.y, b.x - a.x))
    assert wps[-1].yaw == wps[-2].yaw


def test_deterministic(room):
    assert generate_waypoints(room, 0.5, CLEAR) == generate_waypoints(room, 0.5, CLEAR)


def _two_rooms():
    """Big room on the left, small separate free pocket on the right (not connected)."""
    g = OccGrid(np.full((200, 400), UNKNOWN, dtype=np.int8), 0.05, 0.0, 0.0)
    g.data[10:190, 10:250] = FREE          # 12 x 9 m
    g.data[:, 250:260] = OCCUPIED          # wall
    g.data[10:190, 260:390] = FREE         # 6.5 x 9 m, disconnected
    return g


def test_default_keeps_only_largest_connected_region():
    wps = generate_waypoints(_two_rooms(), 1.0, CLEAR)
    assert wps and all(w.x < 12.5 for w in wps)


def test_start_pose_selects_its_region():
    wps = generate_waypoints(_two_rooms(), 1.0, CLEAR, start_xy=(14.0, 4.0))
    assert wps and all(w.x > 13.0 for w in wps)


def test_start_on_blocked_cell_uses_nearest_region():
    wps = generate_waypoints(_two_rooms(), 1.0, CLEAR, start_xy=(12.7, 4.0))  # inside the wall
    assert wps  # does not crash; picks a neighbouring region


def test_doorway_connects_two_rooms():
    g = _two_rooms()
    g.data[80:120, 250:260] = FREE         # 2 m opening
    wps = generate_waypoints(g, 1.0, CLEAR)
    assert any(w.x < 12 for w in wps) and any(w.x > 13 for w in wps)


def test_narrow_gap_below_clearance_is_not_passable():
    g = _two_rooms()
    g.data[100:110, 250:260] = FREE        # 0.5 m gap: robot half-width + margin does not fit
    wps = generate_waypoints(g, 1.0, 0.3)
    assert all(w.x < 12.5 for w in wps)


def test_no_free_space_gives_empty_list():
    g = OccGrid(np.full((50, 50), OCCUPIED, dtype=np.int8), 0.05, 0.0, 0.0)
    assert generate_waypoints(g, 0.5, CLEAR) == []


def test_invalid_spacing_rejected(room):
    with pytest.raises(ValueError):
        generate_waypoints(room, 0.0)


def test_safe_mask_treats_unknown_as_blocked():
    g = OccGrid(np.full((60, 60), FREE, dtype=np.int8), 0.05, 0.0, 0.0)
    g.data[30, 30] = UNKNOWN
    m = safe_mask(g, 0.25)                 # 0.25 m = 5 cells
    assert not m[30, 30]                   # the unknown cell itself
    assert not m[30, 34] and not m[34, 30]  # within 4 cells
    assert m[30, 36] and m[36, 30]         # beyond clearance
    assert not m[0, 30] and not m[30, 0]   # map edge counts as blocked
