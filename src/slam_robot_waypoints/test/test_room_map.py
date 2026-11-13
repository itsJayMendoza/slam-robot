import pytest

from slam_robot_waypoints.generator import generate_waypoints
from slam_robot_waypoints.grid import FREE, OCCUPIED, load_map_yaml
from slam_robot_waypoints.room_map import make_room_map


@pytest.fixture
def room(tmp_path):
    yml, _ = make_room_map(3.10, 5.207, tmp_path)       # the user's room: 310 cm x 205 in
    return load_map_yaml(yml)


def test_free_floor_matches_the_measured_room(room):
    def at(x, y):
        return room.data[room.world_to_cell(x, y)]
    assert at(0.02, 0.02) == FREE and at(3.08, 5.19) == FREE      # inside, near both corners
    assert at(-0.03, 1.0) == OCCUPIED and at(1.0, -0.03) == OCCUPIED   # walls at x=0, y=0
    assert at(3.13, 1.0) == OCCUPIED and at(1.0, 5.23) == OCCUPIED     # far walls
    assert (room.origin_x, room.origin_y) == (pytest.approx(-0.1), pytest.approx(-0.1))


def test_middle_line_waypoints_are_on_free_floor(room):
    for y in (0.5, 1.0, 1.5, 2.0, 2.5):
        assert room.data[room.world_to_cell(1.55, y)] == FREE


def test_waypoint_generator_works_on_the_room(room):
    wps = generate_waypoints(room, 0.5, 0.25)
    assert wps and all(0.25 <= w.x <= 2.85 and 0.25 <= w.y <= 4.96 for w in wps)


def test_rejects_bad_size(tmp_path):
    with pytest.raises(ValueError):
        make_room_map(0, 2, tmp_path)
