import numpy as np
import pytest

from slam_robot_waypoints.grid import FREE, OCCUPIED, UNKNOWN, OccGrid

RES = 0.05


def sim_room(box=True, cylinder=True) -> OccGrid:
    """The Gazebo room: 8x8 m walls at +-4, 1x1 m box at (1.5, 1), r=0.4 cylinder at (-1.5,-1.5).
    The map covers 10x10 m, so a ring of unknown space surrounds the room."""
    n = int(10 / RES)
    g = OccGrid(np.full((n, n), UNKNOWN, dtype=np.int8), RES, -5.0, -5.0)

    def cells(x0, x1, y0, y1):
        r0, c0 = g.world_to_cell(x0, y0)
        r1, c1 = g.world_to_cell(x1, y1)
        return slice(r0, r1 + 1), slice(c0, c1 + 1)

    g.data[cells(-3.95, 3.95, -3.95, 3.95)] = FREE
    for wall in ((-4.05, 4.05, 3.95, 4.05), (-4.05, 4.05, -4.05, -3.95),
                 (3.95, 4.05, -4.05, 4.05), (-4.05, -3.95, -4.05, 4.05)):
        g.data[cells(*wall)] = OCCUPIED
    if box:
        g.data[cells(1.0, 2.0, 0.5, 1.5)] = OCCUPIED
    if cylinder:
        ys, xs = np.mgrid[0:n, 0:n]
        x = g.origin_x + (xs + 0.5) * RES
        y = g.origin_y + (ys + 0.5) * RES
        g.data[(x + 1.5) ** 2 + (y + 1.5) ** 2 <= 0.4 ** 2] = OCCUPIED
    return g


@pytest.fixture
def room():
    return sim_room()
