import numpy as np
import pytest

from slam_robot_waypoints.grid import FREE, OCCUPIED, UNKNOWN, OccGrid


def field(xy):
    """A smooth, known RSSI field: a source at (1, -0.5) plus a gentle ripple."""
    xy = np.asarray(xy, float)
    d = np.hypot(xy[:, 0] - 1.0, xy[:, 1] + 0.5)
    return -40.0 - 6.0 * d + 2.0 * np.sin(0.5 * xy[:, 0])


@pytest.fixture
def lattice():
    """1 m lattice over a 7 x 7 m area (the survey area of the sim room)."""
    g = np.arange(-3.0, 3.01, 1.0)
    xy = np.array([(x, y) for y in g for x in g])
    return xy, field(xy)


@pytest.fixture
def room_occ():
    """8 x 8 m walled room map, 0.05 m cells, 10 x 10 m total with unknown outside."""
    n = 200
    occ = OccGrid(np.full((n, n), UNKNOWN, dtype=np.int8), 0.05, -5.0, -5.0)
    occ.data[10 + 20 : 190 - 20, 10 + 20 : 190 - 20] = FREE   # -3.5..3.5 m free
    occ.data[:, :] = np.where(occ.data == UNKNOWN, OCCUPIED, occ.data)
    return occ
