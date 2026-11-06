"""Interpolate onto a regular grid, limited to the free space of the floor plan."""
from dataclasses import dataclass
from typing import Optional

import numpy as np

from slam_robot_waypoints.grid import FREE, OccGrid

from .methods import predict


@dataclass
class Surface:
    x: np.ndarray                 # (cols,) cell-centre x, metres
    y: np.ndarray                 # (rows,) cell-centre y, row 0 = lowest y
    values: np.ndarray            # (rows, cols), NaN outside the mask
    variance: Optional[np.ndarray]  # Kriging error variance, same shape; None for IDW
    cell_m: float


def free_space_mask(occ: OccGrid, cell_m: float):
    """Regular grid covering the map; True where the floor plan is free space.

    NOTE: interpolation across walls is a known limitation (the wall-crossing problem): two
    rooms separated by a wall are treated as one continuous field."""
    rows = int(np.floor(occ.data.shape[0] * occ.resolution / cell_m))
    cols = int(np.floor(occ.data.shape[1] * occ.resolution / cell_m))
    x = occ.origin_x + (np.arange(cols) + 0.5) * cell_m
    y = occ.origin_y + (np.arange(rows) + 0.5) * cell_m
    mask = np.zeros((rows, cols), dtype=bool)
    for r, yy in enumerate(y):
        for c, xx in enumerate(x):
            rr, cc = occ.world_to_cell(xx, yy)
            mask[r, c] = occ.data[rr, cc] == FREE
    return x, y, mask


def interpolate_surface(method: str, xy, z, occ: OccGrid, cell_m: float = 0.1,
                        **kw) -> Surface:
    x, y, mask = free_space_mask(occ, cell_m)
    gx, gy = np.meshgrid(x, y)
    query = np.column_stack([gx[mask], gy[mask]])
    values = np.full(mask.shape, np.nan)
    variance = None
    if len(query):
        pred, var = predict(method, xy, z, query, **kw)
        values[mask] = pred
        if var is not None:
            variance = np.full(mask.shape, np.nan)
            variance[mask] = var
    return Surface(x, y, values, variance, cell_m)
