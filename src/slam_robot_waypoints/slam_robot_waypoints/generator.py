"""Survey waypoint generation: lattice sampling + clearance + connectivity + lawnmower order."""
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from scipy import ndimage

from .grid import FREE, OccGrid

SPACINGS_M = (0.25, 0.5, 1.0, 1.5, 2.0)   # 0.25 is the ground-truth density


@dataclass(frozen=True)
class Waypoint:
    id: int        # 1-based visit order
    x: float       # metres, map frame
    y: float
    yaw: float     # radians, direction of travel to the next waypoint


def safe_mask(grid: OccGrid, clearance_m: float) -> np.ndarray:
    """Free cells at least clearance_m from any occupied OR unknown cell or the map edge."""
    free = np.pad(grid.data == FREE, 1, constant_values=False)
    dist = ndimage.distance_transform_edt(free)[1:-1, 1:-1] * grid.resolution
    return (grid.data == FREE) & (dist >= clearance_m - 1e-9)


def _select_region(safe: np.ndarray, grid: OccGrid, start_xy: Optional[Tuple[float, float]]):
    """Keep one 4-connected region of safe cells: the one at the start pose, else the largest.

    This drops free space the robot can never reach (for example outside the building)."""
    labels, n = ndimage.label(safe)
    if n == 0:
        return safe
    if start_xy is not None:
        r, c = grid.world_to_cell(*start_xy)
        r = min(max(r, 0), safe.shape[0] - 1)
        c = min(max(c, 0), safe.shape[1] - 1)
        # labels[idx] is the label of the safe cell nearest to the start (itself if safe)
        near_r, near_c = ndimage.distance_transform_edt(
            labels == 0, return_distances=False, return_indices=True)
        return labels == labels[near_r[r, c], near_c[r, c]]
    sizes = ndimage.sum(safe, labels, range(1, n + 1))
    return labels == (int(np.argmax(sizes)) + 1)


def generate_waypoints(grid: OccGrid, spacing_m: float, clearance_m: float = 0.25,
                       start_xy: Optional[Tuple[float, float]] = None) -> List[Waypoint]:
    """Waypoints on a lattice anchored at the map-frame origin, in lawnmower order.

    Anchoring at world (0, 0), not at the first free cell, makes every spacing a subset of
    the denser ones (1.5 m is inside 0.5 m is inside 0.25 m), which the interpolation and
    thinning experiments rely on. Same inputs always give the same list, so a run can repeat
    the exact waypoint set.
    """
    if spacing_m <= 0:
        raise ValueError("spacing_m must be > 0")
    keep = _select_region(safe_mask(grid, clearance_m), grid, start_xy)
    if not keep.any():
        return []

    rows, cols = keep.shape
    res = grid.resolution
    x_min, x_max = grid.origin_x, grid.origin_x + cols * res
    y_min, y_max = grid.origin_y, grid.origin_y + rows * res
    i0, i1 = math.ceil(x_min / spacing_m - 1e-9), math.floor(x_max / spacing_m + 1e-9)
    j0, j1 = math.ceil(y_min / spacing_m - 1e-9), math.floor(y_max / spacing_m + 1e-9)

    pts: List[Tuple[int, int, float, float]] = []   # (j, i, x, y)
    for j in range(j0, j1 + 1):
        for i in range(i0, i1 + 1):
            x, y = i * spacing_m, j * spacing_m
            r, c = grid.world_to_cell(x, y)
            if 0 <= r < rows and 0 <= c < cols and keep[r, c]:
                pts.append((j, i, x, y))

    # Boustrophedon: bottom row left-to-right, next right-to-left, and so on.
    ordered: List[Tuple[float, float]] = []
    row_ids = sorted({p[0] for p in pts})
    for n, j in enumerate(row_ids):
        row = sorted((p for p in pts if p[0] == j), key=lambda p: p[1], reverse=bool(n % 2))
        ordered.extend((p[2], p[3]) for p in row)

    waypoints = []
    yaw = 0.0
    for k, (x, y) in enumerate(ordered):
        if k + 1 < len(ordered):
            yaw = math.atan2(ordered[k + 1][1] - y, ordered[k + 1][0] - x)
        waypoints.append(Waypoint(k + 1, x, y, yaw))
    return waypoints
