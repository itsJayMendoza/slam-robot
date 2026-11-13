"""Phase C: the interpolation grid, computed from the floor's saved map and the cell size."""
import json
import math
from pathlib import Path

import numpy as np
import yaml

from slam_robot_waypoints.grid import FREE, _read_image, load_map_yaml

from .ingest import file_sha256


class MapError(Exception):
    pass


def read_map(map_yaml):
    p = Path(map_yaml)
    meta = yaml.safe_load(p.read_text())
    origin = meta.get("origin", [0, 0, 0])
    if len(origin) > 2 and abs(float(origin[2])) > 1e-9:
        raise MapError(f"{p}: origin rotation is {origin[2]}, the grid calculation assumes 0")
    image_path = p.parent / meta["image"]
    h, w = _read_image(image_path).shape
    return {"width_px": w, "height_px": h, "resolution": float(meta["resolution"]),
            "origin_x": float(origin[0]), "origin_y": float(origin[1]),
            "fingerprint": file_sha256(p, image_path)}


def build_layout(map_info: dict, floor: str, cell_size_m: float) -> dict:
    width_m = map_info["width_px"] * map_info["resolution"]
    height_m = map_info["height_px"] * map_info["resolution"]
    return {
        "floor": str(floor),
        "map_fingerprint": map_info["fingerprint"],
        "origin_x": map_info["origin_x"],
        "origin_y": map_info["origin_y"],
        "cell_size_m": cell_size_m,
        "cells_across": int(math.ceil(width_m / cell_size_m - 1e-9)),   # rounded up: whole floor covered
        "cells_up": int(math.ceil(height_m / cell_size_m - 1e-9)),
    }


def load_or_build_layout(map_yaml, floor: str, cell_size_m: float):
    """Reuse the stored layout when the map fingerprint and cell size are unchanged."""
    info = read_map(map_yaml)
    cache = Path(map_yaml).parent / f"grid_{cell_size_m:g}.json"
    if cache.exists():
        old = json.loads(cache.read_text())
        if old.get("map_fingerprint") == info["fingerprint"] and old.get("cell_size_m") == cell_size_m:
            return old, True
    layout = build_layout(info, floor, cell_size_m)
    cache.write_text(json.dumps(layout, indent=1))
    return layout, False


def cell_centers(layout: dict):
    c = layout["cell_size_m"]
    xs = layout["origin_x"] + (np.arange(layout["cells_across"]) + 0.5) * c
    ys = layout["origin_y"] + (np.arange(layout["cells_up"]) + 0.5) * c
    return xs, ys


def query_points(layout: dict) -> np.ndarray:
    """All cell centres in list order: row by row from the bottom-left, index = j * across + i."""
    xs, ys = cell_centers(layout)
    gx, gy = np.meshgrid(xs, ys)
    return np.column_stack([gx.ravel(), gy.ravel()])


def inside(layout: dict, x: float, y: float) -> bool:
    c = layout["cell_size_m"]
    return (layout["origin_x"] <= x < layout["origin_x"] + layout["cells_across"] * c and
            layout["origin_y"] <= y < layout["origin_y"] + layout["cells_up"] * c)


def free_mask(layout: dict, map_yaml) -> list:
    """1 where the cell centre is free floor on the map, 0 for walls, unknown space or outside.
    Not used by T1's interpolation; stored so the floor-plan view can grey out such cells."""
    occ = load_map_yaml(map_yaml)
    out = []
    for x, y in query_points(layout):
        r, c = occ.world_to_cell(x, y)
        ok = 0 <= r < occ.data.shape[0] and 0 <= c < occ.data.shape[1] and occ.data[r, c] == FREE
        out.append(int(ok))
    return out
