"""Load a saved map (nav2 map_server .yaml + .pgm/.png) into a numpy occupancy grid."""
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

FREE, OCCUPIED, UNKNOWN = 0, 100, -1


@dataclass
class OccGrid:
    data: np.ndarray       # int8 [rows, cols]; row 0 is the LOWEST y (ROS OccupancyGrid order)
    resolution: float      # metres per cell
    origin_x: float        # world position of the lower-left corner of cell (0, 0)
    origin_y: float

    def world_to_cell(self, x: float, y: float):
        """(row, col) of the cell containing world point (x, y); may be out of range."""
        col = int(np.floor((x - self.origin_x) / self.resolution + 1e-9))
        row = int(np.floor((y - self.origin_y) / self.resolution + 1e-9))
        return row, col


def _read_pgm(path: Path) -> np.ndarray:
    with open(path, "rb") as f:
        raw = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:                       # magic, width, height, maxval
        while raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b"#":
            pos = raw.index(b"\n", pos) + 1
            continue
        end = pos
        while not raw[end:end + 1].isspace():
            end += 1
        tokens.append(raw[pos:end])
        pos = end
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic != b"P5" or maxval > 255:
        raise ValueError(f"{path}: only 8-bit binary PGM (P5) is supported")
    pos += 1                                     # single whitespace after maxval
    return np.frombuffer(raw, dtype=np.uint8, count=w * h, offset=pos).reshape(h, w)


def _read_image(path: Path) -> np.ndarray:
    if path.suffix.lower() == ".pgm":
        return _read_pgm(path)
    from PIL import Image                        # png maps; Pillow is optional
    return np.asarray(Image.open(path).convert("L"))


def load_map_yaml(yaml_path) -> OccGrid:
    yaml_path = Path(yaml_path)
    meta = yaml.safe_load(yaml_path.read_text())
    image = _read_image(yaml_path.parent / meta["image"]).astype(float)

    occ = (255.0 - image) / 255.0 if not meta.get("negate", 0) else image / 255.0
    occ_t = float(meta.get("occupied_thresh", 0.65))
    free_t = float(meta.get("free_thresh", 0.196))
    data = np.full(image.shape, UNKNOWN, dtype=np.int8)
    data[occ < free_t] = FREE
    data[occ > occ_t] = OCCUPIED
    data = np.flipud(data)                       # image row 0 is the top (max y)
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    return OccGrid(np.ascontiguousarray(data), float(meta["resolution"]), ox, oy)
