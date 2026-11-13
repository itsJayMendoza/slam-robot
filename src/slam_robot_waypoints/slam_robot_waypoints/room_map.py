"""Make a map_server-style map (.yaml + .pgm) of a plain rectangular room from tape-measure sizes,
for testing before SLAM works. The free floor is exactly x in [0, width], y in [0, length], with
(0, 0) at the chosen room corner, and a wall ring around it.

  make_room_map --width 3.10 --length 5.207 --out maps/floor1
"""
import argparse
from pathlib import Path

import numpy as np

FREE_PX, OCC_PX = 254, 0


def make_room_map(width_m: float, length_m: float, out_dir, name: str = "map",
                  resolution: float = 0.05, wall_m: float = 0.10):
    if width_m <= 0 or length_m <= 0:
        raise ValueError("room size must be positive")
    wall = max(1, int(round(wall_m / resolution)))
    # Nearest whole cell, so the map is within half a cell (2.5 cm) of the measured room.
    free_c = max(1, int(round(width_m / resolution)))
    free_r = max(1, int(round(length_m / resolution)))
    img = np.full((free_r + 2 * wall, free_c + 2 * wall), OCC_PX, dtype=np.uint8)
    img[wall:wall + free_r, wall:wall + free_c] = FREE_PX      # image row 0 is the TOP (max y)
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    pgm, yml = out / f"{name}.pgm", out / f"{name}.yaml"
    h, w = img.shape
    pgm.write_bytes(b"P5\n%d %d\n255\n" % (w, h) + img.tobytes())
    origin = -wall * resolution
    yml.write_text(
        f"image: {pgm.name}\nmode: trinary\nresolution: {resolution}\n"
        f"origin: [{origin:.3f}, {origin:.3f}, 0.0]\nnegate: 0\n"
        f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return yml, pgm


def main(argv=None):
    ap = argparse.ArgumentParser(description="Rectangular room -> map.yaml + map.pgm")
    ap.add_argument("--width", type=float, required=True, help="room size along x, metres")
    ap.add_argument("--length", type=float, required=True, help="room size along y, metres")
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default="map")
    ap.add_argument("--resolution", type=float, default=0.05)
    args = ap.parse_args(argv)
    yml, pgm = make_room_map(args.width, args.length, args.out, args.name, args.resolution)
    r = args.resolution
    print(f"{yml} and {pgm}: free floor x 0..{round(args.width / r) * r:.2f} m, "
          f"y 0..{round(args.length / r) * r:.2f} m (nearest {r * 100:.0f} cm), (0, 0) at the chosen corner")


if __name__ == "__main__":
    main()
