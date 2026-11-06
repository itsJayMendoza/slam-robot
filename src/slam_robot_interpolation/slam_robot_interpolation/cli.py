"""interpolate surface --csv waypoint_avg.csv --map map.yaml --method kriging --out surf.npz
   interpolate cv      --csv waypoint_avg.csv [--k 5]"""
import argparse

import numpy as np

from slam_robot_waypoints.grid import load_map_yaml

from .data import read_waypoint_avg, samples
from .methods import VARIOGRAM_MODELS
from .surface import interpolate_surface
from .validation import cross_validate


def _load(args):
    rows = read_waypoint_avg(args.csv)
    return samples(rows, args.metric, args.bssid, args.ssid, args.strongest)


def main():
    ap = argparse.ArgumentParser(description="Interpolate survey data")
    ap.add_argument("--csv", required=True, help="waypoint_avg.csv")
    ap.add_argument("--metric", default="rssi_mean")
    ap.add_argument("--bssid")
    ap.add_argument("--ssid")
    ap.add_argument("--strongest", action="store_true", help="strongest AP at each waypoint")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("surface")
    s.add_argument("--map", required=True)
    s.add_argument("--method", choices=("idw", "kriging"), default="kriging")
    s.add_argument("--power", type=float, default=2.0)
    s.add_argument("--variogram", choices=VARIOGRAM_MODELS, default="spherical")
    s.add_argument("--cell", type=float, default=0.1)
    s.add_argument("--out", required=True, help="output .npz (x, y, values, variance)")

    c = sub.add_parser("cv")
    c.add_argument("--k", type=int, default=5)
    c.add_argument("--power", type=float, default=2.0)
    c.add_argument("--variogram", choices=VARIOGRAM_MODELS, default="spherical")

    args = ap.parse_args()
    xy, z, _ = _load(args)
    if len(z) == 0:
        raise SystemExit("no usable samples for that metric/filter")

    if args.cmd == "surface":
        surf = interpolate_surface(args.method, xy, z, load_map_yaml(args.map), args.cell,
                                   power=args.power, variogram_model=args.variogram)
        np.savez(args.out, x=surf.x, y=surf.y, values=surf.values,
                 variance=surf.variance if surf.variance is not None else np.array([]))
        print(f"{len(z)} samples -> {args.out}  grid {surf.values.shape}")
    else:
        for method in ("idw", "kriging"):
            m = cross_validate(method, xy, z, args.k, power=args.power,
                               variogram_model=args.variogram)
            print(f"{method:8} k={args.k}  RMSE {m['rmse']:.2f}  MAE {m['mae']:.2f}  "
                  f"bias {m['bias']:+.2f}  (n={m['n']})")


if __name__ == "__main__":
    main()
