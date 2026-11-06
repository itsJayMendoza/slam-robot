"""generate_waypoints --map ~/map.yaml --spacing 1.0 --out runs/test1 [--start X Y]"""
import argparse

from .generator import SPACINGS_M, generate_waypoints
from .grid import load_map_yaml
from .io import save_waypoints


def main():
    ap = argparse.ArgumentParser(description="Generate survey waypoints from a saved map")
    ap.add_argument("--map", required=True, help="map .yaml from map_saver / map_server")
    ap.add_argument("--spacing", type=float, required=True,
                    help=f"grid spacing in metres (experiments use {SPACINGS_M})")
    ap.add_argument("--clearance", type=float, default=0.25,
                    help="min distance to walls/unknown space, metres (default 0.25)")
    ap.add_argument("--start", type=float, nargs=2, metavar=("X", "Y"),
                    help="robot start in the map frame; keeps only space reachable from it")
    ap.add_argument("--out", required=True, help="run folder to write waypoints.json/.csv into")
    args = ap.parse_args()

    grid = load_map_yaml(args.map)
    wps = generate_waypoints(grid, args.spacing, args.clearance,
                             tuple(args.start) if args.start else None)
    json_path, csv_path = save_waypoints(args.out, wps, args.spacing, args.clearance, grid)
    print(f"{len(wps)} waypoints -> {json_path} and {csv_path}")
    if not wps:
        raise SystemExit("no free space found; check the map and --clearance")


if __name__ == "__main__":
    main()
