"""Save and load waypoint sets so a run can repeat exactly the same stops."""
import csv
import json
from pathlib import Path
from typing import List, Optional, Tuple

from .generator import Waypoint
from .grid import OccGrid


def save_waypoints(out_dir, waypoints: List[Waypoint], spacing_m: float, clearance_m: float,
                   grid: Optional[OccGrid] = None) -> Tuple[Path, Path]:
    """Write waypoints.json (for the robot) and waypoints.csv (for a human to mark the floor)."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    meta = {"frame": "map", "spacing_m": spacing_m, "clearance_m": clearance_m}
    if grid is not None:
        meta.update(map_resolution=grid.resolution, map_origin=[grid.origin_x, grid.origin_y])
    meta["waypoints"] = [{"id": w.id, "x": w.x, "y": w.y, "yaw": w.yaw} for w in waypoints]
    json_path, csv_path = out / "waypoints.json", out / "waypoints.csv"
    json_path.write_text(json.dumps(meta, indent=2))
    with open(csv_path, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["waypoint_id", "x_m", "y_m", "yaw_rad"])
        for w in waypoints:
            wr.writerow([w.id, f"{w.x:.3f}", f"{w.y:.3f}", f"{w.yaw:.4f}"])
    return json_path, csv_path


def load_waypoints(path) -> Tuple[List[Waypoint], dict]:
    path = Path(path)
    if path.is_dir():
        path = path / "waypoints.json"
    meta = json.loads(path.read_text())
    wps = [Waypoint(int(w["id"]), float(w["x"]), float(w["y"]), float(w["yaw"]))
           for w in meta["waypoints"]]
    return wps, {k: v for k, v in meta.items() if k != "waypoints"}
