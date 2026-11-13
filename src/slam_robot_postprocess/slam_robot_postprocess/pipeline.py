"""T1 post-processing pipeline, phases A-D, for one session.

  postprocess runs/001 --store runs/store --map maps/floor1/map.yaml

Outputs go to <store>/sessions/<session_id>/processed/:
  cleaning.json   step 6: counts per reason, every rejected record with its reason, flags
  merged.json     step 7: one value per BSSID per waypoint (+ readings, std, min, max), positions
  sets.json       step 8: per-network point sets (strongest BSSID per waypoint)
  grid.json       phase C: grid layout (+ free-floor mask for the floor-plan view)
  <method>_<network>.json and <method>_ALL.json   phase D, in T1's output format
  summary.json    what ran, what was skipped and why, the config and map fingerprint used
"""
import argparse
import json
import re
import sys
from pathlib import Path

from .clean import clean
from .config import Config
from .grid import MapError, free_mask, inside, load_or_build_layout, query_points
from .ingest import SessionError, check_session, ensure_map, store_raw_copy
from .interpolate import all_networks, run_idw, run_kriging, to_list
from .merge import group_by_ssid, merge, waypoint_positions

METHODS = {"IDW": run_idw, "Kriging": run_kriging}


def _safe(name: str) -> str:
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("_") or "network"


def _write(path: Path, obj):
    path.write_text(json.dumps(obj, indent=1))


def process(session_dir, store_dir, map_yaml=None, cfg: Config = None, accept_interrupted=False):
    cfg = cfg or Config()
    # Phase A
    note, records, wp_def = check_session(session_dir, accept_interrupted)
    sid, floor = str(note["session_id"]), str(note.get("floor") or wp_def.get("floor"))
    store_raw_copy(session_dir, store_dir, note)
    floor_map = ensure_map(store_dir, floor, map_yaml)
    out = Path(store_dir) / "sessions" / sid / "processed"
    out.mkdir(parents=True, exist_ok=True)
    waypoints = wp_def["waypoints"]

    # Phase B
    cl = clean(records, [w["waypoint_id"] for w in waypoints], cfg)
    positions = waypoint_positions(records, waypoints)
    merged = merge(cl.accepted, cfg.merge)
    sets = group_by_ssid(merged, positions, cfg.target_ssids)
    _write(out / "cleaning.json", {"accepted": len(cl.accepted), "rejected": len(cl.rejected),
                                   "counts": cl.counts, "flagged_waypoints": cl.flagged_waypoints,
                                   "kept_below_near_sensitivity": cl.near_sensitivity,
                                   "rejected_records": cl.rejected})
    _write(out / "merged.json", {"positions": {str(k): v for k, v in positions.items()},
                                 "merged": merged})
    _write(out / "sets.json", sets)

    # Phase C
    layout, reused = load_or_build_layout(floor_map, floor, cfg.cell_size_m)
    outside = [wid for wid, p in positions.items() if not inside(layout, p["x"], p["y"])]
    _write(out / "grid.json", {**layout, "free": free_mask(layout, floor_map)})
    query = query_points(layout)

    # Phase D
    runs = {}
    for method, fn in METHODS.items():
        per = {net: fn(pts, query, cfg) for net, pts in sets.items()}
        runs[method] = {net: r["status"] for net, r in per.items()}
        for net, r in per.items():
            if r.get("values") is None:
                continue
            doc = {"session_id": sid, "floor": floor, "ssid": net, "method": method,
                   "cells_across": layout["cells_across"], "cells_up": layout["cells_up"],
                   "values": to_list(r["values"])}
            if r.get("variance") is not None:
                doc["variance"] = to_list(r["variance"], 3)
            if r.get("variogram") is not None:
                doc["variogram"] = {"model": cfg.kriging_model, "parameters": r["variogram"]}
            _write(out / f"{method}_{_safe(net)}.json", doc)
        names, values, strongest = all_networks(per)
        if names:
            _write(out / f"{method}_ALL.json", {
                "session_id": sid, "floor": floor, "ssid": "ALL", "method": method,
                "networks": names, "cells_across": layout["cells_across"],
                "cells_up": layout["cells_up"], "values": to_list(values),
                "strongest": [int(i) for i in strongest]})

    summary = {
        "session_id": sid, "floor": floor, "status": note["status"],
        "partial_warning": note["status"] == "interrupted",
        "records": len(records), "accepted": len(cl.accepted), "rejected_by_reason": cl.counts,
        "flagged_waypoints": cl.flagged_waypoints, "waypoints_outside_grid": outside,
        "waypoint_offsets_m": {str(k): round(v["offset_m"], 3) for k, v in positions.items()},
        "networks": list(sets), "methods": runs,
        "grid": {k: layout[k] for k in ("cells_across", "cells_up", "cell_size_m")},
        "grid_reused": reused, "map_fingerprint": layout["map_fingerprint"],
        "config": cfg.to_dict(),
    }
    _write(out / "summary.json", summary)
    return summary, out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Post-process one survey session (T1 phases A-D)")
    ap.add_argument("session_dir", help="folder with notification.json, records.json, waypoints.json")
    ap.add_argument("--store", default="runs/store", help="laptop-side store (raw copies, maps, results)")
    ap.add_argument("--map", help="map.yaml to import for this floor if the store has none yet")
    ap.add_argument("--config", help="YAML file with settings to override")
    ap.add_argument("--cell", type=float, help="cell size in metres (default 0.25)")
    ap.add_argument("--target-ssid", action="append", help="limit to these networks (repeatable)")
    ap.add_argument("--accept-interrupted", action="store_true",
                    help="process an interrupted session anyway (partial warning)")
    args = ap.parse_args(argv)
    try:
        cfg = Config.load(args.config, cell_size_m=args.cell, target_ssids=args.target_ssid)
        s, out = process(args.session_dir, args.store, args.map, cfg, args.accept_interrupted)
    except (SessionError, MapError, ValueError) as e:
        sys.exit(f"stopped: {e}")
    print(f"session {s['session_id']} (floor {s['floor']}, {s['status']}): "
          f"{s['accepted']} of {s['records']} records accepted")
    for reason, n in s["rejected_by_reason"].items():
        if n:
            print(f"  rejected {reason}: {n}")
    if s["flagged_waypoints"]:
        print(f"  FLAG unknown waypoint IDs: {s['flagged_waypoints']}")
    if s["waypoints_outside_grid"]:
        print(f"  FLAG waypoints outside the grid: {s['waypoints_outside_grid']}")
    g = s["grid"]
    print(f"grid {g['cells_across']} x {g['cells_up']} cells of {g['cell_size_m']} m"
          f"{' (reused)' if s['grid_reused'] else ''}")
    for method, per in s["methods"].items():
        ran = [n for n, st in per.items() if st.startswith("ok")]
        print(f"{method}: ran for {len(ran)} of {len(per)} networks")
        for n, st in per.items():
            if not st.startswith("ok"):
                print(f"  {n}: {st}")
    print(f"results -> {out}")


if __name__ == "__main__":
    main()
