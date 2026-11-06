"""Read waypoint_avg.csv (one row per waypoint per BSSID) into arrays."""
import csv
from typing import Dict, List, Optional, Tuple

import numpy as np


def read_waypoint_avg(path) -> List[Dict[str, str]]:
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _num(v) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def samples(rows: List[Dict[str, str]], metric: str = "rssi_mean", bssid: Optional[str] = None,
            ssid: Optional[str] = None, strongest: bool = False
            ) -> Tuple[np.ndarray, np.ndarray, List[int]]:
    """(xy, values, waypoint_ids) for one metric.

    bssid / ssid filter the rows. With strongest=True, keep only the strongest-RSSI AP at each
    waypoint (what a client at that spot would connect to). Rows where the metric is empty or
    NaN (for example SNR when no noise floor was measured) are dropped, never filled in."""
    sel = [r for r in rows
           if (bssid is None or r["bssid"].lower() == bssid.lower())
           and (ssid is None or r["ssid"] == ssid)]
    if strongest:
        best: Dict[str, Dict[str, str]] = {}
        for r in sel:
            cur = best.get(r["waypoint_id"])
            if cur is None or _num(r["rssi_mean"]) > _num(cur["rssi_mean"]):
                best[r["waypoint_id"]] = r
        sel = list(best.values())
    xy, z, ids = [], [], []
    for r in sel:
        v = _num(r.get(metric))
        if np.isfinite(v):
            xy.append((float(r["x"]), float(r["y"])))
            z.append(v)
            ids.append(int(r["waypoint_id"]))
    return np.array(xy).reshape(-1, 2), np.array(z), ids
