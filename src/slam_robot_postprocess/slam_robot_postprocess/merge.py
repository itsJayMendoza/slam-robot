"""Step 7 (merge repeated readings) and step 8 (group per SSID)."""
import math
import statistics
from collections import Counter, defaultdict
from typing import Dict, List, Optional


def waypoint_positions(records: List[dict], waypoints: List[dict]) -> Dict[int, dict]:
    """Measured position of each waypoint: the mean of the robot's positions over the distinct
    scans taken there (a scan counts once, however many APs it heard)."""
    intended = {w["waypoint_id"]: (float(w["x"]), float(w["y"])) for w in waypoints}
    scans = defaultdict(set)
    for r in records:
        wid = r.get("waypoint_id")
        if wid in intended and r.get("x") is not None and r.get("y") is not None:
            scans[wid].add((r.get("timestamp"), float(r["x"]), float(r["y"])))
    out = {}
    for wid, (ix, iy) in intended.items():
        s = scans.get(wid)
        if not s:
            continue
        mx = statistics.fmean(p[1] for p in s)
        my = statistics.fmean(p[2] for p in s)
        out[wid] = {"x": mx, "y": my, "intended_x": ix, "intended_y": iy,
                    "offset_m": math.hypot(mx - ix, my - iy), "scans": len(s)}
    return out


def merge(accepted: List[dict], how: str = "median") -> List[dict]:
    """One value per BSSID per waypoint (median by default), plus spread for experiments E2/E3."""
    groups = defaultdict(list)
    for r in accepted:
        groups[(r["waypoint_id"], r["bssid"].upper())].append(r)
    out = []
    for (wid, bssid), rs in sorted(groups.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        vals = [float(r["rssi"]) for r in rs]
        out.append({
            "waypoint_id": wid,
            "bssid": bssid,
            "ssid": Counter(r.get("ssid") or "" for r in rs).most_common(1)[0][0],
            "channel": Counter(r.get("channel") for r in rs).most_common(1)[0][0],
            "rssi": statistics.median(vals) if how == "median" else statistics.fmean(vals),
            "readings": len(vals),
            "std": statistics.stdev(vals) if len(vals) > 1 else 0.0,
            "min": min(vals),
            "max": max(vals),
        })
    return out


def network_key(ssid: str, bssid: str) -> str:
    """Hidden networks (empty SSID) are kept apart per BSSID, never merged into one 'network'."""
    return ssid if ssid else f"<hidden> {bssid}"


def group_by_ssid(merged: List[dict], positions: Dict[int, dict],
                  target_ssids: Optional[List[str]] = None) -> Dict[str, List[dict]]:
    """For each network at each waypoint keep its strongest BSSID. A network not heard at a
    waypoint simply has no point there (no placeholder value)."""
    best = {}
    for m in merged:
        if m["waypoint_id"] not in positions:
            continue
        key = (network_key(m["ssid"], m["bssid"]), m["waypoint_id"])
        cur = best.get(key)
        if cur is None or (m["rssi"], cur["bssid"]) > (cur["rssi"], m["bssid"]):
            best[key] = m
    sets = defaultdict(list)
    for (net, wid), m in sorted(best.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        if target_ssids and net not in target_ssids:
            continue
        p = positions[wid]
        sets[net].append({"waypoint_id": wid, "x": p["x"], "y": p["y"],
                          "rssi": m["rssi"], "best_bssid": m["bssid"]})
    return dict(sorted(sets.items()))
