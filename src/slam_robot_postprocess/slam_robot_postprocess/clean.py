"""Step 6: remove readings that cannot be trusted. Rejected records are only left out of the
cleaned data; the raw copy keeps them, and the count per reason is saved with the session."""
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from typing import Dict, List

from .config import Config

REASONS = ("incomplete", "unknown_waypoint", "invalid_value", "off_channel", "outlier")


@dataclass
class CleanResult:
    accepted: List[dict]
    rejected: List[dict]                        # {"seq", "reason", "detail"}
    counts: Dict[str, int]
    flagged_waypoints: List = field(default_factory=list)   # unknown IDs, reported to the operator
    near_sensitivity: int = 0                   # kept readings below cfg.near_sensitivity_dbm


def _missing(r):
    for key in ("waypoint_id", "bssid", "rssi"):
        if r.get(key) in (None, ""):
            return key
    return None


def clean(records: List[dict], waypoint_ids, cfg: Config) -> CleanResult:
    known = set(waypoint_ids)
    rejected, stage, flagged = [], [], []
    counts = Counter({r: 0 for r in REASONS})
    placeholders = {float(v) for v in cfg.placeholder_values}

    def reject(r, reason, detail):
        counts[reason] += 1
        rejected.append({"seq": r.get("seq"), "waypoint_id": r.get("waypoint_id"),
                         "bssid": r.get("bssid"), "rssi": r.get("rssi"),
                         "reason": reason, "detail": detail})

    for r in records:
        miss = _missing(r)
        if miss:
            reject(r, "incomplete", f"missing {miss}")
            continue
        if r["waypoint_id"] not in known:
            reject(r, "unknown_waypoint", f"waypoint {r['waypoint_id']!r} not in this session's list")
            if r["waypoint_id"] not in flagged:
                flagged.append(r["waypoint_id"])
            continue
        v = float(r["rssi"])
        if v in placeholders:
            reject(r, "invalid_value", f"placeholder value {v:g}")
            continue
        if not cfg.rssi_min_dbm <= v <= cfg.rssi_max_dbm:
            reject(r, "invalid_value", f"{v:g} dBm outside [{cfg.rssi_min_dbm:g}, {cfg.rssi_max_dbm:g}]")
            continue
        tuned, ch = r.get("tuned_channel"), r.get("channel")
        if cfg.reject_off_channel and tuned is not None and ch is not None and tuned != ch:
            reject(r, "off_channel", f"heard on channel {tuned}, AP is on {ch}")
            continue
        stage.append(r)

    # Statistical outliers: per AP (BSSID) per waypoint, measured from the median.
    groups = defaultdict(list)
    for r in stage:
        groups[(r["waypoint_id"], r["bssid"].upper())].append(r)
    accepted = []
    for (wid, bssid), rs in groups.items():
        if len(rs) < cfg.outlier_min_readings:
            accepted.extend(rs)
            continue
        vals = [float(r["rssi"]) for r in rs]
        med = statistics.median(vals)
        mad = statistics.median(abs(v - med) for v in vals)
        limit = max(cfg.outlier_min_dev_db, cfg.outlier_mad_k * 1.4826 * mad)
        for r, v in zip(rs, vals):
            if abs(v - med) > limit:
                reject(r, "outlier", f"{v:g} is {abs(v - med):.1f} dB from the median {med:g} "
                                     f"(limit {limit:.1f})")
            else:
                accepted.append(r)
    accepted.sort(key=lambda r: (r.get("seq") or 0, r.get("timestamp") or 0))
    near = sum(float(r["rssi"]) < cfg.near_sensitivity_dbm for r in accepted)
    return CleanResult(accepted, rejected, dict(counts), flagged, near)
