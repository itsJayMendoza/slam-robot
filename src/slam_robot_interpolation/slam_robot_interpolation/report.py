"""Per-AP summary table and coverage-gap report."""
import math
from typing import Dict, List, Optional

import numpy as np
from scipy import ndimage

from .surface import Surface

RSSI_THRESHOLD_DBM = -67.0   # Cisco enterprise guidance, configurable
SNR_THRESHOLD_DB = 20.0


def per_ap_summary(rows: List[Dict[str, str]], rssi_threshold: float = RSSI_THRESHOLD_DBM,
                   snr_threshold: float = SNR_THRESHOLD_DB) -> List[Dict[str, object]]:
    """One row per BSSID. 'pass' means the AP's mean RSSI is at or above the threshold; SNR is
    only judged when it was measured (otherwise snr_pass is None, not a guess)."""
    by_ap: Dict[str, List[Dict[str, str]]] = {}
    for r in rows:
        by_ap.setdefault(r["bssid"].lower(), []).append(r)
    out = []
    for bssid, rs in by_ap.items():
        rssi = np.array([float(r["rssi_mean"]) for r in rs])
        snr = np.array([float(r["snr_mean"]) for r in rs
                        if r.get("snr_mean") not in (None, "", "nan")], dtype=float)
        snr = snr[np.isfinite(snr)]
        mean_rssi = float(rssi.mean())
        out.append({
            "bssid": bssid, "ssid": rs[0]["ssid"], "channel": rs[0]["channel"],
            "n_waypoints": len(rs), "rssi_mean": mean_rssi,
            "rssi_min": float(rssi.min()), "rssi_max": float(rssi.max()),
            "snr_mean": float(snr.mean()) if len(snr) else None,
            "rssi_pass": mean_rssi >= rssi_threshold,
            "snr_pass": (float(snr.mean()) >= snr_threshold) if len(snr) else None,
        })
    out.sort(key=lambda d: d["rssi_mean"], reverse=True)
    return out


def find_gaps(surface: Surface, threshold: float, below: bool = True,
              min_area_m2: float = 0.0) -> List[Dict[str, float]]:
    """Connected regions of the surface worse than the threshold (RSSI below -67 dBm, SNR below
    20 dB, ...): approximate location, area, and mean value. For Kriging surfaces, also the mean
    prediction standard deviation as a confidence indicator."""
    v = surface.values
    bad = np.isfinite(v) & ((v < threshold) if below else (v > threshold))
    labels, n = ndimage.label(bad)
    cell_area = surface.cell_m ** 2
    gaps = []
    for lab in range(1, n + 1):
        m = labels == lab
        area = float(m.sum() * cell_area)
        if area < min_area_m2:
            continue
        rr, cc = np.nonzero(m)
        gap = {"area_m2": area, "cx": float(surface.x[cc].mean()), "cy": float(surface.y[rr].mean()),
               "mean_value": float(v[m].mean()), "worst_value": float(v[m].min() if below else v[m].max())}
        if surface.variance is not None:
            gap["mean_std"] = float(np.sqrt(surface.variance[m]).mean())
        gaps.append(gap)
    gaps.sort(key=lambda g: g["area_m2"], reverse=True)
    return gaps


def coverage_fraction(surface: Surface, threshold: float) -> float:
    """Share of the mapped free space at or above the threshold."""
    finite = np.isfinite(surface.values)
    return float((surface.values[finite] >= threshold).mean()) if finite.any() else math.nan
