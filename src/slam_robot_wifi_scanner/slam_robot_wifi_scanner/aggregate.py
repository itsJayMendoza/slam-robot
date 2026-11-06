"""Outlier filtering and per-AP averaging of beacon readings."""
import math
import statistics
from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from .beacon import Beacon


@dataclass
class ApStats:
    bssid: str
    ssid: str
    channel: Optional[int]
    n_samples: int
    n_rejected: int
    rssi_mean: float
    rssi_std: float
    rssi_median: float
    rssi_min: float
    rssi_max: float
    noise_valid: bool
    noise_mean: float   # NaN unless noise_valid
    noise_std: float
    chan_util_valid: bool
    chan_util_mean: float  # NaN unless chan_util_valid


def keep_mask(values: Sequence[float], min_rssi: float = -95.0,
              mad_k: float = 3.0, min_dev_db: float = 5.0) -> List[bool]:
    """True for readings to keep.

    1. Drop anything below min_rssi (below what the receiver can really hear).
    2. With >= 3 left, drop readings further than max(min_dev_db, mad_k * 1.4826 * MAD)
       from the median, so one stray -100 dBm frame among -40s cannot skew a dwell.
    """
    keep = [v >= min_rssi for v in values]
    good = [v for v, k in zip(values, keep) if k]
    if len(good) >= 3:
        med = statistics.median(good)
        mad = statistics.median(abs(v - med) for v in good)
        limit = max(min_dev_db, mad_k * 1.4826 * mad)
        keep = [k and abs(v - med) <= limit for v, k in zip(values, keep)]
    return keep


def _std(xs: Sequence[float]) -> float:
    return statistics.stdev(xs) if len(xs) > 1 else 0.0


def aggregate(readings: Sequence[Beacon], min_rssi: float = -95.0,
              mad_k: float = 3.0, min_dev_db: float = 5.0
              ) -> Tuple[List[ApStats], List[bool]]:
    """Group readings by BSSID. Returns (per-AP stats, rejected flag per input reading)."""
    rejected = [True] * len(readings)
    groups = defaultdict(list)
    for i, r in enumerate(readings):
        if r.rssi_dbm is not None:
            groups[r.bssid].append(i)

    stats: List[ApStats] = []
    for bssid, idx in groups.items():
        rssis = [readings[i].rssi_dbm for i in idx]
        mask = keep_mask(rssis, min_rssi, mad_k, min_dev_db)
        kept = [readings[i] for i, k in zip(idx, mask) if k]
        for i, k in zip(idx, mask):
            rejected[i] = not k
        if not kept:
            continue

        kept_rssi = [float(r.rssi_dbm) for r in kept]
        noises = [float(r.noise_dbm) for r in kept if r.noise_dbm is not None]
        # An absent field, or the constant 0 some drivers emit, is not a measurement.
        noises = [n for n in noises if n != 0]
        utils = [float(r.chan_util_pct) for r in kept if r.chan_util_pct is not None]
        channels = [r.channel for r in kept if r.channel]
        ssids = [r.ssid for r in kept if r.ssid]
        nan = math.nan

        stats.append(ApStats(
            bssid=bssid,
            ssid=Counter(ssids).most_common(1)[0][0] if ssids else "",
            channel=Counter(channels).most_common(1)[0][0] if channels else None,
            n_samples=len(kept),
            n_rejected=len(idx) - len(kept),
            rssi_mean=statistics.fmean(kept_rssi),
            rssi_std=_std(kept_rssi),
            rssi_median=statistics.median(kept_rssi),
            rssi_min=min(kept_rssi),
            rssi_max=max(kept_rssi),
            noise_valid=bool(noises),
            noise_mean=statistics.fmean(noises) if noises else nan,
            noise_std=_std(noises) if noises else nan,
            chan_util_valid=bool(utils),
            chan_util_mean=statistics.fmean(utils) if utils else nan,
        ))
    stats.sort(key=lambda s: s.rssi_mean, reverse=True)
    return stats, rejected
