"""Accuracy metrics, holdout, k-fold, and the inaccessible-zone / thinning experiment."""
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

from .methods import predict


def metrics(pred, truth) -> Dict[str, float]:
    """RMSE, MAE and bias (mean of prediction minus truth, so positive = over-estimates)."""
    e = np.asarray(pred, float) - np.asarray(truth, float)
    if e.size == 0:
        return {"rmse": float("nan"), "mae": float("nan"), "bias": float("nan"), "n": 0}
    return {"rmse": float(np.sqrt(np.mean(e ** 2))), "mae": float(np.mean(np.abs(e))),
            "bias": float(np.mean(e)), "n": int(e.size)}


def holdout_split(n: int, test_frac: float = 0.2, seed: int = 0) -> Tuple[np.ndarray, np.ndarray]:
    """(train_idx, test_idx). Typical test_frac is 0.20-0.25."""
    if not 0 < test_frac < 1:
        raise ValueError("test_frac must be between 0 and 1")
    perm = np.random.default_rng(seed).permutation(n)
    n_test = max(1, int(round(n * test_frac)))
    return np.sort(perm[n_test:]), np.sort(perm[:n_test])


def kfold_indices(n: int, k: int = 5, seed: int = 0) -> List[Tuple[np.ndarray, np.ndarray]]:
    if not 2 <= k <= n:
        raise ValueError("need 2 <= k <= n")
    folds = np.array_split(np.random.default_rng(seed).permutation(n), k)
    return [(np.sort(np.concatenate(folds[:i] + folds[i + 1:])), np.sort(folds[i]))
            for i in range(k)]


def evaluate_split(method: str, xy, z, train_idx, test_idx, **kw) -> Dict[str, float]:
    pred, _ = predict(method, xy[train_idx], z[train_idx], xy[test_idx], **kw)
    return metrics(pred, z[test_idx])


def cross_validate(method: str, xy, z, k: int = 5, seed: int = 0, **kw) -> Dict[str, float]:
    """k-fold CV. Metrics are computed over all held-out predictions pooled together."""
    xy, z = np.asarray(xy, float), np.asarray(z, float)
    preds, truths = [], []
    for train, test in kfold_indices(len(z), k, seed):
        p, _ = predict(method, xy[train], z[train], xy[test], **kw)
        preds.append(p)
        truths.append(z[test])
    return metrics(np.concatenate(preds), np.concatenate(truths))


@dataclass(frozen=True)
class Zone:
    """An inaccessible zone: a rectangle (xmin, ymin, xmax, ymax) or a circle (cx, cy, r)."""
    kind: str
    params: Tuple[float, ...]

    @staticmethod
    def rect(xmin, ymin, xmax, ymax):
        return Zone("rect", (xmin, ymin, xmax, ymax))

    @staticmethod
    def circle(cx, cy, r):
        return Zone("circle", (cx, cy, r))

    def distance_outside(self, xy) -> np.ndarray:
        """0 inside the zone, otherwise the distance to it in metres."""
        xy = np.asarray(xy, float)
        if self.kind == "rect":
            x0, y0, x1, y1 = self.params
            dx = np.maximum.reduce([x0 - xy[:, 0], np.zeros(len(xy)), xy[:, 0] - x1])
            dy = np.maximum.reduce([y0 - xy[:, 1], np.zeros(len(xy)), xy[:, 1] - y1])
            return np.hypot(dx, dy)
        cx, cy, r = self.params
        return np.maximum(np.hypot(xy[:, 0] - cx, xy[:, 1] - cy) - r, 0.0)

    def centre(self) -> Tuple[float, float]:
        if self.kind == "rect":
            x0, y0, x1, y1 = self.params
            return (x0 + x1) / 2, (y0 + y1) / 2
        return self.params[0], self.params[1]


def inaccessible_zone_test(method: str, xy, z, zone: Zone, keep_frac: float = 1.0,
                           band_m: float = 1.0, **kw) -> Dict[str, float]:
    """Experiment 6: hide the measurements inside `zone`, thin the ring of samples just
    outside it (within band_m) to keep_frac of 100/75/50/25 %, predict the hidden values from
    the rest, and score against the ground truth that was collected before masking.

    Thinning keeps evenly spaced samples around the ring (sorted by angle), so it is
    deterministic."""
    xy, z = np.asarray(xy, float), np.asarray(z, float)
    if not 0 < keep_frac <= 1:
        raise ValueError("keep_frac must be in (0, 1]")
    d = zone.distance_outside(xy)
    inside = d == 0
    ring = np.flatnonzero((d > 0) & (d <= band_m))
    cx, cy = zone.centre()
    ring = ring[np.argsort(np.arctan2(xy[ring, 1] - cy, xy[ring, 0] - cx))]
    n_keep = max(1, int(round(len(ring) * keep_frac))) if len(ring) else 0
    kept_ring = ring[np.unique(np.round(np.linspace(0, len(ring) - 1, n_keep)).astype(int))] \
        if n_keep else ring[:0]
    train = np.concatenate([np.flatnonzero(d > band_m), kept_ring])
    test = np.flatnonzero(inside)
    if len(test) == 0:
        raise ValueError("no samples inside the zone to score against")
    out = evaluate_split(method, xy, z, train, test, **kw)
    out.update(n_train=int(len(train)), ring_total=int(len(ring)), ring_kept=int(len(kept_ring)))
    return out


def thinning_experiment(method: str, xy, z, zone: Zone,
                        fractions: Sequence[float] = (1.0, 0.75, 0.5, 0.25),
                        band_m: float = 1.0, **kw) -> Dict[float, Dict[str, float]]:
    return {f: inaccessible_zone_test(method, xy, z, zone, f, band_m, **kw) for f in fractions}
