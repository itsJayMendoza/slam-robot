"""The two interpolators. Both take sample points (n, 2) in metres and values (n,)."""
from typing import Optional, Tuple

import numpy as np
from scipy.spatial import cKDTree

VARIOGRAM_MODELS = ("linear", "power", "gaussian", "spherical", "exponential")


def idw(xy: np.ndarray, z: np.ndarray, query_xy: np.ndarray, power: float = 2.0,
        k: Optional[int] = None) -> np.ndarray:
    """Inverse distance weighting. k limits it to the k nearest samples (default: all).

    Exact at sample points. IDW never predicts outside the range of its samples."""
    xy, z, q = np.asarray(xy, float), np.asarray(z, float), np.asarray(query_xy, float)
    if len(z) == 0:
        raise ValueError("no samples")
    if power <= 0:
        raise ValueError("power must be > 0")
    kk = len(z) if k is None else min(int(k), len(z))
    dist, idx = cKDTree(xy).query(q, k=kk)
    dist, idx = dist.reshape(len(q), kk), idx.reshape(len(q), kk)
    out = np.empty(len(q))
    exact = dist[:, 0] < 1e-12
    out[exact] = z[idx[exact, 0]]
    rest = ~exact
    w = 1.0 / dist[rest] ** power
    out[rest] = (w * z[idx[rest]]).sum(axis=1) / w.sum(axis=1)
    return out


def ordinary_kriging(xy: np.ndarray, z: np.ndarray, query_xy: np.ndarray,
                     variogram_model: str = "spherical", nlags: int = 6,
                     variogram_parameters=None) -> Tuple[np.ndarray, np.ndarray]:
    """Ordinary Kriging via pykrige. Returns (prediction, prediction error variance)."""
    from pykrige.ok import OrdinaryKriging
    if variogram_model not in VARIOGRAM_MODELS:
        raise ValueError(f"variogram_model must be one of {VARIOGRAM_MODELS}")
    xy, z, q = np.asarray(xy, float), np.asarray(z, float), np.asarray(query_xy, float)
    if len(z) < 4:
        raise ValueError("Ordinary Kriging needs at least 4 samples")
    if np.ptp(z) == 0:                       # constant field: the variogram cannot be fitted
        return np.full(len(q), z[0]), np.zeros(len(q))
    ok = OrdinaryKriging(xy[:, 0], xy[:, 1], z, variogram_model=variogram_model,
                         variogram_parameters=variogram_parameters, nlags=nlags,
                         enable_plotting=False, verbose=False, pseudo_inv=True)
    pred, var = ok.execute("points", q[:, 0], q[:, 1])
    return np.asarray(pred, float), np.maximum(np.asarray(var, float), 0.0)


def predict(method: str, xy, z, query_xy, **kw) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Unified entry point: returns (prediction, variance or None)."""
    if method == "idw":
        return idw(xy, z, query_xy, power=kw.get("power", 2.0), k=kw.get("k")), None
    if method == "kriging":
        return ordinary_kriging(xy, z, query_xy,
                                variogram_model=kw.get("variogram_model", "spherical"),
                                nlags=kw.get("nlags", 6))
    raise ValueError("method must be 'idw' or 'kriging'")
