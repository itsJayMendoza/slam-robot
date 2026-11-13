"""Phase D: IDW and Ordinary Kriging per network on the shared grid, then the all-networks view."""
import math

import numpy as np

from slam_robot_interpolation.methods import idw

DEGENERATE_EPS = 1e-6


def _points(points):
    xy = np.array([[p["x"], p["y"]] for p in points], dtype=float)
    z = np.array([p["rssi"] for p in points], dtype=float)
    return xy, z


def run_idw(points, query, cfg):
    if len(points) < cfg.idw_min_points:
        return {"status": f"not enough points ({len(points)} of {cfg.idw_min_points})"}
    xy, z = _points(points)
    return {"status": "ok", "values": idw(xy, z, query, power=cfg.idw_power)}


def run_kriging(points, query, cfg):
    if len(points) < cfg.kriging_min_points:
        return {"status": f"not enough points ({len(points)} of {cfg.kriging_min_points})"}
    xy, z = _points(points)
    if np.ptp(z) == 0:
        return {"status": "ok (constant field)", "values": np.full(len(query), z[0]),
                "variance": np.zeros(len(query))}
    try:
        from pykrige.ok import OrdinaryKriging
        ok = OrdinaryKriging(xy[:, 0], xy[:, 1], z, variogram_model=cfg.kriging_model,
                             nlags=cfg.kriging_nlags, enable_plotting=False, verbose=False,
                             pseudo_inv=True)
        params = list(ok.variogram_model_parameters)
        if cfg.kriging_model in ("spherical", "exponential", "gaussian") and (
                params[0] <= DEGENERATE_EPS or params[1] <= DEGENERATE_EPS):
            return {"status": "Kriging failed: degenerate variogram (no spatial structure)",
                    "variogram": params}
        pred, var = ok.execute("points", query[:, 0], query[:, 1])
    except Exception as e:          # fit or solve failed: fall back to IDW only, as T1 specifies
        return {"status": f"Kriging failed: {type(e).__name__}: {e}"}
    pred = np.asarray(pred, dtype=float)
    if cfg.kriging_clip:
        pred = np.clip(pred, cfg.kriging_clip[0], cfg.kriging_clip[1])
    return {"status": "ok", "values": pred, "variance": np.maximum(np.asarray(var, float), 0.0),
            "variogram": params}


def all_networks(per_network: dict):
    """Highest estimate across the networks that ran, and the index of the winning network."""
    names = [n for n, r in per_network.items() if r.get("values") is not None]
    if not names:
        return names, None, None
    stack = np.vstack([per_network[n]["values"] for n in names])
    idx = np.argmax(stack, axis=0)
    return names, stack[idx, np.arange(stack.shape[1])], idx


def to_list(a, digits=2):
    if a is None:
        return None
    return [None if (v is None or (isinstance(v, float) and math.isnan(v))) else round(float(v), digits)
            for v in np.asarray(a, dtype=float)]
