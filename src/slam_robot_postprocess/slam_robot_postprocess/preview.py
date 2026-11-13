"""Quick-look heatmap of a processed session (needs matplotlib; meant for the laptop).

  heatmap_preview data/sessions/001/processed --map data/sessions/001/map/map.yaml \\
      --network "Yumi 2.4G" --out data/sessions/001/processed/preview_heatmap.png

Two panels on one colour scale: the chosen network, and the all-networks view (strongest network
per cell). Cells further than --extrap metres from every measured spot are faded and hatched,
because their values are extrapolated rather than measured.
"""
import argparse
import json
import re
from pathlib import Path

import numpy as np

from slam_robot_waypoints.grid import FREE, load_map_yaml

INK, INK2, MUTED, SURF, WALL = "#0b0b0b", "#52514e", "#898781", "#fcfcfb", "#c3c2b7"
BLUE_RAMP = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5",
             "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b"]


def _safe(name):
    return re.sub(r"[^A-Za-z0-9._-]+", "_", name).strip("_") or "network"


def free_bbox(map_yaml):
    """Bounding box (x0, y0, x1, y1) of the free floor on the map, in metres."""
    occ = load_map_yaml(map_yaml)
    rows, cols = np.nonzero(occ.data == FREE)
    r = occ.resolution
    return (occ.origin_x + cols.min() * r, occ.origin_y + rows.min() * r,
            occ.origin_x + (cols.max() + 1) * r, occ.origin_y + (rows.max() + 1) * r)


def render(processed, map_yaml, network, out, extrap_m=0.8, vmin=None, vmax=None, title=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LinearSegmentedColormap, ListedColormap
    from matplotlib.patches import Patch, Rectangle

    d = Path(processed)
    load = lambda n: json.loads((d / n).read_text())
    grid, sets, summary = load("grid.json"), load("sets.json"), load("summary.json")
    if network not in sets:
        raise SystemExit(f"network {network!r} not in this session; choose from {list(sets)}")
    A, U, C = grid["cells_across"], grid["cells_up"], grid["cell_size_m"]
    ox, oy = grid["origin_x"], grid["origin_y"]
    extent = [ox, ox + A * C, oy, oy + U * C]
    free = np.array(grid["free"], bool).reshape(U, A)

    def surface(name):
        v = np.array([np.nan if x is None else x for x in load(name)["values"]], float).reshape(U, A)
        v[~free] = np.nan
        return v

    one, alldoc = surface(f"IDW_{_safe(network)}.json"), load("IDW_ALL.json")
    allv = surface("IDW_ALL.json")
    strongest = np.array(alldoc["strongest"]).reshape(U, A)
    pts = sets[network]
    px, py = np.array([p["x"] for p in pts]), np.array([p["y"] for p in pts])
    xs, ys = ox + (np.arange(A) + 0.5) * C, oy + (np.arange(U) + 0.5) * C
    gx, gy = np.meshgrid(xs, ys)
    dist = np.min(np.hypot(gx[..., None] - px, gy[..., None] - py), axis=-1)
    extrap = (dist > extrap_m) & free
    finite = np.concatenate([one[np.isfinite(one)], allv[np.isfinite(allv)]])
    vmin = vmin if vmin is not None else 5 * np.floor(finite.min() / 5)
    vmax = vmax if vmax is not None else 5 * np.ceil(finite.max() / 5)
    x0, y0, x1, y1 = free_bbox(map_yaml)

    plt.rcParams.update({"hatch.color": INK2, "hatch.linewidth": 0.6, "font.size": 10,
                         "text.color": INK, "axes.labelcolor": INK2, "xtick.color": MUTED,
                         "ytick.color": MUTED, "axes.edgecolor": WALL})
    cmap = LinearSegmentedColormap.from_list("blue", BLUE_RAMP)
    fig, axes = plt.subplots(1, 2, figsize=(9.2, 7.6), facecolor=SURF, constrained_layout=True)
    for ax, v, label in ((axes[0], one, network), (axes[1], allv, "All networks (strongest at each cell)")):
        ax.set_facecolor(WALL)
        room = Rectangle((x0, y0), x1 - x0, y1 - y0, transform=ax.transData)
        im = ax.imshow(v, extent=extent, origin="lower", cmap=cmap, vmin=vmin, vmax=vmax,
                       interpolation="nearest")
        im.set_clip_path(room)
        veil = ax.imshow(np.where(extrap, 1.0, np.nan), extent=extent, origin="lower",
                         cmap=ListedColormap([SURF]), alpha=0.55, interpolation="nearest")
        veil.set_clip_path(room)
        h = ax.contourf(gx, gy, extrap.astype(float), levels=[0.5, 1.5], colors="none", hatches=["//"])
        h.set_clip_path(room)
        ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec=INK2, lw=1.0, zorder=2))
        ax.scatter(px, py, s=60, color=INK, edgecolors=SURF, linewidths=2, zorder=3)
        ax.set_title(label, loc="left", fontsize=11, color=INK, pad=8)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_xlim(extent[0], extent[1]); ax.set_ylim(extent[2], extent[3]); ax.set_aspect("equal")
    box = dict(boxstyle="round,pad=0.2", fc=SURF, ec="none", alpha=0.85)
    names = alldoc["networks"]
    for p in pts:
        j, i = int((p["y"] - oy) // C), int((p["x"] - ox) // C)
        axes[0].annotate(f"WP-{p['waypoint_id']}  {p['rssi']:.0f}", (p["x"], p["y"]), xytext=(10, -3),
                         textcoords="offset points", fontsize=9, color=INK, bbox=box)
        axes[1].annotate(f"{allv[j, i]:.0f}  {names[strongest[j, i]]}", (p["x"], p["y"]), xytext=(10, -3),
                         textcoords="offset points", fontsize=8.5, color=INK, bbox=box)
    cb = fig.colorbar(im, ax=axes, shrink=0.55, pad=0.02)
    cb.set_label("RSSI (dBm), IDW estimate"); cb.outline.set_visible(False)
    cb.ax.tick_params(color=MUTED, labelcolor=INK2)
    cb.set_ticks(np.arange(vmin, vmax + 1, 5))
    fig.legend(handles=[Patch(facecolor=WALL, label="wall / outside the room"),
                        Patch(facecolor="#e8eef6", edgecolor=MUTED, hatch="//",
                              label=f"extrapolated: more than {extrap_m:g} m from a measured spot"),
                        plt.Line2D([], [], marker="o", ls="", color=INK, markeredgecolor=SURF,
                                   markersize=8, label="measured spot (median, dBm)")],
               loc="lower center", ncol=3, frameon=False, fontsize=8.5, bbox_to_anchor=(0.45, -0.005))
    g = summary["grid"]
    fig.suptitle(title or f"Session {summary['session_id']}: {len(pts)} spots "
                          f"(IDW, power {summary['config']['idw_power']:g}, {g['cell_size_m']:g} m cells)",
                 x=0.02, ha="left", fontsize=12, color=INK)
    fig.savefig(out, dpi=150, facecolor=SURF, bbox_inches="tight")
    plt.close(fig)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description="Quick-look heatmap of a processed session")
    ap.add_argument("processed", help="the session's processed/ folder")
    ap.add_argument("--map", required=True, help="the floor's map.yaml")
    ap.add_argument("--network", required=True, help="network (SSID) for the left panel")
    ap.add_argument("--out", help="PNG path (default <processed>/preview_heatmap.png)")
    ap.add_argument("--extrap", type=float, default=0.8, help="metres from a measured spot")
    ap.add_argument("--title")
    args = ap.parse_args(argv)
    out = args.out or str(Path(args.processed) / "preview_heatmap.png")
    print(render(args.processed, args.map, args.network, out, args.extrap, title=args.title))


if __name__ == "__main__":
    main()
