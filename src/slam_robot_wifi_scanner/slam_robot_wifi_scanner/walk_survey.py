"""Guided stop-and-scan walk, carrying the Pi + T3U by hand (no robot needed).

  sudo env "PYTHONPATH=$PYTHONPATH" python3 -m slam_robot_wifi_scanner.walk_survey \\
      --session 001 --floor 1 --line 1.55 0.5 --axis y --spacing 0.5 --count 5 \\
      --target "Yumi 2.4G" --monitor

At each waypoint: walk to the mark, face the heading, press Enter, hold still. Ctrl-C ends the
session as 'interrupted' and keeps everything recorded so far.
"""
import argparse
import math
import sys
import time
from pathlib import Path

from .aggregate import aggregate
from .capture import LiveScanner, set_monitor_mode
from .session import SessionWriter


def line_points(x0, y0, axis, spacing, count):
    dx, dy = (spacing, 0.0) if axis == "x" else (0.0, spacing)
    return [(round(x0 + i * dx, 3), round(y0 + i * dy, 3)) for i in range(count)]


def parse_points(text):
    """'1.0,1.0 2.0,1.0' -> [(1.0, 1.0), (2.0, 1.0)]"""
    return [tuple(float(v) for v in p.split(",")) for p in text.split()]


def label(wid, total):
    return f"WP-{wid:0{max(2, len(str(total)))}d}"


def summary(beacons, target):
    stats, _ = aggregate(beacons, min_rssi=-100.0, on_channel_only=True)
    lines = [f"  {len(beacons)} beacons, {len(stats)} APs (on-channel)"]
    for s in stats[:5]:
        mark = "  <- target" if target and s.ssid == target else ""
        lines.append(f"    {s.ssid[:22]:22} {s.rssi_median:6.1f} dBm  n={s.n_samples}{mark}")
    if target and not any(s.ssid == target for s in stats):
        lines.append(f"    target '{target}' not heard on-channel at this spot")
    return "\n".join(lines)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Guided walk survey with the Pi carried by hand")
    ap.add_argument("--session", required=True)
    ap.add_argument("--floor", default="1")
    ap.add_argument("--iface", default="wlx60156f273357")
    ap.add_argument("--monitor", action="store_true", help="put the adapter in monitor mode first")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--line", nargs=2, type=float, metavar=("X0", "Y0"), help="first point of a straight line")
    g.add_argument("--points", help="explicit points, e.g. '1.0,1.0 2.0,1.0'")
    ap.add_argument("--axis", choices=("x", "y"), default="y", help="direction of the line")
    ap.add_argument("--spacing", type=float, default=0.5)
    ap.add_argument("--count", type=int, default=5)
    ap.add_argument("--heading", type=float, help="degrees CCW from +x (default: along the line)")
    ap.add_argument("--dwell", type=float, default=10.0)
    ap.add_argument("--countdown", type=int, default=5)
    ap.add_argument("--target", help="SSID to highlight in the live summary")
    ap.add_argument("--out", help="session folder (default ~/sessions/<session>)")
    args = ap.parse_args(argv)

    if args.iface == "wlan0":
        sys.exit("Refusing to use wlan0: it carries the SSH connection. Use the T3U interface.")
    pts = line_points(args.line[0], args.line[1], args.axis, args.spacing, args.count) \
        if args.line else parse_points(args.points)
    heading = args.heading if args.heading is not None else (90.0 if args.axis == "y" else 0.0)
    out = Path(args.out or Path.home() / "sessions" / args.session)
    if (out / "notification.json").exists():
        sys.exit(f"{out} already holds a finished session; pick another --session or --out")

    if args.monitor:
        set_monitor_mode(args.iface)
    scanner = LiveScanner(args.iface, range(1, 14), 0.2)
    scanner.start()
    writer = SessionWriter(out, args.session, args.floor, args.spacing if args.line else None)
    for i, (x, y) in enumerate(pts, start=1):
        writer.add_waypoint(i, x, y)
    print(f"Session {args.session}: {len(pts)} waypoints, dwell {args.dwell:.0f} s, "
          f"face {heading:.0f} deg (0 = +x, 90 = +y). Files go to {out}")
    status = "complete"
    try:
        for i, (x, y) in enumerate(pts, start=1):
            input(f"\nGo to {label(i, len(pts))} at ({x:.2f}, {y:.2f}). Press Enter when there... ")
            for c in range(args.countdown, 0, -1):
                print(f"  hold still in {c}", end="\r", flush=True)
                time.sleep(1)
            print(f"  scanning {args.dwell:.0f} s...   ", flush=True)
            beacons = scanner.scan(args.dwell)
            writer.add_beacons(i, x, y, heading, beacons)
            print("\a" + summary(beacons, args.target))
    except (KeyboardInterrupt, EOFError):
        status = "interrupted"
        print("\nStopped early.")
    finally:
        scanner.stop()
        note = writer.finish(status)
    print(f"\nSession {note['status']}: {note['record_count']} records -> {out}")
    print("Files: records.json, waypoints.json, notification.json")


if __name__ == "__main__":
    main()
