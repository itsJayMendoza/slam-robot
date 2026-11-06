"""Standalone scan for testing on the Pi without ROS:
   sudo env PYTHONPATH=$PYTHONPATH python3 -m slam_robot_wifi_scanner.cli --iface wlx... --dwell 10 --monitor
Prints each AP twice ('?' marks too few samples to trust): using every beacon, and using only beacons heard while tuned to the
AP's own channel, so the effect of the on-channel filter on the spread is visible.
"""
import argparse

from .aggregate import aggregate
from .capture import LiveScanner, set_monitor_mode


def _cell(s):
    if not s:
        return "  -      -    -  "
    return f"{s.n_samples:3d} {s.rssi_mean:6.1f} {s.rssi_std:4.1f}{'?' if s.low_confidence else ' '}"


def main():
    ap = argparse.ArgumentParser(description="One passive WiFi scan, printed as a table")
    ap.add_argument("--iface", required=True)
    ap.add_argument("--dwell", type=float, default=5.0)
    ap.add_argument("--channel-dwell", type=float, default=0.2)
    ap.add_argument("--monitor", action="store_true", help="put the interface in monitor mode first")
    ap.add_argument("--min-rssi", type=float, default=-95.0)
    ap.add_argument("--all-channels", action="store_true", help="blind 13-channel sweep (no adaptive hopping)")
    args = ap.parse_args()

    if args.monitor:
        set_monitor_mode(args.iface)
    scanner = LiveScanner(args.iface, range(1, 14), args.channel_dwell, adaptive=not args.all_channels)
    scanner.start()
    try:
        readings = scanner.scan(args.dwell)
    finally:
        scanner.stop()

    all_stats, rej_all = aggregate(readings, min_rssi=args.min_rssi)
    on_stats, rej_on = aggregate(readings, min_rssi=args.min_rssi, on_channel_only=True)
    on = {s.bssid: s for s in on_stats}
    print(f"{len(readings)} beacons; rejected {sum(rej_all)} (all frames) / {sum(rej_on)} "
          f"(on-channel only); noise reported: {any(r.noise_dbm is not None for r in readings)}")
    print(f"{'BSSID':17}  {'SSID':20} ch |  all frames: n  mean  std | on-channel: n  mean  std | util%")
    for s in all_stats:
        util = f"{s.chan_util_mean:5.1f}" if s.chan_util_valid else "  n/a"
        print(f"{s.bssid:17}  {s.ssid[:20]:20} {s.channel or 0:2} |  {_cell(s)}   |  {_cell(on.get(s.bssid))}  | {util}")


if __name__ == "__main__":
    main()
