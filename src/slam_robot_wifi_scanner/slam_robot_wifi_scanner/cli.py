"""Standalone scan for testing on the Pi without ROS:
   sudo env PYTHONPATH=$PYTHONPATH python3 -m slam_robot_wifi_scanner.cli --iface wlx... --dwell 5
"""
import argparse

from .aggregate import aggregate
from .capture import LiveScanner, set_monitor_mode


def main():
    ap = argparse.ArgumentParser(description="One passive WiFi scan, printed as a table")
    ap.add_argument("--iface", required=True)
    ap.add_argument("--dwell", type=float, default=5.0)
    ap.add_argument("--channel-dwell", type=float, default=0.2)
    ap.add_argument("--monitor", action="store_true", help="put the interface in monitor mode first")
    ap.add_argument("--min-rssi", type=float, default=-95.0)
    args = ap.parse_args()

    if args.monitor:
        set_monitor_mode(args.iface)
    scanner = LiveScanner(args.iface, range(1, 14), args.channel_dwell)
    scanner.start()
    try:
        readings = scanner.scan(args.dwell)
    finally:
        scanner.stop()

    stats, rejected = aggregate(readings, min_rssi=args.min_rssi)
    print(f"{len(readings)} beacons, {sum(rejected)} rejected, {len(stats)} APs, "
          f"noise reported: {any(r.noise_dbm is not None for r in readings)}")
    print(f"{'BSSID':17}  {'SSID':24} ch   n  rssi_mean  std  min  max  util%")
    for s in stats:
        util = f"{s.chan_util_mean:5.1f}" if s.chan_util_valid else "  n/a"
        print(f"{s.bssid:17}  {s.ssid[:24]:24} {s.channel or 0:2} {s.n_samples:3}  "
              f"{s.rssi_mean:8.1f} {s.rssi_std:4.1f} {s.rssi_min:4.0f} {s.rssi_max:4.0f}  {util}")


if __name__ == "__main__":
    main()
