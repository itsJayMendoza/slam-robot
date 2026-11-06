"""ROS 2 node exposing the scan as the ScanAtWaypoint action."""
import math
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from slam_robot_interfaces.action import ScanAtWaypoint
from slam_robot_interfaces.msg import ApReading, RawReading

from .aggregate import aggregate
from .capture import LiveScanner

NAN = math.nan


class WifiScannerNode(Node):
    def __init__(self):
        super().__init__("wifi_scanner")
        self.declare_parameter("interface", "wlx60156f273357")
        self.declare_parameter("channels", list(range(1, 14)))
        self.declare_parameter("channel_dwell_s", 0.2)
        self.declare_parameter("min_rssi_dbm", -95.0)
        self.declare_parameter("mad_k", 3.0)
        self.declare_parameter("min_dev_db", 5.0)
        self.declare_parameter("on_channel_only", False)

        p = self.get_parameter
        self.scanner = LiveScanner(p("interface").value,
                                   [int(c) for c in p("channels").value],
                                   p("channel_dwell_s").value)
        self.scanner.start()
        self._server = ActionServer(self, ScanAtWaypoint, "scan_at_waypoint", self._execute)
        self.get_logger().info(f"Scanning on {p('interface').value}; action: scan_at_waypoint")

    def _execute(self, goal_handle):
        dwell = float(goal_handle.request.dwell_s)
        p = self.get_parameter
        result = ScanAtWaypoint.Result()
        if dwell <= 0:
            goal_handle.abort()
            result.success, result.message = False, "dwell_s must be > 0"
            return result

        collector = self.scanner.collector
        collector.start()
        t0 = time.monotonic()
        while (elapsed := time.monotonic() - t0) < dwell:
            fb = ScanAtWaypoint.Feedback()
            fb.elapsed_s, fb.frames = float(elapsed), collector.count()
            goal_handle.publish_feedback(fb)
            time.sleep(min(0.25, max(0.0, dwell - elapsed)))
        readings = collector.stop()

        stats, rejected = aggregate(readings, p("min_rssi_dbm").value,
                                    p("mad_k").value, p("min_dev_db").value,
                                    p("on_channel_only").value)
        result.aps = [self._ap_msg(s) for s in stats]
        result.raw = [self._raw_msg(r, rej) for r, rej in zip(readings, rejected)]
        result.success = bool(stats)
        result.message = (f"{len(readings)} beacons, {sum(rejected)} rejected, {len(stats)} APs"
                          if stats else "no beacons received")
        goal_handle.succeed()
        return result

    @staticmethod
    def _ap_msg(s):
        m = ApReading()
        m.bssid, m.ssid, m.channel = s.bssid, s.ssid, int(s.channel or 0)
        m.n_samples, m.n_rejected = s.n_samples, s.n_rejected
        m.rssi_mean, m.rssi_std, m.rssi_median = s.rssi_mean, s.rssi_std, s.rssi_median
        m.rssi_min, m.rssi_max = s.rssi_min, s.rssi_max
        m.noise_valid, m.noise_mean, m.noise_std = s.noise_valid, s.noise_mean, s.noise_std
        m.chan_util_valid, m.chan_util_mean = s.chan_util_valid, s.chan_util_mean
        return m

    @staticmethod
    def _raw_msg(r, rejected):
        m = RawReading()
        m.t_unix, m.bssid, m.ssid, m.channel = float(r.t_unix), r.bssid, r.ssid, int(r.channel or 0)
        m.rssi_dbm = NAN if r.rssi_dbm is None else float(r.rssi_dbm)
        m.noise_valid = r.noise_dbm is not None and r.noise_dbm != 0
        m.noise_dbm = float(r.noise_dbm) if m.noise_valid else NAN
        m.chan_util_valid = r.chan_util_pct is not None
        m.chan_util_pct = float(r.chan_util_pct) if m.chan_util_valid else NAN
        m.rejected = rejected
        return m

    def destroy_node(self):
        self.scanner.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = WifiScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
